/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ROCKNIX joypad driver
 *
 * Copyright (C) 2024 ROCKNIX (https://github.com/ROCKNIX)
 */

/*----------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/input-polldev.h>
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0))
#include <linux/of_gpio.h>
#else
#include <linux/of_gpio_legacy.h>
#endif
#include <linux/delay.h>
#include <linux/pwm.h>
#include "rocknix-joypad.h"

#include <linux/kthread.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/fs.h>
#include <linux/termios.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

/*----------------------------------------------------------------------------*/
#define DRV_NAME "rocknix-joypad"

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
#define	ADC_MAX_VOLTAGE		1800
#define	ADC_DATA_TUNING(x, p)	((x * p) / 100)
#define	ADC_TUNING_DEFAULT	180
#define	CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

struct bt_adc {
	/* report value (mV) */
	int value;
	/* report type */
	int report_type;
	/* input device init value (mV) */
	int max, min;
	/* calibrated adc value */
	int cal;
	/*  adc scale value */
	int scale;
	/* invert report */
	bool invert;
	/* adc channel */
	int channel;
	/* adc data tuning value([percent), p = positive, n = negative */
	int tuning_p, tuning_n;
};

struct bt_gpio {
	/* GPIO Request label */
	const char *label;
	/* GPIO Number */
	int num;
	/* report type */
	int report_type;
	/* report linux code */
	int linux_code;
	/* prev button value */
	bool old_value;
	/* button press level */
	bool active_level;
};

struct joypad {
	struct device *dev;
	struct input_polled_dev	*poll_dev;
	int poll_interval;

	/* report enable/disable */
	bool enable;

	/* analog mux & joystick control */
	struct iio_channel *adc_ch[SARADC_CH_NUM];

	/* adc input channel count */
	int chan_count;
	/* analog button */
	struct bt_adc *adcs;

	/* report reference point */
	bool invert_absx;
	bool invert_absy;
	bool invert_absrx;
	bool invert_absry;
	bool invert_absz;
	bool invert_absrz;
	bool skip_absr;

	/* report interval (ms) */
	int bt_gpio_count;
	struct bt_gpio *gpios;

	/* button auto repeat */
	int auto_repeat;

	/* report threshold (mV) */
	int bt_adc_fuzz, bt_adc_flat;
	/* adc read value scale */
	int bt_adc_scale;
	/* joystick deadzone control */
	int bt_adc_deadzone;

	struct mutex lock;

	/* pwm device for rumble*/
	struct input_dev *input;
	struct pwm_device *pwm;
	struct work_struct play_work;
	u16 level;
	u16 boost_weak;
	u16 boost_strong;
	bool has_rumble;
	bool rumble_enabled; /* to turn rumble on/off if a device has it */
	int rumble_gpio;
};

static int pwm_vibrator_start(struct joypad *joypad)
{
	struct pwm_state state;
	int err;

	pwm_get_state(joypad->pwm, &state);
	pwm_set_relative_duty_cycle(&state, joypad->level, 0xffff);
	state.enabled = true;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
	err = pwm_apply_state(joypad->pwm, &state);
#else
	err = pwm_apply_might_sleep(joypad->pwm, &state);
#endif
	if (err) {
		 dev_err(joypad->dev, "failed to apply pwm state: %d", err);
		 return err;
	}

	return 0;
}

static int joypad_vibrator_start(struct joypad *joypad)
{
	if (joypad->rumble_gpio) {
		gpio_set_value(joypad->rumble_gpio, 1);
		return 0;
	}
	return pwm_vibrator_start(joypad);
}

static void pwm_vibrator_stop(struct joypad *joypad)
{
	if (joypad->rumble_gpio) {
		gpio_set_value(joypad->rumble_gpio, 0);
		return;
	}

	pwm_disable(joypad->pwm);
}


static void joypad_vibrator_stop(struct joypad *joypad)
{
	pwm_vibrator_stop(joypad);
}

static void joypad_vibrator_play_work(struct work_struct *work)
{
	struct joypad *joypad = container_of(work,
					    struct joypad, play_work);
	mutex_lock(&joypad->lock);
	if (!joypad->rumble_enabled) {
		joypad_vibrator_stop(joypad);
		mutex_unlock(&joypad->lock);
		return;
	}

	if (joypad->level)
		 joypad_vibrator_start(joypad);
	else
		 joypad_vibrator_stop(joypad);

	mutex_unlock(&joypad->lock);
}

/*----------------------------------------------------------------------------*/
static int joypad_adc_read(struct joypad *joypad, struct bt_adc *adc)
{
	int value;

	if (iio_read_channel_processed(joypad->adc_ch[adc->channel], &value) < 0)
		return 0;

	value *= adc->scale;

	return value;
}

/*----------------------------------------------------------------------------*/
static void joypad_gpio_check(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn, value;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];

		if (gpio_get_value_cansleep(gpio->num) < 0) {
			dev_err(joypad->dev, "failed to get gpio state\n");
			continue;
		}
		value = gpio_get_value(gpio->num);
		if (value != gpio->old_value) {
			input_event(poll_dev->input,
				gpio->report_type,
				gpio->linux_code,
				(value == gpio->active_level) ? 1 : 0);
			gpio->old_value = value;
		}
	}
	input_sync(poll_dev->input);
}

/*----------------------------------------------------------------------------*/
static void joypad_adc_check(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn;
	int mag;
	int start_index = joypad->skip_absr ? 2 : 0;

	/* Assumes an even number of axes and that joystick axis pairs are sequential */
	/* e.g. left stick Y immediately follows left stick X */
	for (nbtn = start_index; nbtn < joypad->chan_count; nbtn += 2) {
		struct bt_adc *adcx = &joypad->adcs[nbtn];
		struct bt_adc *adcy = &joypad->adcs[nbtn + 1];

		/* Read first joystick axis */
		adcx->value = joypad_adc_read(joypad, adcx);
		if (!adcx->value) {
			dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
				__func__, nbtn);
			continue;
		}
		adcx->value = adcx->value - adcx->cal;

		/* Read second joystick axis */
		adcy->value = joypad_adc_read(joypad, adcy);
		if (!adcy->value) {
			dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
				__func__, nbtn + 1);
			continue;
		}
		adcy->value = adcy->value - adcy->cal;

		/* Scaled Radial Deadzone */
		/* https://web.archive.org/web/20190129113357/http://www.third-helix.com/2013/04/12/doing-thumbstick-dead-zones-right.html */
		mag = int_sqrt((adcx->value * adcx->value) + (adcy->value * adcy->value));
		if (joypad->bt_adc_deadzone) {
			if (mag <= joypad->bt_adc_deadzone) {
				adcx->value = 0;
				adcy->value = 0;
			}
			else {
				/* Assumes adcx->max == -adcx->min == adcy->max == -adcy->min */
				/* Order of operations is critical to avoid integer overflow */
				adcx->value = (((adcx->max * adcx->value) / mag) * (mag - joypad->bt_adc_deadzone)) / (adcx->max - joypad->bt_adc_deadzone);
				adcy->value = (((adcy->max * adcy->value) / mag) * (mag - joypad->bt_adc_deadzone)) / (adcy->max - joypad->bt_adc_deadzone);
			}
		}

		/* adc data tuning */
		if (adcx->tuning_n && adcx->value < 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_n);
		if (adcx->tuning_p && adcx->value > 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_p);
		if (adcy->tuning_n && adcy->value < 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_n);
		if (adcy->tuning_p && adcy->value > 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_p);

		/* Clamp to [min, max] */
		adcx->value = adcx->value > adcx->max ? adcx->max : adcx->value;
		adcx->value = adcx->value < adcx->min ? adcx->min : adcx->value;
		adcy->value = adcy->value > adcy->max ? adcy->max : adcy->value;
		adcy->value = adcy->value < adcy->min ? adcy->min : adcy->value;

		input_report_abs(poll_dev->input,
			adcx->report_type,
			adcx->invert ? adcx->value * (-1) : adcx->value);
		input_report_abs(poll_dev->input,
			adcy->report_type,
			adcy->invert ? adcy->value * (-1) : adcy->value);
	}
	input_sync(poll_dev->input);
}

/*----------------------------------------------------------------------------*/
static void joypad_poll(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	if (joypad->enable) {
		joypad_adc_check(poll_dev);
		joypad_gpio_check(poll_dev);
	}
	if (poll_dev->poll_interval != joypad->poll_interval) {
		mutex_lock(&joypad->lock);
		poll_dev->poll_interval = joypad->poll_interval;
		mutex_unlock(&joypad->lock);
	}
}

/*----------------------------------------------------------------------------*/
static void joypad_open(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		gpio->old_value = gpio->active_level ? 0 : 1;
	}
	for (nbtn = 0; nbtn < joypad->chan_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		adc->value = joypad_adc_read(joypad, adc);
		if (!adc->value) {
			dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
				__func__, nbtn);
			continue;
		}
		adc->cal = adc->value;
		dev_info(joypad->dev, "%s : adc[%d] adc->cal = %d\n",
			__func__, nbtn, adc->cal);
	}
	/* buttons status sync */
	joypad_adc_check(poll_dev);
	joypad_gpio_check(poll_dev);

	/* button report enable */
	mutex_lock(&joypad->lock);
	joypad->enable = true;
	mutex_unlock(&joypad->lock);

	dev_info(joypad->dev, "%s : opened\n", __func__);
}

/*----------------------------------------------------------------------------*/
static void joypad_close(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		joypad_vibrator_stop(joypad);
	}

	/* button report disable */
	mutex_lock(&joypad->lock);
	joypad->enable = false;
	mutex_unlock(&joypad->lock);

	dev_info(joypad->dev, "%s : closed\n", __func__);
}

/*----------------------------------------------------------------------------*/
static int joypad_iochannel_setup(struct device *dev, struct joypad *joypad)
{
	enum iio_chan_type type;
	unsigned char cnt;
	const char *uname;
	int ret;

	for (cnt = 0; cnt < joypad->chan_count; cnt++) {

		ret = of_property_read_string_index(dev->of_node,
				"io-channel-names", cnt, &uname);
		if (ret < 0) {
			dev_err(dev, "invalid channel name index[%d]\n", cnt);
			return -EINVAL;
		}

		joypad->adc_ch[cnt] = devm_iio_channel_get(dev,
				uname);
		if (IS_ERR(joypad->adc_ch[cnt])) {
			dev_err(dev, "iio channel get error\n");
			return -EINVAL;
		}
		if (!joypad->adc_ch[cnt]->indio_dev)
			return -ENXIO;

		if (iio_get_channel_type(joypad->adc_ch[cnt], &type))
			return -EINVAL;

		if (type != IIO_VOLTAGE) {
			dev_err(dev, "Incompatible channel type %d\n", type);
			return -EINVAL;
		}
	}
	return ret;
}

/*----------------------------------------------------------------------------*/
static int joypad_adc_setup(struct device *dev, struct joypad *joypad)
{
	int nbtn;

	/* adc button struct init */
	joypad->adcs = devm_kzalloc(dev, joypad->chan_count *
				sizeof(struct bt_adc), GFP_KERNEL);
	if (!joypad->adcs) {
		dev_err(dev, "%s devm_kzmalloc error!", __func__);
		return -ENOMEM;
	}

	for (nbtn = 0; nbtn < joypad->chan_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		adc->scale = joypad->bt_adc_scale;

		adc->max = (ADC_MAX_VOLTAGE / 2);
		adc->min = (ADC_MAX_VOLTAGE / 2) * (-1);
		if (adc->scale) {
			adc->max *= adc->scale;
			adc->min *= adc->scale;
		}
		adc->channel = nbtn;
		adc->invert = false;

		switch (nbtn) {
			case 0:
				if (joypad->invert_absry)
					adc->invert = true;
				adc->report_type = ABS_RY;
				if (device_property_read_u32(dev,
					"abs_ry-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_ry-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 1:
				if (joypad->invert_absrx)
					adc->invert = true;
				adc->report_type = ABS_RX;
				if (device_property_read_u32(dev,
					"abs_rx-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_rx-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 2:
				if (joypad->invert_absy)
					adc->invert = true;
				adc->report_type = ABS_Y;
				if (device_property_read_u32(dev,
					"abs_y-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_y-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 3:
				if (joypad->invert_absx)
					adc->invert = true;
				adc->report_type = ABS_X;
				if (device_property_read_u32(dev,
					"abs_x-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_x-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 4:
				if (joypad->invert_absz)
					adc->invert = true;
				adc->report_type = ABS_Z;
				if (device_property_read_u32(dev,
					"abs_z-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_z-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 5:
				if (joypad->invert_absrz)
					adc->invert = true;
				adc->report_type = ABS_RZ;
				if (device_property_read_u32(dev,
					"abs_rz-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_rz-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			default :
				dev_err(dev, "%s io channel count(%d) error!",
					__func__, nbtn);
				return -EINVAL;
		}
	}
	return	0;
}

/*----------------------------------------------------------------------------*/
static int rumble_play_effect(struct input_dev *dev, void *data, struct ff_effect *effect)
{
	struct joypad *joypad = data;
	u32 boosted_level;
	if (effect->type != FF_RUMBLE)
		 return 0;

	mutex_lock(&joypad->lock);
	if (!joypad->rumble_enabled) {
		mutex_unlock(&joypad->lock);
		return 0; // Ignore rumble effects if disabled
	}

	if (effect->u.rumble.strong_magnitude)
		boosted_level = effect->u.rumble.strong_magnitude + joypad->boost_strong;
	else
		boosted_level = effect->u.rumble.weak_magnitude + joypad->boost_weak;

	joypad->level = (u16)CLAMP(boosted_level, 0, 0xffff);

	mutex_unlock(&joypad->lock);

	schedule_work(&joypad->play_work);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_rumble_setup(struct device *dev, struct joypad *joypad)
{
	int error;
	struct pwm_state state;

	INIT_WORK(&joypad->play_work, joypad_vibrator_play_work);

	if (joypad->rumble_gpio)
		return 0;

	joypad->pwm = devm_pwm_get(dev, "enable");
	if (IS_ERR(joypad->pwm)) {
		dev_err(dev, "rumble get error\n");
		return -EINVAL;
	}

	/* Sync up PWM state and ensure it is off. */
	pwm_init_state(joypad->pwm, &state);
	state.enabled = false;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
	error = pwm_apply_state(joypad->pwm, &state);
#else
	error = pwm_apply_might_sleep(joypad->pwm, &state);
#endif
	if (error) {
		 dev_err(dev, "failed to apply initial PWM state: %d",
			 error);
		 return error;
	}

	dev_info(dev, "rumble setup success!\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_gpio_setup(struct device *dev, struct joypad *joypad)
{
	struct device_node *node, *pp;
	int nbtn;

	node = dev->of_node;
	if (!node)
		return -ENODEV;

	joypad->gpios = devm_kzalloc(dev, joypad->bt_gpio_count *
				sizeof(struct bt_gpio), GFP_KERNEL);

	if (!joypad->gpios) {
		dev_err(dev, "%s devm_kzmalloc error!", __func__);
		return -ENOMEM;
	}

	nbtn = 0;
	for_each_child_of_node(node, pp) {
		enum of_gpio_flags flags;
		struct bt_gpio *gpio = &joypad->gpios[nbtn++];
		int error;

		gpio->num = of_get_gpio_flags(pp, 0, &flags);
		if (gpio->num < 0) {
			error = gpio->num;
			dev_err(dev, "Failed to get gpio flags, error: %d\n",
				error);
			return error;
		}

		/* gpio active level(key press level) */
		gpio->active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

		gpio->label = of_get_property(pp, "label", NULL);

		if (gpio_is_valid(gpio->num)) {
			error = devm_gpio_request_one(dev, gpio->num,
						      GPIOF_IN, gpio->label);
			if (error < 0) {
				dev_err(dev,
					"Failed to request GPIO %d, error %d\n",
					gpio->num, error);
				return error;
			}
		}
		if (of_property_read_u32(pp, "linux,code", &gpio->linux_code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				gpio->num);
			return -EINVAL;
		}
		if (of_property_read_u32(pp, "linux,input-type",
				&gpio->report_type))
			gpio->report_type = EV_KEY;
	}
	if (nbtn == 0)
		return -EINVAL;

	return	0;
}

/*----------------------------------------------------------------------------*/
static int joypad_input_setup(struct device *dev, struct joypad *joypad)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input;
	int nbtn, error;
	u32 joypad_vendor = 0;
	u32 joypad_revision = 0;
	u32 joypad_product = 0;

	poll_dev = devm_input_allocate_polled_device(dev);
	if (!poll_dev) {
		dev_err(dev, "no memory for polled device\n");
		return -ENOMEM;
	}

	poll_dev->private	= joypad;
	poll_dev->poll		= joypad_poll;
	poll_dev->poll_interval	= joypad->poll_interval;
	poll_dev->open		= joypad_open;
	poll_dev->close		= joypad_close;

	input = poll_dev->input;

	input->name = DRV_NAME;

	device_property_read_string(dev, "joypad-name", &input->name);
	input->phys = DRV_NAME"/input0";

	device_property_read_u32(dev, "joypad-vendor", &joypad_vendor);
	device_property_read_u32(dev, "joypad-revision", &joypad_revision);
	device_property_read_u32(dev, "joypad-product", &joypad_product);
	//input->id.bustype = BUS_HOST;
	input->id.bustype = BUS_USB;
	input->id.vendor  = (u16)joypad_vendor;
	input->id.product = (u16)joypad_product;
	input->id.version = (u16)joypad_revision;

	/* IIO ADC key setup (0 mv ~ 1800 mv) * adc->scale */
	__set_bit(EV_ABS, input->evbit);

	// Set mapped ones on dt
	for(nbtn = 0; nbtn < joypad->chan_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		input_set_abs_params(input, adc->report_type,
				adc->min, adc->max,
				joypad->bt_adc_fuzz,
				joypad->bt_adc_flat);
		dev_info(dev,
			"%s : SCALE = %d, ABS min = %d, max = %d,"
			" fuzz = %d, flat = %d, deadzone = %d\n",
			__func__, adc->scale, adc->min, adc->max,
			joypad->bt_adc_fuzz, joypad->bt_adc_flat,
			joypad->bt_adc_deadzone);
		dev_info(dev,
			"%s : adc tuning_p = %d, adc_tuning_n = %d\n\n",
			__func__, adc->tuning_p, adc->tuning_n);
	}

	/* GPIO key setup */
	__set_bit(EV_KEY, input->evbit);
	for(nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		input_set_capability(input, gpio->report_type,
				gpio->linux_code);
	}

	/* Rumble setup */
	if (joypad->has_rumble) {
		u32 boost_weak = 0;
		u32 boost_strong = 0;
		device_property_read_u32(dev, "rumble-boost-weak", &boost_weak);
		device_property_read_u32(dev, "rumble-boost-strong", &boost_strong);
		joypad->boost_weak = boost_weak;
		joypad->boost_strong = boost_strong;
		dev_info(dev, "Boost = %d, %d", boost_weak, boost_strong);
		input_set_capability(input, EV_FF, FF_RUMBLE);
		error = input_ff_create_memless(input, joypad, rumble_play_effect);
		if (error) {
			dev_err(dev, "unable to register rumble, err=%d\n",
				error);
			return error;
		}
	}

	if (joypad->auto_repeat)
		__set_bit(EV_REP, input->evbit);

	joypad->dev = dev;

	error = input_register_polled_device(poll_dev);
	if (error) {
		dev_err(dev, "unable to register polled device, err=%d\n",
			error);
		return error;
	}
	joypad->dev = dev;
	joypad->poll_dev = poll_dev;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_dt_parse(struct device *dev, struct joypad *joypad)
{
	int error = 0;
	enum of_gpio_flags flags;

	/* initialize values from device-tree */
	device_property_read_u32(dev, "button-adc-fuzz",
				&joypad->bt_adc_fuzz);
	device_property_read_u32(dev, "button-adc-flat",
				&joypad->bt_adc_flat);
	device_property_read_u32(dev, "button-adc-scale",
				&joypad->bt_adc_scale);
	device_property_read_u32(dev, "button-adc-deadzone",
				&joypad->bt_adc_deadzone);

	joypad->chan_count = of_property_count_strings(dev->of_node,
		"io-channel-names");

	device_property_read_u32(dev, "poll-interval",
				&joypad->poll_interval);

	joypad->auto_repeat = device_property_present(dev, "autorepeat");
	joypad->skip_absr = device_property_present(dev, "skip-absr");
	dev_info(dev, "%s : skip-absr = %d\n", __func__, joypad->skip_absr);

	/* change the report reference point? (ADC MAX - read value) */
	joypad->invert_absx = device_property_present(dev, "invert-absx");
	joypad->invert_absy = device_property_present(dev, "invert-absy");
	joypad->invert_absz = device_property_present(dev, "invert-absz");
	joypad->invert_absrx = device_property_present(dev, "invert-absrx");
	joypad->invert_absry = device_property_present(dev, "invert-absry");
	joypad->invert_absrz = device_property_present(dev, "invert-absrz");
	dev_info(dev, "%s : invert-absx = %d, inveret-absy = %d, invert-absz = %d, invert-absrx = %d, invert-absry = %d, invert-absrz = %d\n",
		__func__, joypad->invert_absx, joypad->invert_absy, joypad->invert_absz, joypad->invert_absrx, joypad->invert_absry, joypad->invert_absrz);

	joypad->bt_gpio_count = device_get_child_node_count(dev);

	if ((joypad->chan_count == 0) || (joypad->bt_gpio_count == 0)) {
		dev_err(dev, "adc key = %d, gpio key = %d error!",
			joypad->chan_count, joypad->bt_gpio_count);
		return -EINVAL;
	}

	error = joypad_adc_setup(dev, joypad);
	if (error)
		return error;

	error = joypad_iochannel_setup(dev, joypad);
	if (error)
		return error;

	error = joypad_gpio_setup(dev, joypad);
	if (error)
		return error;

	joypad->rumble_gpio = of_get_named_gpio_flags(dev->of_node, "rumble-gpio", 0, &flags);
	if (gpio_is_valid(joypad->rumble_gpio)) {
		error = devm_gpio_request(dev, joypad->rumble_gpio, "rumble-gpio");
		if (error < 0) {
			dev_err(dev, "%s : failed to request rumble gpio %d\n",
				__func__, joypad->rumble_gpio);
			return error;
		}
		error = gpio_direction_output(joypad->rumble_gpio, 0);
		if (error < 0)
			return error;
		joypad->has_rumble = true;
		dev_info(dev, "%s : has gpio rumble\n", __func__);
	} else {
		dev_err(dev, "joypad rumble gpio error!");
	}

	dev_info(dev, "%s : adc key cnt = %d, gpio key cnt = %d\n",
			__func__, joypad->chan_count, joypad->bt_gpio_count);

	return error;
}

/*----------------------------------------------------------------------------*/
static int joypad_probe(struct platform_device *pdev)
{
	struct joypad *joypad;
	struct device *dev = &pdev->dev;
	int error;

	joypad = devm_kzalloc(dev, sizeof(struct joypad), GFP_KERNEL);
	if (!joypad) {
		dev_err(dev, "joypad devm_kzmalloc error!");
		return -ENOMEM;
	}

	/* device tree data parse */
	error = joypad_dt_parse(dev, joypad);
	if (error) {
		dev_err(dev, "dt parse error!(err = %d)\n", error);
		return error;
	}

	mutex_init(&joypad->lock);
	platform_set_drvdata(pdev, joypad);

	/* poll input device setup */
	error = joypad_input_setup(dev, joypad);
	if (error) {
		dev_err(dev, "input setup failed!(err = %d)\n", error);
		return error;
	}

        if (joypad->has_rumble) {
		error = joypad_rumble_setup(dev, joypad);
		if (error) {
			 dev_err(dev, "rumble setup failed!(err = %d)\n", error);
			 return error;
		}

		joypad->rumble_enabled = true;
	}

	dev_info(dev, "%s : probe success\n", __func__);
	return 0;
}

/*----------------------------------------------------------------------------*/
static const struct of_device_id joypad_of_match[] = {
	{ .compatible = "rocknix-joypad", },
	{},
};

MODULE_DEVICE_TABLE(of, joypad_of_match);

/*----------------------------------------------------------------------------*/
static struct platform_driver joypad_driver = {
	.probe = joypad_probe,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 13, 0))
	.remove_new = joypad_remove,
#else
	.remove = joypad_remove,
#endif
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(joypad_of_match),
	},
};

/*----------------------------------------------------------------------------*/
static int __init joypad_init(void)
{
	return platform_driver_register(&joypad_driver);
}

/*----------------------------------------------------------------------------*/
static void __exit joypad_exit(void)
{
	platform_driver_unregister(&joypad_driver);
}

/*----------------------------------------------------------------------------*/
late_initcall(joypad_init);
module_exit(joypad_exit);

/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("ROCKNIX");
MODULE_DESCRIPTION("ROCKNIX joypad driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_INFO(intree, "Y");

/*----------------------------------------------------------------------------*/
