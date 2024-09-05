// SPDX-License-Identifier: MIT License
/*
 * IIO driver for ADS122C04 chip
 *
 *
 * Author: Aluisio Leonello Victal <alvictal@gmail.com>
 *
 * Datasheet for ADS122C04 can be found here:
 * https://www.ti.com/lit/ds/symlink/ads122c04.pdf
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define ADS122C04_DRV_NAME "ads122c04"

#define ADS122C04_CHANNELS 16

#define ADS1122C04_CFG0_REG	0x00
#define ADS1122C04_CFG1_REG	0x01
#define ADS1122C04_CFG2_REG	0x02
#define ADS1122C04_CFG3_REG	0x03

enum chip_ids {
	ADSXXXXXXX = 0,
	ADS1122C04,
};

enum ads122c04_channels {
	ADS1122C04_AIN0_AIN1 = 0,
	ADS1122C04_AIN0_AIN2,
	ADS1122C04_AIN0_AIN3,
    ADS1122C04_AIN1_AIN0,
    ADS1122C04_AIN1_AIN2,
    ADS1122C04_AIN1_AIN3,
    ADS1122C04_AIN2_AIN3,
    ADS1122C04_AIN3_AIN2,
    ADS1122C04_AIN0_AVSS,
    ADS1122C04_AIN1_AVSS,
    ADS1122C04_AIN2_AVSS,
    ADS1122C04_AIN3_AVSS,
    ADS1122C04_VREF_MON,
    ADS1122C04_AVDD_AVSS,
    ADS1122C04_AINP_AINN_SHORTED,
    ADS1122C04_RESERVED,
};

enum ads122c04_vref {
    ADS1122C04_VREF_INTERNAL = 0,
    ADS1122C04_VREF_EXTERNAL,
    ADS1122C04_VREF_ANALOG1,
    ADS1122C04_VREF_ANALOG2,
};

enum ads122c04_integrity_check {
    ADS1122C04_ICHECK_DISABLED = 0,
    ADS1122C04_ICHECK_INVERTED,
    ADS1122C04_ICHECK_CRC16,
    ADS1122C04_ICHECK_RESERVED,
};

enum ads122c04_idac_rounting {
    ADS1122C04_IDAC_DISABLED = 0,
    ADS1122C04_IDAC_AIN0,
    ADS1122C04_IDAC_AIN1,
    ADS1122C04_IDAC_AIN2,
    ADS1122C04_IDAC_AIN3,
    ADS1122C04_IDAC_REFP,
    ADS1122C04_IDAC_REFN,
    ADS1122C04_IDAC_RESERVED,
};

/* Turbo mode multiply data_rate x2*/
static const unsigned int ads122c04_gain_conf[] = {
	1, 2, 4, 8, 16, 32, 64, 128
};

static const unsigned int ads122c04_data_rate[] = {
	20, 45, 90, 175, 330, 600, 1000
};

static const unsigned int ads122c04_idac_current[] = {
	0, 10, 50, 100, 250, 500, 1000, 1500
};

#define ADS1122C04_V_CHAN(_chan, _addr) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
	.datasheet_name = "AIN"#_chan,				\
}

#define ADS1122C04_V_DIFF_CHAN(_chan, _chan2, _addr) {		\
	.type = IIO_VOLTAGE,					\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}



struct ads122c04_channel_data {
	bool is_channel_enabled; /* is channel enabled */
    bool is_pga_bypass_disabled;
    unsigned short int gain_conf; /* gain configuration */
    unsigned short int data_rate;
    unsigned short int idac_1_routing;
    unsigned short int idac_2_routing;
};




struct ads122c04_data {
	struct regmap *regmap;
	/*
	 * Protects ADC ops, e.g: concurrent sysfs/buffered
	 * data reads, configuration updates
	 */
	struct mutex lock;
	struct ads122c04_channel_data channel_data[ADS122C04_CHANNELS];

	unsigned int event_channel;

	unsigned int *data_rate;

    unsigned short int voltage_reference; /* Voltage reference Selection */
    unsigned short int data_integrity;
    unsigned short int idac_current;

    bool conv_invalid; /* Invalid in case of changing from mode operand*/
    bool is_continues_mode_enabled;  /* Conversion Mode */
    bool is_turbo_mode_enabled; /* Operating mode */
    bool is_temperature_mode_enabled; /* Temperature sensor mode*/
    bool is_counter_enabled; /* data counter enabled */   
    bool is_burnout_enabled; /* Burn-out current sources. */
};



#ifdef CONFIG_PM
static int ads122c04_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads122c04_data *data = iio_priv(indio_dev);

	return 0;
}

static int ads122c04_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads122c04_data *data = iio_priv(indio_dev);
	int ret;

	if (!ret)
		data->conv_invalid = true;

	return ret;
}
#endif


static int ads122c04_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122c04_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static int ads122c04_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    return 0;
}

static const struct dev_pm_ops ads122c04_pm_ops = {
	SET_RUNTIME_PM_OPS(ads122c04_runtime_suspend,
			   ads122c04_runtime_resume, NULL)
};

static const struct i2c_device_id ads122c04_id[] = {
	{"ads122c04", ADS1122C04},
	{}
};
MODULE_DEVICE_TABLE(i2c, ads1015_id);


static const struct of_device_id ads122c04_of_match[] = {
	{
		.compatible = "ti,ads122c04",
		.data = (void *)ADS1122C04
	},
	{}
};
MODULE_DEVICE_TABLE(of, ads122c04_of_match);

static struct i2c_driver ads122c04_driver = {
	.driver = {
		.name = ADS122C04_DRV_NAME,
		.of_match_table = ads122c04_of_match,
		.pm = &ads1015_pm_ops,
	},
	.probe		= ads122c04_probe,
	.remove		= ads122c04_remove,
	.id_table	= ads122c04_id,
};

module_i2c_driver(ads122c04_driver);

MODULE_AUTHOR("Aluisio Leonello Victal <alvictal@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS122C04 ADC driver");
MODULE_LICENSE("MIT");
