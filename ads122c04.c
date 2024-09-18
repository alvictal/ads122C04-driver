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

#define ADC122C04_CHANNELS 12

#define ADC122C04_POWERDOWN	    0x02    /* 0000 001X */
#define ADC122C04_RESET         0x06    /* 0000 011X */
#define ADC122C04_STARTSYNC	    0x08    /* 0000 100X */
#define ADC122C04_RDATA	        0x16    /* 0001 XXXX */
#define ADC122C04_RREG_CGF0_REG	0x32    /* 0010 00XX */
#define ADC122C04_RREG_CGF1_REG	0x36    /* 0010 01XX */
#define ADC122C04_RREG_CGF2_REG	0x40    /* 0010 10XX */
#define ADC122C04_RREG_CGF3_REG	0x44    /* 0010 11XX */
#define ADC122C04_WREG_CGF0_REG	0x64    /* 0100 00XX */
#define ADC122C04_WREG_CGF1_REG	0x68    /* 0100 01XX */
#define ADC122C04_WREG_CGF2_REG	0x72    /* 0100 10XX */
#define ADC122C04_WREG_CGF3_REG	0x76    /* 0100 11XX */



#define ADC122C04_CFG0_GAIN_SHIFT   1
#define ADC122C04_CFG0_MUX_SHIFT    4

#define ADC122C04_CFG0_PGA_MASK     BIT(0)
#define ADC122C04_CFG0_GAIN_MASK    GENMASK(3, 1)
#define ADC122C04_CFG0_MUX_MASK     GENMASK(7, 4)


#define ADC122C04_CFG1_VREF_SHIFT   1
#define ADC122C04_CFG1_CM_SHIFT     3
#define ADC122C04_CFG1_MODE_SHIFT   4
#define ADC122C04_CFG1_DR_SHIFT     5

#define ADC122C04_CFG1_TS_MASK      BIT(0)
#define ADC122C04_CFG1_VREF_MASK    GENMASK(2, 1)
#define ADC122C04_CFG1_CM_MASK      BIT(3)
#define ADC122C04_CFG1_MODE_MASK    BIT(4)
#define ADC122C04_CFG1_DR_MASK      GENMASK(7, 5)


#define ADC122C04_CFG2_BCS_SHIFT   3
#define ADC122C04_CFG2_CRC_SHIFT   4
#define ADC122C04_CFG2_DCNT_SHIFT  6
#define ADC122C04_CFG2_DRDY_SHIFT  7

#define ADC122C04_CFG2_IDAC_MASK   GENMASK(2, 0)
#define ADC122C04_CFG2_BCS_MASK    BIT(3)
#define ADC122C04_CFG2_CRC_MASK    GENMASK(5, 4)
#define ADC122C04_CFG2_DCNT_MASK   BIT(6)
#define ADC122C04_CFG2_DRDY_MASK   BIT(7)


#define ADC122C04_CFG3_I2MUX_SHIFT  2
#define ADC122C04_CFG3_I1MUX_SHIFT  5

#define ADC122C04_CFG3_I2MUX_MASK   GENMASK(4, 2)
#define ADC122C04_CFG3_I1MUX_MASK   GENMASK(7, 5)


enum ads122c04_channels {
	ADS122C04_AIN0_AIN1 = 0,
	ADS122C04_AIN0_AIN2,
	ADS122C04_AIN0_AIN3,
    ADS122C04_AIN1_AIN0,
    ADS122C04_AIN1_AIN2,
    ADS122C04_AIN1_AIN3,
    ADS122C04_AIN2_AIN3,
    ADS122C04_AIN3_AIN2,
    ADS122C04_AIN0_AVSS,
    ADS122C04_AIN1_AVSS,
    ADS122C04_AIN2_AVSS,
    ADS122C04_AIN3_AVSS,
    // Options not supported for this driver
    /*ADS122C04_VREF_MON, 
    ADS122C04_AVDD_AVSS, 
    ADS122C04_AINP_AINN_SHORTED,
    ADS122C04_RESERVED, */
};

enum ads122c04_vref {
    ADS122C04_VREF_INTERNAL = 0,
    ADS122C04_VREF_EXTERNAL,
    ADS122C04_VREF_ANALOG1,
    ADS122C04_VREF_ANALOG2,
};

enum ads122c04_integrity_check {
    ADS122C04_ICHECK_DISABLED = 0,
    ADS122C04_ICHECK_INVERTED,
    ADS122C04_ICHECK_CRC16,
    ADS122C04_ICHECK_RESERVED,
};

enum ads122c04_idac_rounting {
    ADS122C04_IDAC_DISABLED = 0,
    ADS122C04_IDAC_AIN0,
    ADS122C04_IDAC_AIN1,
    ADS122C04_IDAC_AIN2,
    ADS122C04_IDAC_AIN3,
    ADS122C04_IDAC_REFP,
    ADS122C04_IDAC_REFN,
    ADS122C04_IDAC_RESERVED,
};


static const unsigned int ads122c04_gain_conf[] = {
	1, 2, 4, 8, 16, 32, 64, 128
};

static const unsigned int ads122c04_data_rate[] = {
	20, 45, 90, 175, 330, 600, 1000
};

static const unsigned int ads122c04_data_rate_turbo[] = {
	40, 90, 180, 350, 660, 1200, 2000
};

static const unsigned int ads122c04_idac_current[] = {
	0, 10, 50, 100, 250, 500, 1000, 1500
};

#define ADS122C04_CHAN(_chan, _addr) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 24,					\
		.storagebits = 24,				\
		.endianness = IIO_CPU,				\
	},							\
	.datasheet_name = "AIN"#_chan,				\
}

#define ADS122C04_DIFF_CHAN(_chan, _chan2, _addr) {		\
	.type = IIO_VOLTAGE,					\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\   
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 24,					\
		.storagebits = 24,				\
		.endianness = IIO_CPU,				\
	},							\
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}


struct ads122c04_channel_data {
	bool enabled; /* is channel enabled */
    u8 gain; /* gain configuration */
    bool pga_enabled;
    u8 data_rate;
    bool turbo_mode; /* false - normal mode; true -  Turbo mode*/
    bool conv_mode /* false - single ; true - continues*/
    bool temperature_mode;
    u8 vref; 
};


struct ads122c04_st {
	struct i2c_client *i2c;
	/*
	 * Protects ADC ops, e.g: concurrent sysfs/buffered
	 * data reads, configuration updates
	 */
	struct mutex lock;

	struct ads122c04_channel_data channel_data[ADS122C04_CHANNELS];
    
    bool counter_enabled;    
    u8 crc_check;
    bool burnout;
    u8 idac;

    u8 idac1_mux;
    u8 idac2_mux;
};

static const struct iio_chan_spec ads122c04_channels[] = {
	ADS122C04_DIFF_CHAN(0, 1, ADS122C04_AIN0_AIN1),
    ADS122C04_DIFF_CHAN(0, 2, ADS122C04_AIN0_AIN2),
    ADS122C04_DIFF_CHAN(0, 3, ADS122C04_AIN0_AIN3),
    ADS122C04_DIFF_CHAN(1, 0, ADS122C04_AIN1_AIN0),
    ADS122C04_DIFF_CHAN(1, 2, ADS122C04_AIN1_AIN2),
    ADS122C04_DIFF_CHAN(1, 3, ADS122C04_AIN1_AIN3),
    ADS122C04_DIFF_CHAN(2, 3, ADS122C04_AIN2_AIN3),
    ADS122C04_DIFF_CHAN(2, 3, ADS122C04_AIN3_AIN2),
    ADS122C04_DIFF_CHAN(2, 3, ADS122C04_AIN3_AIN2),
	ADS122C04_CHAN(0, ADS122C04_AIN0_AVSS),
	ADS122C04_CHAN(1, ADS122C04_AIN1_AVSS),
	ADS122C04_CHAN(2, ADS122C04_AIN2_AVSS),
	ADS122C04_CHAN(3, ADS122C04_AIN3_AVSS),
};

static int ads122c04_write_value(const struct ads122c04_state *st, const u8 cmd, const u8 *value) 
{
    int ret;

    if (value == NULL) {
        ret = i2c_smbus_write_byte_data(st->client, cmd, *value);
    } else {
        ret = i2c_smbus_write_byte(st->client, cmd);
    }
    
    if (ret < 0) {
        dev_err(&st->client->dev, "Failed to write configuration: %d cmd %d\n", ret, cmd);
        return ret;
    }

    return 0;
}

static int ads122c04_read_reg_value(const struct ads122c04_state *st, const u8 reg, u8 *value)
{
    int ret;
    if (reg < ADC122C04_RREG_CGF0_REG || reg > ADC122C04_RREG_CGF3_REG) 
        return -EINVAL;

    return  i2c_smbus_read_byte_data(st->client, reg, value);
}

static int ads122c04_read_adc(const struct ads122c04_state *st, int *value)
{
    int ret;

    u8 result_buf[3]; 

    msleep(1); 

    // Read the ADC result from the data registers of ADS122C04
    ret = i2c_smbus_read_i2c_block_data(st->client, ADC122C04_RDATA, 3, result_buf);
    if (ret < 0) {
        dev_err(&st->client->dev, "Failed to read ADC result: %d\n", ret);
        return ret;
    }

    // The result is in 3 bytes (24 bits), 
    // Combine the 3 bytes to form the 24-bit ADC result
    *val = (result_buf[0] << 16) | (result_buf[1] << 8) | result_buf[2];

    return 0;
}


static int ads122c04_write_cfg0(const struct ads122c04_st *st, const int chan)
{
    int ret; 
    u8 cfg = 0;

    cfg = chan << ADC122C04_CFG0_MUX_SHIFT |
        st->channel_data[chan].gain << ADC122C04_CFG0_GAIN_SHIFT | 
        st->channel_data[chan].pga_enabled;

    ads122c04_write_value(st, ADC122C04_WREG_CGF0_REG, &cfg);
    return 0;
}

static int ads122c04_write_cfg1(const struct ads122c04_st *st, const int chan)
{
    int ret; 
    u8 cfg = 0;

    cfg = st->channel_data[chan].data_rate << ADC122C04_CFG1_DR_SHIFT | 
        st->channel_data[chan].turbo_mode << ADC122C04_CFG1_MODE_SHIFT | 
        st->channel_data[chan].conv_mode << ADC122C04_CFG1_CM_SHIFT |
        st->channel_data[chan].vref << ADC122C04_CFG1_VREF_SHIFT |
        st->channel_data[chan].temperature_mode;

    ads122c04_write_value(st, ADC122C04_WREG_CGF1_REG, &cfg);
    return 0;
}

static int ads122c04_write_cfg2(const struct ads122c04_st *st)
{
    int ret; 
    u8 cfg = 0;
    
    cfg = st->counter_enabled << ADC122C04_CFG2_DCNT_SHIFT | 
        st->crc_check << ADC122C04_CFG2_CRC_SHIFT | 
        st->burnout << ADC122C04_CFG2_BCS_SHIFT |
        st->idac;

    ads122c04_write_value(st, ADC122C04_WREG_CGF2_REG, &cfg);
    return 0;
}


static int ads122c04_write_cfg3(const struct ads122c04_st *st)
{ 
    int ret; 
    u8 cfg = 0;

    cfg = st->idac1_mux << ADC122C04_CFG3_I1MUX_SHIFT | 
        st->idac2_mux << ADC122C04_CFG3_I2MUX_SHIFT;

    ads122c04_write_value(st, ADC122C04_WREG_CGF3_REG, &cfg);
    return 0;
}


static int ads122c04_get_adc_result(const struct ads122c04_st *st, const int chan, int *val)
{
    int ret 

	if (chan < 0 || chan >= ADC122C04_CHANNELS)
		return -EINVAL;

    ret = ads122c04_write_cfg0(st, data, chan);
    if (ret)
        return ret;

    ret = ads122c04_write_cfg1(st, data, chan);
    if (ret)
        return ret;

    /* Start/Sync depends of the convertion mode and the
       Datasheet recomends send de STARTSYNC cmd when the cfg1 is modified */
    ret = ads122c04_write_value(st, ADC122C04_STARTSYNC, NULL);
	if (ret) {
		return ret;
    }

	return ads122c04_read_adc(st, ADC122C04_RDATA, val);
}



static int ads122c04_set_data_rate(struct ads122c04_st *st, int chan, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads122c04_data_rate); i++) {
        if (st->channel_data[chan].turbo_mode) {
		    if (ads122c04_data_rate[i] == rate) {
			    st->channel_data[chan].data_rate = rate;
			    return 0;
		    }
        } else {
            if (ads122c04_data_rate_turbo[i] == rate) {
			    st->channel_data[chan].data_rate = rate;
			    return 0;
		    }
        }
	}

	return -EINVAL;
}


static int ads122c04_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	int ret, idx;
	struct ads122c04_st *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			break;

		ret = ads122c04_get_adc_result(st, chan->address, val);
		if (ret < 0) {
			goto release_direct;
		}

		*val = sign_extend32(*val >> chan->scan_type.shift,
				     chan->scan_type.realbits - 1);

		ret = IIO_VAL_INT;

release_direct:
		iio_device_release_direct_mode(indio_dev);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->channel_data[chan->address].data_rate;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int ads1015_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads122c04_st *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ads122c04_set_data_rate(st, chan->address, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&data->lock);

	return ret;
}


static const struct iio_info ads122c04_info = {
	.read_raw	= ads1015_read_raw,
	.write_raw	= ads1015_write_raw,
};


static int ads122c04_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122c04_st *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static int ads122c04_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ads122c04_st *data;
	enum chip_ids chip;
	int i;
    int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);

    data->i2c = client;
	mutex_init(&data->lock);

	indio_dev->name = ADS122C04_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	chip = (enum chip_ids)device_get_match_data(&client->dev);
	if ( chip == ADS122C04) {
		indio_dev->channels = ads122c04_channels;
		indio_dev->num_channels = ARRAY_SIZE(ads122c04_channels);
		indio_dev->info = &ads1015_info;
		data->data_rate = (unsigned int *) &ads122c04_st_rate;
    } else {
		dev_err(&client->dev, "Unknown chip %d\n", chip);
		return -EINVAL;
    }

	/* we need to keep this ABI the same as used by hwmon ADS1015 driver */
	ads1015_get_channels_config(client);

	data->regmap = devm_regmap_init_i2c(client, &ads1015_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "Failed to allocate register map\n");
		return PTR_ERR(data->regmap);
	}

	ret = ads1015_set_conv_mode(data, ADS1015_CONTINUOUS);
	if (ret)
		return ret;

	data->conv_invalid = true;

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register IIO device\n");
		return ret;
	}

	return 0;
}

static const struct dev_pm_ops ads122c04_pm_ops = {
	SET_RUNTIME_PM_OPS(ads122c04_runtime_suspend,
			   ads122c04_runtime_resume, NULL)
};

static const struct i2c_device_id ads122c04_id[] = {
	{"ads122c04", ADS122C04},
	{}
};
MODULE_DEVICE_TABLE(i2c, ads1015_id);


static const struct of_device_id ads122c04_of_match[] = {
	{
		.compatible = "ti,ads122c04",
		.data = (void *)ADS122C04
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

