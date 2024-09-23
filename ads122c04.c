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

#define ADS122C04_CHANNELS 12

#define ADS122C04_POWERDOWN	    0x02    /* 0000 001X */
#define ADS122C04_RESET         0x06    /* 0000 011X */
#define ADS122C04_STARTSYNC	    0x08    /* 0000 100X */
#define ADS122C04_RDATA	        0x16    /* 0001 XXXX */
#define ADS122C04_RREG_CGF0_REG	0x32    /* 0010 00XX */
#define ADS122C04_RREG_CGF1_REG	0x36    /* 0010 01XX */
#define ADS122C04_RREG_CGF2_REG	0x40    /* 0010 10XX */
#define ADS122C04_RREG_CGF3_REG	0x44    /* 0010 11XX */
#define ADS122C04_WREG_CGF0_REG	0x64    /* 0100 00XX */
#define ADS122C04_WREG_CGF1_REG	0x68    /* 0100 01XX */
#define ADS122C04_WREG_CGF2_REG	0x72    /* 0100 10XX */
#define ADS122C04_WREG_CGF3_REG	0x76    /* 0100 11XX */



#define ADS122C04_CFG0_GAIN_SHIFT   1
#define ADS122C04_CFG0_MUX_SHIFT    4

#define ADS122C04_CFG0_PGA_MASK     BIT(0)
#define ADS122C04_CFG0_GAIN_MASK    GENMASK(3, 1)
#define ADS122C04_CFG0_MUX_MASK     GENMASK(7, 4)


#define ADS122C04_CFG1_VREF_SHIFT   1
#define ADS122C04_CFG1_CM_SHIFT     3
#define ADS122C04_CFG1_MODE_SHIFT   4
#define ADS122C04_CFG1_DR_SHIFT     5

#define ADS122C04_CFG1_TS_MASK      BIT(0)
#define ADS122C04_CFG1_VREF_MASK    GENMASK(2, 1)
#define ADS122C04_CFG1_CM_MASK      BIT(3)
#define ADS122C04_CFG1_MODE_MASK    BIT(4)
#define ADS122C04_CFG1_DR_MASK      GENMASK(7, 5)


#define ADS122C04_CFG2_BCS_SHIFT   3
#define ADS122C04_CFG2_CRC_SHIFT   4
#define ADS122C04_CFG2_DCNT_SHIFT  6
#define ADS122C04_CFG2_DRDY_SHIFT  7

#define ADS122C04_CFG2_IDAC_MASK   GENMASK(2, 0)
#define ADS122C04_CFG2_BCS_MASK    BIT(3)
#define ADS122C04_CFG2_CRC_MASK    GENMASK(5, 4)
#define ADS122C04_CFG2_DCNT_MASK   BIT(6)
#define ADS122C04_CFG2_DRDY_MASK   BIT(7)


#define ADS122C04_CFG3_I2MUX_SHIFT  2
#define ADS122C04_CFG3_I1MUX_SHIFT  5

#define ADS122C04_CFG3_I2MUX_MASK   GENMASK(4, 2)
#define ADS122C04_CFG3_I1MUX_MASK   GENMASK(7, 5)


#define CHANNEL_DISABLED                0
#define CHANNEL_ENABLED                 1

#define ADS122C04_PGA_ON                0    
#define ADS122C04_PGA_OFF               1 

#define ADS122C04_TURBO_MODE_OFF        0    
#define ADS122C04_TURBO_MODE_ON         1

#define ADS122C04_TEMPERATURE_MODE_OFF  0    
#define ADS122C04_TEMPERATURE_MODE_ON   1    

#define ADS122C04_CONV_MODE_SINGLE      0
#define ADS122C04_CONV_MODE_CONTINUES   1


#define ADS122C04_DEFAULT_PGA                   ADS122C04_PGA_ON
#define ADS122C04_DEFAULT_GAIN                  0 /*1*/        
#define ADS122C04_DEFAULT_DATA_RATE             4 /*330*/
#define ADS122C04_DEFAULT_TURBO_MODE            ADS122C04_TURBO_MODE_OFF
#define ADS122C04_DEFAULT_CONV_MODE             ADS122C04_CONV_MODE_SINGLE
#define ADS122C04_DEFAULT_TEMPERATURE_MODE      ADS122C04_TEMPERATURE_MODE_OFF
#define ADS122C04_DEFAULT_MAIN_VREF_REFERENCE   ADS122C04_VREF_INTERNAL

enum chip_ids {
	ADSXXXXXX = 0,
	ADS122C04,
};

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


static const unsigned int ads122c04_gain_cfg[] = {
	1, 2, 4, 8, 16, 32 , 64, 128
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
	.type = IIO_VOLTAGE,				\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
                BIT(IIO_CHAN_INFO_SCALE) |          \
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	    \
	.scan_index = _addr,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 24,					\
		.storagebits = 24,				\
		.endianness = IIO_CPU,			\
	},							        \
	.datasheet_name = "AIN"#_chan,		\
}

#define ADS122C04_DIFF_CHAN(_chan, _chan2, _addr) {	\
	.type = IIO_VOLTAGE,				\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
    	        BIT(IIO_CHAN_INFO_SCALE) |          \
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	    \
	.scan_index = _addr,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 24,					\
		.storagebits = 24,				\
		.endianness = IIO_CPU,			\
	},							        \
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}


struct ads122c04_channel_data {
	bool enabled; /* is channel enabled */
    u8 gain; /* gain configuration */
    bool pga_enabled;
    u8 data_rate;
    bool turbo_mode; /* false - normal mode; true -  Turbo mode*/
    bool conv_mode; /* false - single ; true - continues*/
    bool temperature_mode;
    u8 vref; 
};


struct ads122c04_st {
	struct i2c_client *client;
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

static int ads122c04_write_value(const struct ads122c04_st *st, const u8 cmd, const u8 *value) 
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

static int ads122c04_read_reg_value(const struct ads122c04_st *st, const u8 reg, u8 *value)
{
    if (reg < ADS122C04_RREG_CGF0_REG || reg > ADS122C04_RREG_CGF3_REG) 
        return -EINVAL;

    return  i2c_smbus_read_byte_data(st->client, reg);
}

static int ads122c04_read_adc(const struct ads122c04_st *st, int *value)
{
    int ret;

    u8 result_buf[3]; 

    msleep(1); 

    // Read the ADC result from the data registers of ADS122C04
    ret = i2c_smbus_read_i2c_block_data(st->client, ADS122C04_RDATA, 3, result_buf);
    if (ret < 0) {
        dev_err(&st->client->dev, "Failed to read ADC result: %d\n", ret);
        return ret;
    }

    // The result is in 3 bytes (24 bits), 
    // Combine the 3 bytes to form the 24-bit ADC result
    *value = (result_buf[0] << 16) | (result_buf[1] << 8) | result_buf[2];

    return 0;
}


static int ads122c04_write_cfg0(const struct ads122c04_st *st, const int chan)
{
    u8 cfg = 0;

    cfg = chan << ADS122C04_CFG0_MUX_SHIFT |
        st->channel_data[chan].gain << ADS122C04_CFG0_GAIN_SHIFT | 
        st->channel_data[chan].pga_enabled;

    ads122c04_write_value(st, ADS122C04_WREG_CGF0_REG, &cfg);
    return 0;
}

static int ads122c04_write_cfg1(const struct ads122c04_st *st, const int chan)
{
    u8 cfg = 0;

    cfg = st->channel_data[chan].data_rate << ADS122C04_CFG1_DR_SHIFT | 
        st->channel_data[chan].turbo_mode << ADS122C04_CFG1_MODE_SHIFT | 
        st->channel_data[chan].conv_mode << ADS122C04_CFG1_CM_SHIFT |
        st->channel_data[chan].vref << ADS122C04_CFG1_VREF_SHIFT |
        st->channel_data[chan].temperature_mode;

    ads122c04_write_value(st, ADS122C04_WREG_CGF1_REG, &cfg);
    return 0;
}

static int ads122c04_write_cfg2(const struct ads122c04_st *st)
{
    u8 cfg = 0;
    
    cfg = st->counter_enabled << ADS122C04_CFG2_DCNT_SHIFT | 
        st->crc_check << ADS122C04_CFG2_CRC_SHIFT | 
        st->burnout << ADS122C04_CFG2_BCS_SHIFT |
        st->idac;

    ads122c04_write_value(st, ADS122C04_WREG_CGF2_REG, &cfg);
    return 0;
}


static int ads122c04_write_cfg3(const struct ads122c04_st *st)
{ 
    u8 cfg = 0;

    cfg = st->idac1_mux << ADS122C04_CFG3_I1MUX_SHIFT | 
        st->idac2_mux << ADS122C04_CFG3_I2MUX_SHIFT;

    ads122c04_write_value(st, ADS122C04_WREG_CGF3_REG, &cfg);
    return 0;
}


static int ads122c04_get_adc_result(const struct ads122c04_st *st, const int chan, int *val)
{
    int ret;

	if (chan < 0 || chan >= ADS122C04_CHANNELS)
		return -EINVAL;

    ret = ads122c04_write_cfg0(st, chan);
    if (ret)
        return ret;

    ret = ads122c04_write_cfg1(st, chan);
    if (ret)
        return ret;

    /* Start/Sync depends of the convertion mode and the
       Datasheet recomends send de STARTSYNC cmd when the cfg1 is modified */
    ret = ads122c04_write_value(st, ADS122C04_STARTSYNC, NULL);
	if (ret) {
		return ret;
    }

	return ads122c04_read_adc(st, val);
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


static int ads122c04_set_gain(struct ads122c04_st *st, int chan, int gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads122c04_gain_cfg); i++) {
		if (ads122c04_gain_cfg[i] == gain) {
			st->channel_data[chan].data_rate = gain;
			return 0;
		}
	}

	return -EINVAL;
}



static int ads122c04_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	int ret;
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
	case IIO_CHAN_INFO_SCALE:
		*val = st->channel_data[chan->address].gain;
		ret = IIO_VAL_INT;
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

static int ads122c04_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads122c04_st *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	switch (mask) {
    case IIO_CHAN_INFO_SCALE:
		ret = ads122c04_set_gain(st, chan->address, val);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ads122c04_set_data_rate(st, chan->address, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}


static IIO_CONST_ATTR_NAMED(ads122c04_scale_available,
	scale_available, "1 2 4 8 16 32 64 128");

static IIO_CONST_ATTR_NAMED(ads122c04_sampling_frequency_available,
	sampling_frequency_available, "20 40 45 90 180 175 350 330 600 660 1000 1200 2000");

static struct attribute *ads122c04_attributes[] = {
    &iio_const_attr_ads122c04_scale_available.dev_attr.attr,
	&iio_const_attr_ads122c04_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads122c04_attribute_group = {
	.attrs = ads122c04_attributes,
};


static const struct iio_info ads122c04_info = {
	.read_raw	= ads122c04_read_raw,
	.write_raw	= ads122c04_write_raw,
    .attrs      = &ads122c04_attribute_group,
};


static int ads122c04_client_get_channels_config(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122c04_st *data = iio_priv(indio_dev);
	struct device *dev = &client->dev;
	struct fwnode_handle *node;
	int i = -1;
    int channel;

	device_for_each_child_node(dev, node) {
		u32 pval;

		if (fwnode_property_read_u32(node, "reg", &pval)) {
			dev_err(dev, "invalid reg on %pfw\n", node);
			continue;
		}

		channel = pval;
		if (channel >= ADS122C04_CHANNELS) {
			dev_err(dev, "invalid channel index %d on %pfw\n",
				channel, node);
			continue;
		}

        data->channel_data[channel].enabled = CHANNEL_ENABLED;

        if (!fwnode_property_present(node, "ti,pga-disable"))
            data->channel_data[channel].pga = ADS122C04_PGA_OFF;

		if (!fwnode_property_read_u32(node, "ti,gain", &pval)) {
            if (data->channel_data[channel].pga == ADS122C04_PGA_ON) {
			    if (pval > 7) {
				    dev_err(dev, "invalid gain on %pfw\n", node);
				    fwnode_handle_put(node);
				    return -EINVAL;
			    }
            } else {
                	if (pval > 3) {
				    dev_err(dev, "invalid gain on %pfw. pga is off\n", node);
				    fwnode_handle_put(node);
				    return -EINVAL;
			    }
            }
            data->channel_data[channel].gain = pval;
		}

		if (!fwnode_property_read_u32(node, "ti,datarate", &pval)) {
			
			if (pval > 7) {
				dev_err(dev, "invalid data_rate on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
            data->channel_data[channel].datarate = pval;
		}

        if (!fwnode_property_read_u32(node, "ti,vref", &pval)) {
			if (pval > 4) {
				dev_err(dev, "invalid vref on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
            data->channel_data[channel].vref = pval;
		}

        if (!fwnode_property_present(node, "ti,turbo-mode-enabled"))
            data->channel_data[channel].turbo_mode = ADS122C04_TURBO_MODE_ON;

        if (!fwnode_property_present(node, "ti,temperature-mode-enabled"))
            data->channel_data[channel].turbo_mode = ADS122C04_TEMPERATURE_MODE_ON;

        if (!fwnode_property_present(node, "ti,continues-mode"))
            data->channel_data[channel].turbo_mode = ADS122C04_CONV_MODE_CONTINUES;

		i++;
	}

	return i < 0 ? -EINVAL : 0;
}


static void ads122c04_get_channels_config(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122c04_st *data = iio_priv(indio_dev);
    struct ads122c04_channel_data chan_default = {
            .enabled = CHANNEL_DISABLED,
            .pga_enabled = ADS122C04_DEFAULT_PGA,
            .gain = ADS122C04_DEFAULT_GAIN,
            .data_rate  = ADS122C04_DEFAULT_DATA_RATE,
            .turbo_mode = ADS122C04_DEFAULT_TURBO_MODE,
            .conv_mode = ADS122C04_DEFAULT_CONV_MODE,
            .temperature_mode = ADS122C04_DEFAULT_TEMPERATURE_MODE,
            .vref = ADS122C04_DEFAULT_MAIN_VREF_REFERENCE,
    };
    int i = 0;

    
	/* Default configuration */
	for(i = 0; i < ADS122C04_CHANNELS; ++i) {
		data->channel_data[i] = chan_default;
	}

	if (!ads122c04_client_get_channels_config(client))
		return;

}

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

    data->client = client;
	mutex_init(&data->lock);

	indio_dev->name = ADS122C04_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	chip = (enum chip_ids)device_get_match_data(&client->dev);
	if ( chip == ADS122C04) {
		indio_dev->channels = ads122c04_channels;
		indio_dev->num_channels = ARRAY_SIZE(ads122c04_channels);
		indio_dev->info = &ads122c04_info;
    } else {
		dev_err(&client->dev, "Unknown chip %d\n", chip);
		return -EINVAL;
    }

	ads122c04_get_channels_config(client);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register IIO device\n");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id ads122c04_id[] = {
	{"ads122c04", ADS122C04},
	{}
};

MODULE_DEVICE_TABLE(i2c, ads122c04_id);


static const struct of_device_id ads122c04_of_match[] = {
	{
		.compatible = "ti,ads122c04",
	},
	{}
};
MODULE_DEVICE_TABLE(of, ads122c04_of_match);

static struct i2c_driver ads122c04_driver = {
	.driver = {
		.name = ADS122C04_DRV_NAME,
		.of_match_table = ads122c04_of_match,
	},
	.probe		= ads122c04_probe,
	.remove		= ads122c04_remove,
	.id_table	= ads122c04_id,
};

module_i2c_driver(ads122c04_driver);

MODULE_AUTHOR("Aluisio Leonello Victal <alvictal@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS122C04 ADC driver");
MODULE_LICENSE("MIT");

