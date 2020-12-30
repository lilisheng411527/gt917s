/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-01     tyustli     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>
#include <stdlib.h>

#define DBG_TAG "gt917s"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "touch.h"
#include "gt917s.h"

static struct rt_i2c_client *gt917s_client;

static const rt_uint8_t GT917S_CFG_TBL[] ={
  0x97,0x20,0x03,0xE0,0x01,0x0A,0x35,0x04,0x00,0x69,
  0x09,0x0F,0x50,0x32,0x33,0x11,0x00,0x32,0x11,0x11,
  0x28,0x8C,0xAA,0xDC,0x58,0x04,0x00,0x00,0x1E,0x3C,
  0x00,0x00,0x00,0x31,0x00,0x00,0x00,0x00,0x00,0x40,
  0x32,0x00,0x00,0x50,0x38,0x00,0x8D,0x20,0x16,0x4E,
  0x4C,0x7C,0x05,0x28,0x3E,0x28,0x0D,0x43,0x24,0x00,
  0x01,0x39,0x6B,0xC0,0x94,0x84,0x2D,0x00,0x54,0xB0,
  0x41,0x9D,0x49,0x8D,0x52,0x7F,0x5A,0x75,0x62,0x6C,
  0x42,0x50,0x14,0x00,0x00,0x00,0x00,0xF0,0x50,0x3C,
  0x88,0x88,0x27,0x50,0x3C,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x02,0x78,
  0x0A,0x50,0xFF,0xE4,0x04,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x3C,0xB0,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x56,0xA2,0x07,0x50,0x1E,
  0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,
  0x0F,0x10,0x12,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,
  0x1D,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0x1F,0x1E,0x1D,0x1C,0x1B,0x1A,0x19,0x18,
  0x17,0x15,0x14,0x13,0x12,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,0x00,0x30,0x7F,0x7F,0x7F,0xFF,
  0x54,0x64,0x00,0x80,0x46,0x07,0x50,0x3C,0x32,0x14,
  0x0A,0x64,0x32,0x00,0x00,0x00,0x00,0x11,0x02,0x62,
  0x32,0x03,0x14,0x50,0x0C,0xE2,0x14,0x50,0x00,0x54,
  0x10,0x00,0x32,0xA2,0x07,0x64,0xA4,0xB6,0x01
};

static rt_err_t gt917s_write_reg(struct rt_i2c_client *dev, rt_uint8_t write_len, rt_uint8_t *write_data)
{
    struct rt_i2c_msg msgs;

    msgs.addr  = dev->client_addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf   = write_data;
    msgs.len   = write_len;

    if (rt_i2c_transfer(dev->bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static rt_err_t gt917s_read_regs(struct rt_i2c_client *dev, rt_uint8_t *cmd_buf, rt_uint8_t cmd_len, rt_uint8_t read_len, rt_uint8_t *read_buf)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = dev->client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = cmd_buf;
    msgs[0].len   = cmd_len;

    msgs[1].addr  = dev->client_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = read_buf;
    msgs[1].len   = read_len;

    if (rt_i2c_transfer(dev->bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

/**
 * This function read the product id
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for gt9xx
 * @param read data len
 * @param read data pointer
 *
 * @return the read status, RT_EOK reprensents  read the value of the register successfully.
 */
static rt_err_t gt917s_get_product_id(struct rt_i2c_client *dev, rt_uint8_t read_len, rt_uint8_t *read_data)
{
    rt_uint8_t cmd_buf[2];

    cmd_buf[0] = (rt_uint8_t)(GT9XX_PRODUCT_ID >> 8);
    cmd_buf[1] = (rt_uint8_t)(GT9XX_PRODUCT_ID & 0xff);

    if (gt917s_read_regs(dev, cmd_buf, 2, read_len, read_data) != RT_EOK)
    {
        LOG_D("read id failed \n");

        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t gt917s_get_info(struct rt_i2c_client *dev, struct rt_touch_info *info)
{
    rt_uint8_t opr_buf[7] = {0};
    rt_uint8_t cmd_buf[2];

    cmd_buf[0] = (rt_uint8_t)(GT917S_CONFIG >> 8);
    cmd_buf[1] = (rt_uint8_t)(GT917S_CONFIG & 0xff);

    if (gt917s_read_regs(dev, cmd_buf, 2, 7, opr_buf) != RT_EOK)
    {
        LOG_D("read id failed \n");

        return -RT_ERROR;
    }

    info->range_x = (opr_buf[2] << 8) + opr_buf[1];
    info->range_y = (opr_buf[4] << 8) + opr_buf[3];
    info->point_num = opr_buf[5] & 0x0f;

    return RT_EOK;

}

static rt_err_t gt917s_soft_reset(struct rt_i2c_client *dev)
{
    rt_uint8_t buf[3];

    buf[0] = (rt_uint8_t)(GT917S_COMMAND >> 8);
    buf[1] = (rt_uint8_t)(GT917S_COMMAND & 0xFF);
    buf[2] = 0x02;

    if (gt917s_write_reg(dev, 3, buf) != RT_EOK)
    {
        LOG_D("soft reset gt917s failed\n");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t gt917s_control(struct rt_touch_device *device, int cmd, void *data)
{
    if (cmd == RT_TOUCH_CTRL_GET_ID)
    {
        return gt917s_get_product_id(gt917s_client, 6, data);
    }

    if (cmd == RT_TOUCH_CTRL_GET_INFO)
    {
        return gt917s_get_info(gt917s_client, data);
    }

    rt_uint8_t buf[4];
    rt_uint8_t i = 0;
    rt_uint8_t *config;

    config = (rt_uint8_t *)rt_calloc(1, sizeof(GT917S_CFG_TBL) + GTP_ADDR_LENGTH);

    if (config == RT_NULL)
    {
        LOG_D("malloc config memory failed\n");
        return -RT_ERROR;
    }

    config[0] = (rt_uint8_t)((GT917S_CONFIG >> 8) & 0xFF); /* config reg */
    config[1] = (rt_uint8_t)(GT917S_CONFIG & 0xFF);

    memcpy(&config[2], GT917S_CFG_TBL, sizeof(GT917S_CFG_TBL)); /* config table */

    switch(cmd)
    {
    case RT_TOUCH_CTRL_SET_X_RANGE: /* set x range */
    {
        rt_uint16_t x_ran;

        x_ran = *(rt_uint16_t *)data;
        config[4] = (rt_uint8_t)(x_ran >> 8);
        config[3] = (rt_uint8_t)(x_ran & 0xff);

        break;
    }
    case RT_TOUCH_CTRL_SET_Y_RANGE: /* set y range */
    {
        rt_uint16_t y_ran;

        y_ran = *(rt_uint16_t *)data;
        config[6] = (rt_uint8_t)(y_ran >> 8);
        config[5] = (rt_uint8_t)(y_ran & 0xff);

        break;
    }
    case RT_TOUCH_CTRL_SET_X_TO_Y: /* change x y */
    {
        config[8] = config[8] ^= (1 << 3);
        break;
    }
    case RT_TOUCH_CTRL_SET_MODE: /* change int trig type */
    {
        rt_uint16_t trig_type;
        trig_type = *(rt_uint16_t *)data;

        switch (trig_type)
        {
        case RT_DEVICE_FLAG_INT_RX:
            config[8] &= 0xFC;
            break;
        case RT_DEVICE_FLAG_RDONLY:
            config[8] &= 0xFC;
            config[8] |= 0x02;
            break;
        default:
            break;
        }
        break;
    }
    default:
    {
        break;
    }
    }

    if (gt917s_write_reg(gt917s_client, sizeof(GT917S_CFG_TBL) + GTP_ADDR_LENGTH, config) != RT_EOK)  /* send config */
    {
        LOG_D("send config failed\n");
        return -RT_ERROR;
    }

    buf[0] = (rt_uint8_t)((GT917S_CHECK_SUM >> 8) & 0xFF);
    buf[1] = (rt_uint8_t)(GT917S_CHECK_SUM & 0xFF);
    buf[2] = 0;

    for(i = GTP_ADDR_LENGTH; i < sizeof(GT917S_CFG_TBL) + GTP_ADDR_LENGTH; i++)
        buf[GTP_ADDR_LENGTH] += config[i];

    buf[2] = (~buf[2]) + 1;
    buf[3] = 1;

    gt917s_write_reg(gt917s_client, 4, buf);
    rt_free(config);

    return RT_EOK;
}

static int16_t pre_x[GT917S_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static int16_t pre_y[GT917S_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static int16_t pre_w[GT917S_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static rt_uint8_t s_tp_dowm[GT917S_MAX_TOUCH];
static struct rt_touch_data *read_data;

static void gt917s_touch_up(void *buf, int8_t id)
{
    read_data = (struct rt_touch_data *)buf;

    if(s_tp_dowm[id] == 1)
    {
        s_tp_dowm[id] = 0;
        read_data[id].event = RT_TOUCH_EVENT_UP;
    }
    else
    {
        read_data[id].event = RT_TOUCH_EVENT_NONE;
    }

    read_data[id].timestamp = rt_touch_get_ts();
    read_data[id].width = pre_w[id];
    read_data[id].x_coordinate = pre_x[id];
    read_data[id].y_coordinate = pre_y[id];
    read_data[id].track_id = id;

    pre_x[id] = -1;  /* last point is none */
    pre_y[id] = -1;
    pre_w[id] = -1;
}

static void gt917s_touch_down(void *buf, int8_t id, int16_t x, int16_t y, int16_t w)
{
    read_data = (struct rt_touch_data *)buf;

    if (s_tp_dowm[id] == 1)
    {
        read_data[id].event = RT_TOUCH_EVENT_MOVE;

    }
    else
    {
        read_data[id].event = RT_TOUCH_EVENT_DOWN;
        s_tp_dowm[id] = 1;
    }

    read_data[id].timestamp = rt_touch_get_ts();
    read_data[id].width = w;
    read_data[id].x_coordinate = x;
    read_data[id].y_coordinate = y;
    read_data[id].track_id = id;

    pre_x[id] = x; /* save last point */
    pre_y[id] = y;
    pre_w[id] = w;
}

static rt_size_t gt917s_read_point(struct rt_touch_device *touch, void *buf, rt_size_t read_num)
{
    rt_uint8_t point_status = 0;
    rt_uint8_t touch_num = 0;
    rt_uint8_t write_buf[3];
    rt_uint8_t cmd[2];
    rt_uint8_t read_buf[8 * GT917S_MAX_TOUCH] = {0};
    rt_uint8_t read_index;
    int8_t read_id = 0;
    int16_t input_x = 0;
    int16_t input_y = 0;
    int16_t input_w = 0;

    static rt_uint8_t pre_touch = 0;
    static int8_t pre_id[GT917S_MAX_TOUCH] = {0};

    /* point status register */
    cmd[0] = (rt_uint8_t)((GT917S_READ_STATUS >> 8) & 0xFF);
    cmd[1] = (rt_uint8_t)(GT917S_READ_STATUS & 0xFF);

    if (gt917s_read_regs(gt917s_client, cmd, 2, 1, &point_status) != RT_EOK)
    {
        LOG_D("read point failed\n");
        read_num = 0;
        goto exit_;
    }

    if (point_status == 0)             /* no data */
    {
        read_num = 0;
        goto exit_;
    }

    if ((point_status & 0x80) == 0)    /* data is not ready */
    {
        read_num = 0;
        goto exit_;
    }

    touch_num = point_status & 0x0f;  /* get point num */

    if (touch_num > GT917S_MAX_TOUCH) /* point num is not correct */
    {
        read_num = 0;
        goto exit_;
    }

    cmd[0] = (rt_uint8_t)((GT917S_POINT1_REG >> 8) & 0xFF);
    cmd[1] = (rt_uint8_t)(GT917S_POINT1_REG & 0xFF);

    /* read point num is read_num */
    if (gt917s_read_regs(gt917s_client, cmd, 2, read_num * GT917S_POINT_INFO_NUM, read_buf) != RT_EOK)
    {
        LOG_D("read point failed\n");
        read_num = 0;
        goto exit_;
    }

    if (pre_touch > touch_num)                                       /* point up */
    {
        for (read_index = 0; read_index < pre_touch; read_index++)
        {
            rt_uint8_t j;

            for (j = 0; j < touch_num; j++)                          /* this time touch num */
            {
                read_id = read_buf[j * 8] & 0x0F;

                if (pre_id[read_index] == read_id)                   /* this id is not free */
                    break;

                if (j >= touch_num - 1)
                {
                    rt_uint8_t up_id;
                    up_id = pre_id[read_index];
                    gt917s_touch_up(buf, up_id);
                }
            }
        }
    }

    if (touch_num)                                                 /* point down */
    {
        rt_uint8_t off_set;

        for (read_index = 0; read_index < touch_num; read_index++)
        {
            off_set = read_index * 8;
            read_id = read_buf[off_set] & 0x0f;
            pre_id[read_index] = read_id;
            input_x = read_buf[off_set + 1] | (read_buf[off_set + 2] << 8);	/* x */
            input_y = read_buf[off_set + 3] | (read_buf[off_set + 4] << 8);	/* y */
            input_w = read_buf[off_set + 5] | (read_buf[off_set + 6] << 8);	/* size */

            gt917s_touch_down(buf, read_id, input_x, input_y, input_w);
        }
    }
    else if (pre_touch)
    {
        for(read_index = 0; read_index < pre_touch; read_index++)
        {
            gt917s_touch_up(buf, pre_id[read_index]);
        }
    }

    pre_touch = touch_num;

exit_:
    write_buf[0] = (rt_uint8_t)((GT917S_READ_STATUS >> 8) & 0xFF);
    write_buf[1] = (rt_uint8_t)(GT917S_READ_STATUS & 0xFF);
    write_buf[2] = 0x00;
    gt917s_write_reg(gt917s_client, 3, write_buf);
    return read_num;
}

static struct rt_touch_ops touch_ops =
{
    .touch_readpoint = gt917s_read_point,
    .touch_control = gt917s_control,
};

int rt_hw_gt917s_init(const char *name, struct rt_touch_config *cfg)
{
    rt_touch_t touch_device = RT_NULL;

    touch_device = (rt_touch_t)rt_calloc(1, sizeof(struct rt_touch_device));

    if (touch_device == RT_NULL)
        return -RT_ERROR;

    /* hardware init */
    rt_pin_mode(*(rt_uint8_t *)cfg->user_data, PIN_MODE_OUTPUT);
    rt_pin_mode(cfg->irq_pin.pin, PIN_MODE_OUTPUT);
    rt_pin_write(*(rt_uint8_t *)cfg->user_data, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(*(rt_uint8_t *)cfg->user_data, PIN_HIGH);
    rt_thread_mdelay(10);
    rt_pin_mode(cfg->irq_pin.pin, PIN_MODE_INPUT);
    rt_thread_mdelay(100);

    /* interface bus */
    gt917s_client = (struct rt_i2c_client *)rt_calloc(1, sizeof(struct rt_i2c_client));

    gt917s_client->bus = (struct rt_i2c_bus_device *)rt_device_find(cfg->dev_name);

    if (gt917s_client->bus == RT_NULL)
    {
        LOG_E("Can't find device\n");
        return -RT_ERROR;
    }

    if (rt_device_open((rt_device_t)gt917s_client->bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open device failed\n");
        return -RT_ERROR;
    }

    gt917s_client->client_addr = GT917S_ADDRESS_HIGH;
    gt917s_soft_reset(gt917s_client);

    /* register touch device */
    touch_device->info.type = RT_TOUCH_TYPE_CAPACITANCE;
    touch_device->info.vendor = RT_TOUCH_VENDOR_GT;
    rt_memcpy(&touch_device->config, cfg, sizeof(struct rt_touch_config));
    touch_device->ops = &touch_ops;

    rt_hw_touch_register(touch_device, name, RT_DEVICE_FLAG_INT_RX, RT_NULL);

    LOG_I("touch device gt917s init success\n");

    return RT_EOK;
}

/************************** end of file ********************************/


