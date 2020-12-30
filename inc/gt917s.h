/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-01     tyustli     the first version
 */

#ifndef __GT917S_H_
#define __GT917S_H_
#include <rtthread.h>
#include "touch.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define GTP_ADDR_LENGTH        (2)
#define GT917S_MAX_TOUCH       (5)
#define GT917S_POINT_INFO_NUM  (5)

#define GT917S_ADDRESS_HIGH    (0x5D)
#define GT917S_ADDRESS_LOW     (0x14)

#define GT917S_COMMAND         (0x8040)
#define GT917S_CONFIG          (0x8050)

#define GT9XX_PRODUCT_ID       (0x8140)
#define GT917S_READ_STATUS     (0x814E)

#define GT917S_POINT1_REG      (0x814F)
#define GT917S_POINT2_REG      (0X8157)
#define GT917S_POINT3_REG      (0X815F)
#define GT917S_POINT4_REG      (0X8167)
#define GT917S_POINT5_REG      (0X816F)

#define GT917S_CHECK_SUM       (0X80FF)

int rt_hw_gt917s_init(const char *name, struct rt_touch_config *cfg);


#ifdef __cplusplus
}
#endif

#endif	/* __GT917S_H_ */
