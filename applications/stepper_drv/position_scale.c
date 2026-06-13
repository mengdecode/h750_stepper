/**
 * @file position_scale.c
 * @brief 物理位置 ↔ 编码器计数 线性拟合实现
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          the first version
 */

#include "position_scale.h"
#include <string.h>

void position_scale_init(position_scale_t *s,
                         float counts_per_unit,
                         const char *unit,
                         float pos_min, float pos_max)
{
    s->counts_per_unit = (counts_per_unit > 0) ? counts_per_unit : 1.0f;
    s->home_offset     = 0;
    s->pos_min         = pos_min;
    s->pos_max         = pos_max;

    rt_strncpy(s->unit, unit, sizeof(s->unit) - 1);
    s->unit[sizeof(s->unit) - 1] = '\0';

    /* 初始 counts 限位基于 home_offset=0 */
    s->pos_min_counts = position_scale_to_counts(s, pos_min);
    s->pos_max_counts = position_scale_to_counts(s, pos_max);
}

void position_scale_set_home(position_scale_t *s, rt_int32_t encoder_zero)
{
    /* 物理零点 = 当前位置 → 反推 home_offset
     *   0 = (encoder_zero - home_offset) / counts_per_unit
     *   → home_offset = encoder_zero
     */
    s->home_offset = encoder_zero;

    /* 重新计算 counts 限位 */
    s->pos_min_counts = position_scale_to_counts(s, s->pos_min);
    s->pos_max_counts = position_scale_to_counts(s, s->pos_max);
}

rt_int32_t position_scale_to_counts(position_scale_t *s, float physical)
{
    return (rt_int32_t)((float)s->home_offset
                        + physical * s->counts_per_unit);
}

float position_scale_to_physical(position_scale_t *s, rt_int32_t counts)
{
    return (float)(counts - s->home_offset) / s->counts_per_unit;
}

int position_scale_check_range(position_scale_t *s, float target)
{
    return (target >= s->pos_min && target <= s->pos_max) ? 1 : 0;
}
