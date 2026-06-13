/**
 * @file position_scale.h
 * @brief 物理位置 ↔ 编码器计数 线性拟合
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * 公式:
 *   counts = home_offset + physical_value * counts_per_unit
 *   physical = (counts - home_offset) / counts_per_unit
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          the first version
 */

#ifndef POSITION_SCALE_H__
#define POSITION_SCALE_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float      counts_per_unit;   /* 1 个物理单位 = 多少个编码器 count */
    rt_int32_t home_offset;       /* 物理零点对应的编码器值 */
    char       unit[8];           /* 单位名: "mm", "°", "工位" */
    float      pos_min;           /* 软限位 (物理单位) */
    float      pos_max;           /* 软限位 (物理单位) */
    rt_int32_t pos_min_counts;    /* 软限位 (counts, 内部缓存) */
    rt_int32_t pos_max_counts;    /* 软限位 (counts, 内部缓存) */
} position_scale_t;

/**
 * 初始化比例尺
 * @param counts_per_unit  每个物理单位的编码器计数值
 * @param unit             单位名 ("mm", "°" 等)
 * @param pos_min / pos_max 物理软限位
 */
void position_scale_init(position_scale_t *s,
                         float counts_per_unit,
                         const char *unit,
                         float pos_min, float pos_max);

/**
 * 找零后设置物理零点对应的编码器值
 * 自动根据 pos_min/pos_max 算出 counts 限位
 */
void position_scale_set_home(position_scale_t *s, rt_int32_t encoder_zero);

/** 物理单位 → 编码器计数 */
rt_int32_t position_scale_to_counts(position_scale_t *s, float physical);

/** 编码器计数 → 物理单位 */
float position_scale_to_physical(position_scale_t *s, rt_int32_t counts);

/** 检查目标是否在软限位范围内, 1=OK 0=超限 */
int position_scale_check_range(position_scale_t *s, float target);

#ifdef __cplusplus
}
#endif
#endif
