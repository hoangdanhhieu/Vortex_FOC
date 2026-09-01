/**
 * @file foc_slow_task.h
 * @brief FOC 1kHz Slow Task, Speed Loop, Ramps & Safety Protections
 */

#ifndef FOC_SLOW_TASK_H
#define FOC_SLOW_TASK_H

/**
 * @brief 1 kHz Slow Control Task (Vbus filter, OV/UV/Stall/GF protection, speed ramp & LADRC)
 */
void FOC_SlowTask(void);

#endif /* FOC_SLOW_TASK_H */
