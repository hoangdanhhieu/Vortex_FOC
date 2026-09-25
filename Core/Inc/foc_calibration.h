/**
 * @file foc_calibration.h
 * @brief FOC ADC Offset Calibration, Noise Characterization & AWD Protection
 */

#ifndef FOC_CALIBRATION_H
#define FOC_CALIBRATION_H

#include <stdint.h>

#define CAL_SAMPLES 768

/**
 * @brief Reset calibration accumulators and min/max statistics
 */
void FOC_Calibration_Reset(void);

/**
 * @brief Accumulate sample data during calibration step
 * @param adc1_data Raw ADC1 measurement
 * @param adc2_data Raw ADC2 measurement
 * @param cal_phase Active calibration phase skip index (0 = Skip A, 1 = Skip B, 2 = Skip C)
 */
void FOC_Calibration_Accumulate(uint16_t adc1_data, uint16_t adc2_data, uint8_t cal_phase);

/**
 * @brief State handler for FOC_STATE_CALIBRATION (computes offsets, noise profile and configures AWD)
 */
void FOC_StateCalibration(void);

/**
 * @brief Configure hardware analog watchdogs (AWD1 on ADC1 & ADC2) based on measured offsets
 */
void FOC_ConfigureAWD(void);

/**
 * @brief Calculate physical minimum observer speed based on noise floor, deadtime & SNR constraints
 * @return Minimum observer speed in RPM
 */
float FOC_CalculateObserverMinSpeed(void);

#endif /* FOC_CALIBRATION_H */
