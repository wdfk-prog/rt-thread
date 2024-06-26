/*
 * Copyright (c) 2021-2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#ifndef HPM_TRGMMUX_SRC_H
#define HPM_TRGMMUX_SRC_H

/* trgm0_input mux definitions */
#define HPM_TRGM0_INPUT_SRC_VSS                            (0x0UL)
#define HPM_TRGM0_INPUT_SRC_VDD                            (0x1UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P0                       (0x2UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P1                       (0x3UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P2                       (0x4UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P3                       (0x5UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P4                       (0x6UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P5                       (0x7UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P6                       (0x8UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P7                       (0x9UL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P8                       (0xAUL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P9                       (0xBUL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P10                      (0xCUL)
#define HPM_TRGM0_INPUT_SRC_TRGM0_P11                      (0xDUL)
#define HPM_TRGM0_INPUT_SRC_TRGM1_OUTX0                    (0x12UL)
#define HPM_TRGM0_INPUT_SRC_TRGM1_OUTX1                    (0x13UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH8REF                    (0x14UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH9REF                    (0x15UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH10REF                   (0x16UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH11REF                   (0x17UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH12REF                   (0x18UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH13REF                   (0x19UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH14REF                   (0x1AUL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH15REF                   (0x1BUL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH16REF                   (0x1CUL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH17REF                   (0x1DUL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH18REF                   (0x1EUL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH19REF                   (0x1FUL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH20REF                   (0x20UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH21REF                   (0x21UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH22REF                   (0x22UL)
#define HPM_TRGM0_INPUT_SRC_PWM0_CH23REF                   (0x23UL)
#define HPM_TRGM0_INPUT_SRC_QEI0_TRGO                      (0x24UL)
#define HPM_TRGM0_INPUT_SRC_HALL0_TRGO                     (0x25UL)
#define HPM_TRGM0_INPUT_SRC_USB0_SOF                       (0x26UL)
#define HPM_TRGM0_INPUT_SRC_NTMR0_CH1_OUT                  (0x27UL)
#define HPM_TRGM0_INPUT_SRC_ENET0_PTP_OUT3                 (0x28UL)
#define HPM_TRGM0_INPUT_SRC_NTMR0_CH0_OUT                  (0x29UL)
#define HPM_TRGM0_INPUT_SRC_PTPC_CMP0                      (0x2AUL)
#define HPM_TRGM0_INPUT_SRC_PTPC_CMP1                      (0x2BUL)
#define HPM_TRGM0_INPUT_SRC_SYNT0_CH0                      (0x2CUL)
#define HPM_TRGM0_INPUT_SRC_SYNT0_CH1                      (0x2DUL)
#define HPM_TRGM0_INPUT_SRC_SYNT0_CH2                      (0x2EUL)
#define HPM_TRGM0_INPUT_SRC_SYNT0_CH3                      (0x2FUL)
#define HPM_TRGM0_INPUT_SRC_GPTMR0_OUT2                    (0x30UL)
#define HPM_TRGM0_INPUT_SRC_GPTMR0_OUT3                    (0x31UL)
#define HPM_TRGM0_INPUT_SRC_GPTMR1_OUT2                    (0x32UL)
#define HPM_TRGM0_INPUT_SRC_GPTMR1_OUT3                    (0x33UL)
#define HPM_TRGM0_INPUT_SRC_CMP0_OUT                       (0x34UL)
#define HPM_TRGM0_INPUT_SRC_CMP1_OUT                       (0x35UL)
#define HPM_TRGM0_INPUT_SRC_DEBUG_FLAG                     (0x38UL)

/* trgm1_input mux definitions */
#define HPM_TRGM1_INPUT_SRC_VSS                            (0x0UL)
#define HPM_TRGM1_INPUT_SRC_VDD                            (0x1UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P0                       (0x2UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P1                       (0x3UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P2                       (0x4UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P3                       (0x5UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P4                       (0x6UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P5                       (0x7UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P6                       (0x8UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P7                       (0x9UL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P8                       (0xAUL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P9                       (0xBUL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P10                      (0xCUL)
#define HPM_TRGM1_INPUT_SRC_TRGM1_P11                      (0xDUL)
#define HPM_TRGM1_INPUT_SRC_TRGM0_OUTX0                    (0x12UL)
#define HPM_TRGM1_INPUT_SRC_TRGM0_OUTX1                    (0x13UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH8REF                    (0x14UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH9REF                    (0x15UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH10REF                   (0x16UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH11REF                   (0x17UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH12REF                   (0x18UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH13REF                   (0x19UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH14REF                   (0x1AUL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH15REF                   (0x1BUL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH16REF                   (0x1CUL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH17REF                   (0x1DUL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH18REF                   (0x1EUL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH19REF                   (0x1FUL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH20REF                   (0x20UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH21REF                   (0x21UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH22REF                   (0x22UL)
#define HPM_TRGM1_INPUT_SRC_PWM1_CH23REF                   (0x23UL)
#define HPM_TRGM1_INPUT_SRC_QEI1_TRGO                      (0x24UL)
#define HPM_TRGM1_INPUT_SRC_HALL1_TRGO                     (0x25UL)
#define HPM_TRGM1_INPUT_SRC_USB0_SOF                       (0x26UL)
#define HPM_TRGM1_INPUT_SRC_NTMR0_CH1_OUT                  (0x27UL)
#define HPM_TRGM1_INPUT_SRC_ENET0_PTP_OUT3                 (0x28UL)
#define HPM_TRGM1_INPUT_SRC_NTMR0_CH0_OUT                  (0x29UL)
#define HPM_TRGM1_INPUT_SRC_PTPC_CMP0                      (0x2AUL)
#define HPM_TRGM1_INPUT_SRC_PTPC_CMP1                      (0x2BUL)
#define HPM_TRGM1_INPUT_SRC_SYNT0_CH0                      (0x2CUL)
#define HPM_TRGM1_INPUT_SRC_SYNT0_CH1                      (0x2DUL)
#define HPM_TRGM1_INPUT_SRC_SYNT0_CH2                      (0x2EUL)
#define HPM_TRGM1_INPUT_SRC_SYNT0_CH3                      (0x2FUL)
#define HPM_TRGM1_INPUT_SRC_GPTMR2_OUT2                    (0x30UL)
#define HPM_TRGM1_INPUT_SRC_GPTMR2_OUT3                    (0x31UL)
#define HPM_TRGM1_INPUT_SRC_GPTMR3_OUT2                    (0x32UL)
#define HPM_TRGM1_INPUT_SRC_GPTMR3_OUT3                    (0x33UL)
#define HPM_TRGM1_INPUT_SRC_CMP0_OUT                       (0x34UL)
#define HPM_TRGM1_INPUT_SRC_CMP1_OUT                       (0x35UL)
#define HPM_TRGM1_INPUT_SRC_DEBUG_FLAG                     (0x38UL)

/* trgm0_output mux definitions */
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P0                      (0x0UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P1                      (0x1UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P2                      (0x2UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P3                      (0x3UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P4                      (0x4UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P5                      (0x5UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P6                      (0x6UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P7                      (0x7UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P8                      (0x8UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P9                      (0x9UL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P10                     (0xAUL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_P11                     (0xBUL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_OUTX0                   (0xCUL)
#define HPM_TRGM0_OUTPUT_SRC_TRGM0_OUTX1                   (0xDUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_SYNCI                    (0xEUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_FRCI                     (0xFUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_FRCSYNCI                 (0x10UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_SHRLDSYNCI               (0x11UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_FAULTI0                  (0x12UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_FAULTI1                  (0x13UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_FAULTI2                  (0x14UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_FAULTI3                  (0x15UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN8                      (0x16UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN9                      (0x17UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN10                     (0x18UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN11                     (0x19UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN12                     (0x1AUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN13                     (0x1BUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN14                     (0x1CUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN15                     (0x1DUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN16                     (0x1EUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN17                     (0x1FUL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN18                     (0x20UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN19                     (0x21UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN20                     (0x22UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN21                     (0x23UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN22                     (0x24UL)
#define HPM_TRGM0_OUTPUT_SRC_PWM0_IN23                     (0x25UL)
#define HPM_TRGM0_OUTPUT_SRC_QEI0_A                        (0x26UL)
#define HPM_TRGM0_OUTPUT_SRC_QEI0_B                        (0x27UL)
#define HPM_TRGM0_OUTPUT_SRC_QEI0_Z                        (0x28UL)
#define HPM_TRGM0_OUTPUT_SRC_QEI0_H                        (0x29UL)
#define HPM_TRGM0_OUTPUT_SRC_QEI0_PAUSE                    (0x2AUL)
#define HPM_TRGM0_OUTPUT_SRC_QEI0_SNAPI                    (0x2BUL)
#define HPM_TRGM0_OUTPUT_SRC_HALL0_U                       (0x2CUL)
#define HPM_TRGM0_OUTPUT_SRC_HALL0_V                       (0x2DUL)
#define HPM_TRGM0_OUTPUT_SRC_HALL0_W                       (0x2EUL)
#define HPM_TRGM0_OUTPUT_SRC_HALL0_SNAPI                   (0x2FUL)
#define HPM_TRGM0_OUTPUT_SRC_ADC0_STRGI_ADCX_PTRGI2A       (0x30UL)
#define HPM_TRGM0_OUTPUT_SRC_ADC1_STRGI_ADCX_PTRGI2B       (0x31UL)
#define HPM_TRGM0_OUTPUT_SRC_ADC2_STRGI_ADCX_PTRGI2C       (0x32UL)
#define HPM_TRGM0_OUTPUT_SRC_DAC_BUFF_TRIGGER              (0x33UL)
#define HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A                  (0x34UL)
#define HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B                  (0x35UL)
#define HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0C                  (0x36UL)
#define HPM_TRGM0_OUTPUT_SRC_GPTMR0_SYNCI                  (0x37UL)
#define HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN2                    (0x38UL)
#define HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN3                    (0x39UL)
#define HPM_TRGM0_OUTPUT_SRC_GPTMR1_SYNCI                  (0x3AUL)
#define HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN2                    (0x3BUL)
#define HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN3                    (0x3CUL)
#define HPM_TRGM0_OUTPUT_SRC_ACMP0_WIN                     (0x3DUL)
#define HPM_TRGM0_OUTPUT_SRC_PTPC_CAP0                     (0x3EUL)
#define HPM_TRGM0_OUTPUT_SRC_PTPC_CAP1                     (0x3FUL)
#define HPM_TRGM0_OUTPUT_SRC_DAC_STEP_TRIGGER_IN0          (0x40UL)
#define HPM_TRGM0_OUTPUT_SRC_DAC_STEP_TRIGGER_IN1          (0x41UL)
#define HPM_TRGM0_OUTPUT_SRC_DAC_STEP_TRIGGER_IN2          (0x42UL)
#define HPM_TRGM0_OUTPUT_SRC_DAC_STEP_TRIGGER_IN3          (0x43UL)

/* trgm1_output mux definitions */
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P0                      (0x0UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P1                      (0x1UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P2                      (0x2UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P3                      (0x3UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P4                      (0x4UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P5                      (0x5UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P6                      (0x6UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P7                      (0x7UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P8                      (0x8UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P9                      (0x9UL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P10                     (0xAUL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_P11                     (0xBUL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_OUTX0                   (0xCUL)
#define HPM_TRGM1_OUTPUT_SRC_TRGM1_OUTX1                   (0xDUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_SYNCI                    (0xEUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_FRCI                     (0xFUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_FRCSYNCI                 (0x10UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_SHRLDSYNCI               (0x11UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_FAULTI0                  (0x12UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_FAULTI1                  (0x13UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_FAULTI2                  (0x14UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_FAULTI3                  (0x15UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN8                      (0x16UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN9                      (0x17UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN10                     (0x18UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN11                     (0x19UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN12                     (0x1AUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN13                     (0x1BUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN14                     (0x1CUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN15                     (0x1DUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN16                     (0x1EUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN17                     (0x1FUL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN18                     (0x20UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN19                     (0x21UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN20                     (0x22UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN21                     (0x23UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN22                     (0x24UL)
#define HPM_TRGM1_OUTPUT_SRC_PWM1_IN23                     (0x25UL)
#define HPM_TRGM1_OUTPUT_SRC_QEI1_A                        (0x26UL)
#define HPM_TRGM1_OUTPUT_SRC_QEI1_B                        (0x27UL)
#define HPM_TRGM1_OUTPUT_SRC_QEI1_Z                        (0x28UL)
#define HPM_TRGM1_OUTPUT_SRC_QEI1_H                        (0x29UL)
#define HPM_TRGM1_OUTPUT_SRC_QEI1_PAUSE                    (0x2AUL)
#define HPM_TRGM1_OUTPUT_SRC_QEI1_SNAPI                    (0x2BUL)
#define HPM_TRGM1_OUTPUT_SRC_HALL1_U                       (0x2CUL)
#define HPM_TRGM1_OUTPUT_SRC_HALL1_V                       (0x2DUL)
#define HPM_TRGM1_OUTPUT_SRC_HALL1_W                       (0x2EUL)
#define HPM_TRGM1_OUTPUT_SRC_HALL1_SNAPI                   (0x2FUL)
#define HPM_TRGM1_OUTPUT_SRC_ADC0_STRGI_ADCX_PTRGI3A       (0x30UL)
#define HPM_TRGM1_OUTPUT_SRC_ADC1_STRGI_ADCX_PTRGI3B       (0x31UL)
#define HPM_TRGM1_OUTPUT_SRC_ADC2_STRGI_ADCX_PTRGI3C       (0x32UL)
#define HPM_TRGM1_OUTPUT_SRC_DAC_BUFF_TRIGGER              (0x33UL)
#define HPM_TRGM1_OUTPUT_SRC_ADCX_PTRGI1A                  (0x34UL)
#define HPM_TRGM1_OUTPUT_SRC_ADCX_PTRGI1B                  (0x35UL)
#define HPM_TRGM1_OUTPUT_SRC_ADCX_PTRGI1C                  (0x36UL)
#define HPM_TRGM1_OUTPUT_SRC_GPTMR2_SYNCI                  (0x37UL)
#define HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN2                    (0x38UL)
#define HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN3                    (0x39UL)
#define HPM_TRGM1_OUTPUT_SRC_GPTMR3_SYNCI                  (0x3AUL)
#define HPM_TRGM1_OUTPUT_SRC_GPTMR3_IN2                    (0x3BUL)
#define HPM_TRGM1_OUTPUT_SRC_GPTMR3_IN3                    (0x3CUL)
#define HPM_TRGM1_OUTPUT_SRC_ACMP1_WIN                     (0x3DUL)
#define HPM_TRGM1_OUTPUT_SRC_PTPC_CAP0                     (0x3EUL)
#define HPM_TRGM1_OUTPUT_SRC_PTPC_CAP1                     (0x3FUL)
#define HPM_TRGM1_OUTPUT_SRC_DAC_STEP_TRIGGER_IN0          (0x40UL)
#define HPM_TRGM1_OUTPUT_SRC_DAC_STEP_TRIGGER_IN1          (0x41UL)
#define HPM_TRGM1_OUTPUT_SRC_DAC_STEP_TRIGGER_IN2          (0x42UL)
#define HPM_TRGM1_OUTPUT_SRC_DAC_STEP_TRIGGER_IN3          (0x43UL)

/* trgm0_filter mux definitions */
#define HPM_TRGM0_FILTER_SRC_PWM0_IN0                      (0x0UL)
#define HPM_TRGM0_FILTER_SRC_PWM0_IN1                      (0x1UL)
#define HPM_TRGM0_FILTER_SRC_PWM0_IN2                      (0x2UL)
#define HPM_TRGM0_FILTER_SRC_PWM0_IN3                      (0x3UL)
#define HPM_TRGM0_FILTER_SRC_PWM0_IN4                      (0x4UL)
#define HPM_TRGM0_FILTER_SRC_PWM0_IN5                      (0x5UL)
#define HPM_TRGM0_FILTER_SRC_PWM0_IN6                      (0x6UL)
#define HPM_TRGM0_FILTER_SRC_PWM0_IN7                      (0x7UL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN0                     (0x8UL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN1                     (0x9UL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN2                     (0xAUL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN3                     (0xBUL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN4                     (0xCUL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN5                     (0xDUL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN6                     (0xEUL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN7                     (0xFUL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN8                     (0x10UL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN9                     (0x11UL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN10                    (0x12UL)
#define HPM_TRGM0_FILTER_SRC_TRGM0_IN11                    (0x13UL)

/* trgm1_filter mux definitions */
#define HPM_TRGM1_FILTER_SRC_PWM1_IN0                      (0x0UL)
#define HPM_TRGM1_FILTER_SRC_PWM1_IN1                      (0x1UL)
#define HPM_TRGM1_FILTER_SRC_PWM1_IN2                      (0x2UL)
#define HPM_TRGM1_FILTER_SRC_PWM1_IN3                      (0x3UL)
#define HPM_TRGM1_FILTER_SRC_PWM1_IN4                      (0x4UL)
#define HPM_TRGM1_FILTER_SRC_PWM1_IN5                      (0x5UL)
#define HPM_TRGM1_FILTER_SRC_PWM1_IN6                      (0x6UL)
#define HPM_TRGM1_FILTER_SRC_PWM1_IN7                      (0x7UL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN0                     (0x8UL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN1                     (0x9UL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN2                     (0xAUL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN3                     (0xBUL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN4                     (0xCUL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN5                     (0xDUL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN6                     (0xEUL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN7                     (0xFUL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN8                     (0x10UL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN9                     (0x11UL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN10                    (0x12UL)
#define HPM_TRGM1_FILTER_SRC_TRGM1_IN11                    (0x13UL)

/* trgm0_dma mux definitions */
#define HPM_TRGM0_DMA_SRC_PWM0_CMP0                        (0x0UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP1                        (0x1UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP2                        (0x2UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP3                        (0x3UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP4                        (0x4UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP5                        (0x5UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP6                        (0x6UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP7                        (0x7UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP8                        (0x8UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP9                        (0x9UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP10                       (0xAUL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP11                       (0xBUL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP12                       (0xCUL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP13                       (0xDUL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP14                       (0xEUL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP15                       (0xFUL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP16                       (0x10UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP17                       (0x11UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP18                       (0x12UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP19                       (0x13UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP20                       (0x14UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP21                       (0x15UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP22                       (0x16UL)
#define HPM_TRGM0_DMA_SRC_PWM0_CMP23                       (0x17UL)
#define HPM_TRGM0_DMA_SRC_PWM0_RLD                         (0x18UL)
#define HPM_TRGM0_DMA_SRC_PWM0_HALFRLD                     (0x19UL)
#define HPM_TRGM0_DMA_SRC_PWM0_XRLD                        (0x1AUL)
#define HPM_TRGM0_DMA_SRC_QEI0                             (0x1BUL)
#define HPM_TRGM0_DMA_SRC_HALL0                            (0x1CUL)

/* trgm1_dma mux definitions */
#define HPM_TRGM1_DMA_SRC_PWM1_CMP0                        (0x0UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP1                        (0x1UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP2                        (0x2UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP3                        (0x3UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP4                        (0x4UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP5                        (0x5UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP6                        (0x6UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP7                        (0x7UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP8                        (0x8UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP9                        (0x9UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP10                       (0xAUL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP11                       (0xBUL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP12                       (0xCUL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP13                       (0xDUL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP14                       (0xEUL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP15                       (0xFUL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP16                       (0x10UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP17                       (0x11UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP18                       (0x12UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP19                       (0x13UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP20                       (0x14UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP21                       (0x15UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP22                       (0x16UL)
#define HPM_TRGM1_DMA_SRC_PWM1_CMP23                       (0x17UL)
#define HPM_TRGM1_DMA_SRC_PWM1_RLD                         (0x18UL)
#define HPM_TRGM1_DMA_SRC_PWM1_HALFRLD                     (0x19UL)
#define HPM_TRGM1_DMA_SRC_PWM1_XRLD                        (0x1AUL)
#define HPM_TRGM1_DMA_SRC_QEI1                             (0x1BUL)
#define HPM_TRGM1_DMA_SRC_HALL1                            (0x1CUL)



#endif /* HPM_TRGMMUX_SRC_H */
