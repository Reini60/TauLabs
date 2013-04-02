/**
 *****************************************************************************
 * @file       pios_board.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://www.taulabs.org, Copyright (C) 2012-2013
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @{
 * @brief Defines board specific static initializers for hardware for the CopterControl board.
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.  
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */
#include "board_hw_defs.c"

#include <pios.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include <hwnaze32.h>
#include <modulesettings.h>
#include <manualcontrolsettings.h>
#include <gcsreceiver.h>


/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, DSMMAINPORT, DSMFLEXIPORT, SBUS
 * NOTE: No slot in this map for NONE.
 */
uint32_t pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE];

#define PIOS_COM_TELEM_RF_RX_BUF_LEN 32
#define PIOS_COM_TELEM_RF_TX_BUF_LEN 12

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
#define PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN 40
uintptr_t pios_com_debug_id;
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */

uintptr_t pios_com_telem_rf_id;

/**
 * Configuration for MPU6050 chip
 */
#if defined(PIOS_INCLUDE_MPU6050)
#include "pios_mpu6050.h"
static const struct pios_exti_cfg pios_exti_mpu6050_cfg __exti_config = {
	.vector = PIOS_MPU6050_IRQHandler,
	.line = EXTI_Line13,
	.pin = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_13,
			.GPIO_Speed = GPIO_Speed_10MHz,
			.GPIO_Mode  = GPIO_Mode_IN_FLOATING,
		},
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = EXTI15_10_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.exti = {
		.init = {
			.EXTI_Line = EXTI_Line13, // matches above GPIO pin
			.EXTI_Mode = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising,
			.EXTI_LineCmd = ENABLE,
		},
	},
};

static const struct pios_mpu60x0_cfg pios_mpu6050_cfg = {
	.exti_cfg = &pios_exti_mpu6050_cfg,
	.Fifo_store = PIOS_MPU60X0_FIFO_TEMP_OUT | PIOS_MPU60X0_FIFO_GYRO_X_OUT | PIOS_MPU60X0_FIFO_GYRO_Y_OUT | PIOS_MPU60X0_FIFO_GYRO_Z_OUT,
	// Clock at 8 khz, downsampled by 20 for 400 Hz
	.Smpl_rate_div = 19,
	.interrupt_cfg = PIOS_MPU60X0_INT_CLR_ANYRD,
	.interrupt_en = PIOS_MPU60X0_INTEN_DATA_RDY,
	.User_ctl = PIOS_MPU60X0_USERCTL_FIFO_EN,
	.Pwr_mgmt_clk = PIOS_MPU60X0_PWRMGMT_PLL_X_CLK,
	.filter = PIOS_MPU60X0_LOWPASS_256_HZ,
	.orientation = PIOS_MPU60X0_TOP_90DEG
};
#endif /* PIOS_INCLUDE_MPU6050 */

#include <pios_board_info.h>
/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */
//int32_t init_test;
void PIOS_Board_Init(void) {

	/* Delay system */
	PIOS_DELAY_Init();

#if defined(PIOS_INCLUDE_LED)
	const struct pios_led_cfg * led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(1);
	PIOS_Assert(led_cfg);
	PIOS_LED_Init(led_cfg);
#endif	/* PIOS_INCLUDE_LED */

#if defined(PIOS_INCLUDE_SPI)
	/* Set up the SPI interface to the serial flash */

	if (PIOS_SPI_Init(&pios_spi_generic_id, &pios_spi_generic_cfg)) {
		PIOS_Assert(0);
	}
#endif

#if defined(PIOS_INCLUDE_I2C)
	/* Set up the I2C interface */
	if (PIOS_I2C_Init(&pios_i2c_adapter_id, &pios_i2c_adapter_cfg)) {
		PIOS_Assert(0);
	}
#endif	/* PIOS_INCLUDE_I2C */

#if defined(PIOS_INCLUDE_FLASH)
	/* Connect flash to the appropriate interface and configure it */
	uintptr_t flash_id;
	PIOS_Flash_Internal_Init(&flash_id, &flash_internal_cfg);
	uintptr_t fs_id;
	PIOS_FLASHFS_Logfs_Init(&fs_id, &flashfs_internal_cfg, &pios_internal_flash_driver, flash_id);
#endif

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();

#if defined(PIOS_INCLUDE_RTC)
	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif

	HwNaze32Initialize();
	ModuleSettingsInitialize();

#ifndef ERASE_FLASH
	/* Initialize watchdog as early as possible to catch faults during init */
	PIOS_WDG_Init();
#endif

	/* Initialize the alarms library */
	AlarmsInitialize();

	/* Check for repeated boot failures */
	PIOS_IAP_Init();
	uint16_t boot_count = PIOS_IAP_ReadBootCount();
	if (boot_count < 3) {
		PIOS_IAP_WriteBootCount(++boot_count);
		AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
	} else {
		/* Too many failed boot attempts, force hw configuration to defaults */
		HwNaze32SetDefaults(HwNaze32Handle(), 0);
		ModuleSettingsSetDefaults(ModuleSettingsHandle(),0);
		AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
	}

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* Set up pulse timers */
	PIOS_TIM_InitClock(&tim_1_cfg);
	PIOS_TIM_InitClock(&tim_2_cfg);
	PIOS_TIM_InitClock(&tim_3_cfg);
	PIOS_TIM_InitClock(&tim_4_cfg);

#if defined(PIOS_INCLUDE_TELEMETRY_RF)
	uint32_t pios_usart_generic_id;
	if (PIOS_USART_Init(&pios_usart_generic_id, &pios_usart_generic_main_cfg)) {
		PIOS_Assert(0);
	}

	uint8_t * rx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_TELEM_RF_RX_BUF_LEN);
	uint8_t * tx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_TELEM_RF_TX_BUF_LEN);
	PIOS_Assert(rx_buffer);
	PIOS_Assert(tx_buffer);
	if (PIOS_COM_Init(&pios_com_telem_rf_id, &pios_usart_com_driver, pios_usart_generic_id,
			  rx_buffer, PIOS_COM_TELEM_RF_RX_BUF_LEN,
			  tx_buffer, PIOS_COM_TELEM_RF_TX_BUF_LEN)) {
		PIOS_Assert(0);
	}
#endif	/* PIOS_INCLUDE_TELEMETRY_RF */


	/* Configure the rcvr port */
	uint8_t hw_rcvrport;
	HwNaze32RcvrPortGet(&hw_rcvrport);

	switch (hw_rcvrport) {
	case HWNAZE32_RCVRPORT_DISABLED:
		break;
	case HWNAZE32_RCVRPORT_PWM:
#if defined(PIOS_INCLUDE_PWM)
		{
			uint32_t pios_pwm_id;
			PIOS_PWM_Init(&pios_pwm_id, &pios_pwm_cfg);

			uint32_t pios_pwm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_pwm_rcvr_id, &pios_pwm_rcvr_driver, pios_pwm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PWM] = pios_pwm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PWM */
		break;
	case HWNAZE32_RCVRPORT_PPM:
	case HWNAZE32_RCVRPORT_PPMOUTPUTS:
#if defined(PIOS_INCLUDE_PPM)
		{
			uint32_t pios_ppm_id;
			PIOS_PPM_Init(&pios_ppm_id, &pios_ppm_cfg);

			uint32_t pios_ppm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, pios_ppm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PPM */
		break;
	case HWNAZE32_RCVRPORT_PPMPWM:
		/* This is a combination of PPM and PWM inputs */
#if defined(PIOS_INCLUDE_PPM)
		{
			uint32_t pios_ppm_id;
			PIOS_PPM_Init(&pios_ppm_id, &pios_ppm_cfg);

			uint32_t pios_ppm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, pios_ppm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PPM */
#if defined(PIOS_INCLUDE_PWM)
		{
			uint32_t pios_pwm_id;
			PIOS_PWM_Init(&pios_pwm_id, &pios_pwm_with_ppm_cfg);

			uint32_t pios_pwm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_pwm_rcvr_id, &pios_pwm_rcvr_driver, pios_pwm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PWM] = pios_pwm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PWM */
		break;
	}

#if defined(PIOS_INCLUDE_GCSRCVR)
	GCSReceiverInitialize();
	uint32_t pios_gcsrcvr_id;
	PIOS_GCSRCVR_Init(&pios_gcsrcvr_id);
	uint32_t pios_gcsrcvr_rcvr_id;
	if (PIOS_RCVR_Init(&pios_gcsrcvr_rcvr_id, &pios_gcsrcvr_rcvr_driver, pios_gcsrcvr_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_GCS] = pios_gcsrcvr_rcvr_id;
#endif	/* PIOS_INCLUDE_GCSRCVR */

	/* Remap AFIO pin for PB4 (Servo 5 Out)*/
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_NoJTRST, ENABLE);

#ifndef PIOS_DEBUG_ENABLE_DEBUG_PINS
	switch (hw_rcvrport) {
		case HWNAZE32_RCVRPORT_DISABLED:
		case HWNAZE32_RCVRPORT_PWM:
		case HWNAZE32_RCVRPORT_PPM:
		case HWNAZE32_RCVRPORT_PPMPWM:
			PIOS_Servo_Init(&pios_servo_cfg);
			break;
		case HWNAZE32_RCVRPORT_PPMOUTPUTS:
		case HWNAZE32_RCVRPORT_OUTPUTS:
			PIOS_Servo_Init(&pios_servo_rcvr_cfg);
			break;
	}
#else
	PIOS_DEBUG_Init(&pios_tim_servo_all_channels, NELEMENTS(pios_tim_servo_all_channels));
#endif	/* PIOS_DEBUG_ENABLE_DEBUG_PINS */

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

#if defined(PIOS_INCLUDE_MPU6050) && defined(PIOS_INCLUDE_I2C)
	PIOS_MPU6050_Init(pios_i2c_adapter_id, PIOS_MPU6050_I2C_ADD_A0_LOW, &pios_mpu6050_cfg);
	//init_test = PIOS_MPU6050_Test();

	uint8_t hw_gyro_range;
	HwNaze32GyroRangeGet(&hw_gyro_range);
	switch(hw_gyro_range) {
		case HWNAZE32_GYRORANGE_250:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_250_DEG);
			break;
		case HWNAZE32_GYRORANGE_500:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_500_DEG);
			break;
		case HWNAZE32_GYRORANGE_1000:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_1000_DEG);
			break;
		case HWNAZE32_GYRORANGE_2000:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_2000_DEG);
			break;
	}

	uint8_t hw_accel_range;
	HwNaze32AccelRangeGet(&hw_accel_range);
	switch(hw_accel_range) {
		case HWNAZE32_ACCELRANGE_2G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_2G);
			break;
		case HWNAZE32_ACCELRANGE_4G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_4G);
			break;
		case HWNAZE32_ACCELRANGE_8G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_8G);
			break;
		case HWNAZE32_ACCELRANGE_16G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_16G);
			break;
	}
/*	
	uint8_t hw_mpu6050_rate;
	HwNaze32MPU6050RateGet(&hw_mpu6050_rate);
	uint16_t hw_mpu6050_divisor = \
		(hw_mpu6050_rate == HWNAZE32_MPU6050RATE_500) ? 15 : \
		(hw_mpu6050_rate == HWNAZE32_MPU6050RATE_666) ? 11 : \
		(hw_mpu6050_rate == HWNAZE32_MPU6050RATE_1000) ? 7 : \
		(hw_mpu6050_rate == HWNAZE32_MPU6050RATE_2000) ? 3 : \
		(hw_mpu6050_rate == HWNAZE32_MPU6050RATE_4000) ? 1 : \
		(hw_mpu6050_rate == HWNAZE32_MPU6050RATE_8000) ? 0 : \
		15;
	PIOS_MPU6050_SetDivisor(hw_mpu6050_divisor);

	uint8_t hw_dlpf;
	HwNaze32MPU6050DLPFGet(&hw_dlpf);
	uint16_t hw_mpu6050_dlpf = \
		(hw_dlpf == HWNAZE32_MPU6050DLPF_256) ? PIOS_MPU60X0_LOWPASS_256_HZ : \
		(hw_dlpf == HWNAZE32_MPU6050DLPF_188) ? PIOS_MPU60X0_LOWPASS_188_HZ : \
		(hw_dlpf == HWNAZE32_MPU6050DLPF_98) ? PIOS_MPU60X0_LOWPASS_98_HZ : \
		(hw_dlpf == HWNAZE32_MPU6050DLPF_42) ? PIOS_MPU60X0_LOWPASS_42_HZ : \
		(hw_dlpf == HWNAZE32_MPU6050DLPF_20) ? PIOS_MPU60X0_LOWPASS_20_HZ : \
		(hw_dlpf == HWNAZE32_MPU6050DLPF_10) ? PIOS_MPU60X0_LOWPASS_10_HZ : \
		(hw_dlpf == HWNAZE32_MPU6050DLPF_5) ? PIOS_MPU60X0_LOWPASS_5_HZ : \
		PIOS_MPU60X0_LOWPASS_256_HZ;
	PIOS_MPU6050_SetLPF(hw_mpu6050_dlpf);
*/
#endif /* PIOS_INCLUDE_MPU6050 */

	PIOS_GPIO_Init();

	/* Make sure we have at least one telemetry link configured or else fail initialization */
	PIOS_Assert(pios_com_telem_rf_id);
}

/**
 * @}
 */
