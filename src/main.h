/*
 * main.h
 *
 *  Created on: Apr 3, 2019
 *      Author: Mohit
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "display.h"
#include "em_core.h"
#include "gpio.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Header files required for display */
#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

// Count for logging time stamp
uint32_t msecCount;

/// For indexing elements of the node (this assignment requires only one element)
static uint16 _elem_index = 0x00;

/// number of active Bluetooth connections
static uint8 num_connections = 0;

/* Hardware soft timer handles */
#define TIMER_ID_FACTORY_RESET 	(0x01)
#define TIMER_ID_RESTART 		(0x02)
#define DISPLAY_UPDATE 			(0x04)
#define LOG_TIME_UPDATE 		(0x08)
#define TIMER_ID_FRIEND_FIND 	(0x10)
#define IR1_TIMEOUT_FLAG 		(0x20)
#define IR2_TIMEOUT_FLAG 		(0x40)
#define VIB_TIMEOUT_FLAG 		(0x80)
#define FRIEND_ALERT			(0x81)

/* External signal flags */
#define PB0_FLAG 				(0x01)
#define PB1_FLAG 				(0x02)
#define IR1_FLAG 				(0x04)
#define IR2_FLAG 				(0x08)
#define VIB_FLAG 				(0x10)
#define RELAY_FLAG				(0x20)

/* MACROS FOR DATA SENT VIA MODELS */
#define PB0_STOP_ALERT			(0x01)		// LEVEL model
#define VIBRATION_ALERT 		(0x0A)		// LEVEL model
#define LIGHT_CONTROL_ON		(0x1B)		// LEVEL model
#define LIGHT_CONTROL_OFF		(0x2B)		// LEVEL model
#define GAS_ALERT				(0x0C)		// LEVEL model
#define FIRE_ALERT 				(0x0D)		// LEVEL model
#define NOISE_ALERT 			(0x0E)		// LEVEL model
#define HUMIDITY_ALERT 			(0x0F)		// LEVEL model

#if 0
#define PB0_STOP_ALERT			(0x01)		// ON_OFF model
#define GAS_ALERT				(0x00)		// ON_OFF model
#endif


void set_device_name(bd_addr *pAddr);

static void init_models(void);

static void onoff(uint16_t model_id,
					uint16_t element_index,
					uint16_t client_addr,
					uint16_t server_addr,
					const struct mesh_generic_state *current,
					const struct mesh_generic_state *target,
					uint32_t remaining_ms,
					uint8_t response_flags);

static void level(uint16_t model_id,
					uint16_t element_index,
					uint16_t client_addr,
					uint16_t server_addr,
					const struct mesh_generic_state *current,
					const struct mesh_generic_state *target,
					uint32_t remaining_ms,
					uint8_t response_flags);

#endif /* SRC_MAIN_H_ */
