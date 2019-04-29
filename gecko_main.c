/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

/* Display header */
#include "src/display.h"

/* GPIO header */
#include "src/gpio.h"

/* Log header */
#include "src/log.h"

#include "src/main.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"

/* Variables required for project */
/* variable to store flash load data - needs to be global */
uint8_t flashData[DISPLAY_ALERT_DATA_LENGTH];
uint8_t uintArray[DISPLAY_ALERT_DATA_LENGTH];
char charArray[DISPLAY_ALERT_DATA_LENGTH];

/* People count using IR Sensor */
uint8_t peopleCount = 0;
uint8_t* peopleCountPtr;
uint8_t ir1Value = 0;
uint8_t ir2Value = 0;

/* Lights/LED0 state */
uint8_t lightState = 0;
uint8_t* lightStatePtr;

/* Sensors activation flag - to remove false notifications */
uint8_t ir1ActivationFlag = 1;
uint8_t ir2ActivationFlag = 1;
uint8_t vibActivationFlag = 1;

/* Buzzer and LED1 toggle count */
uint8_t toggleCount = 0;
uint8_t* toggleCountPtr;

/* Character array to store display message */
char* displayString;
uint8_t* displayBuffer;

/* Relay level from received from one LPN to external signal where it will be published */
uint8_t relayLevel;

/***********************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
		.sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
		.bluetooth.max_connections = MAX_CONNECTIONS,
		.bluetooth.max_advertisers = MAX_ADVERTISERS,
		.bluetooth.heap = bluetooth_stack_heap,
		.bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
		.bluetooth.sleep_clock_accuracy = 100,
		.bluetooth.linklayer_priorities = &linklayer_priorities,
		.gattdb = &bg_gattdb_data,
		.btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
		.pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
		.pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
		.pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
		.max_timers = 16,
};

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] pAddr  Pointer to Bluetooth address.
 ******************************************************************************/
void set_device_name(bd_addr *pAddr)
{
	char name[20];
	uint16 res;

	// create unique device name using the last two bytes of the Bluetooth address
	sprintf(name, "5823FRIEND %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

	DISPLAY_PRINTF(DISPLAY_ROW_NAME, "%s", name);

	DISPLAY_PRINTF(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x", pAddr-> addr[0], pAddr-> addr[1], pAddr-> addr[2], pAddr-> addr[3], pAddr-> addr[4], pAddr-> addr[5]);

	// write device name to the GATT database
	res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
}

/***************************************************************************//**
 * Initialization of the models supported by this node.
 * This function registers callbacks for each of the supported models.
 ******************************************************************************/
static void init_models(void)
{
	mesh_lib_generic_client_register_handler(MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
                                           0,
										   onoff);

	mesh_lib_generic_client_register_handler(MESH_GENERIC_LEVEL_CLIENT_MODEL_ID,
                                           0,
                                           level);
}

static void onoff(uint16_t model_id,
					uint16_t element_index,
					uint16_t client_addr,
					uint16_t server_addr,
					const struct mesh_generic_state *current,
					const struct mesh_generic_state *target,
					uint32_t remaining_ms,
					uint8_t response_flags)
{
	LOG_INFO("OnOff State Changed");
}

static void level(uint16_t model_id,
					uint16_t element_index,
					uint16_t client_addr,
					uint16_t server_addr,
					const struct mesh_generic_state *current,
					const struct mesh_generic_state *target,
					uint32_t remaining_ms,
					uint8_t response_flags)
{
	// stop alert
	if(current->level.level == PB0_STOP_ALERT)
	{
		toggleCount = 101;
		flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);
	}

	// gas alert
	if(current->level.level == GAS_ALERT)
	{
		DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, "GAS ALERT");
		// store alert display message in persistent memory
		displayString = "   GAS ALERT   ";
		flashStore(DISPLAY_ALERT_FLASH_ID, strToUint(displayString));

		toggleCount = 0;
		flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);

		gecko_cmd_hardware_set_soft_timer(3277, FRIEND_ALERT, 0);
	}

	// fire alert
	if(current->level.level == FIRE_ALERT)
	{
		DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, "  FIRE ALERT  ");
		displayString = "FIRE ALERT";
		flashStore(DISPLAY_ALERT_FLASH_ID, strToUint(displayString));

		toggleCount = 0;
		flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);
		gecko_cmd_hardware_set_soft_timer(3277, FRIEND_ALERT, 0);
	}

	// noise alert
	if(current->level.level == NOISE_ALERT)
	{
		DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, "  NOISE ALERT  ");
		displayString = "  NOISE ALERT  ";
		flashStore(DISPLAY_ALERT_FLASH_ID, strToUint(displayString));

		toggleCount = 0;
		flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);
		gecko_cmd_hardware_set_soft_timer(3277, FRIEND_ALERT, 0);
	}

	// humidity alert
	if(current->level.level == HUMIDITY_ALERT)
	{
		DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, "HUMIDITY ALERT");
		displayString = "HUMIDITY ALERT";
		flashStore(DISPLAY_ALERT_FLASH_ID, strToUint(displayString));

		toggleCount = 0;
		flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);
		gecko_cmd_hardware_set_soft_timer(3277, FRIEND_ALERT, 0);
	}

	// to relay data to another LPN
	relayLevel = current->level.level;
	gecko_external_signal(RELAY_FLAG);
}

/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 ******************************************************************************/
void lpn_init(void)
{
	uint16 res;
	// Initialize LPN functionality.
	res = gecko_cmd_mesh_lpn_init()->result;
	if (res) {
		LOG_ERROR("LPN init failed (0x%x)", res);
		return;
	}

	// Configure the lpn with following parameters:
	// - Minimum friend queue length = 2
	// - Poll timeout = 1 seconds
	res = gecko_cmd_mesh_lpn_configure(2, 1 * 1000)->result;
	if (res) {
		LOG_ERROR("LPN conf failed (0x%x)", res);
		return;
	}

	LOG_INFO("trying to find friend...");
	res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (res != 0) {
		LOG_ERROR("ret.code %x", res);
	}
}


void gecko_bgapi_classes_init_client_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	gecko_bgapi_class_mesh_generic_client_init();
	//gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
//	gecko_bgapi_class_mesh_lpn_init();
	gecko_bgapi_class_mesh_friend_init();

}
void gecko_main_init()
{
	// Initialize device
	initMcu();
	// Initialize board
	initBoard();
	// Initialize application
	initApp();

	// Minimize advertisement latency by allowing the advertiser to always
	// interrupt the scanner.
	linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

	gecko_stack_init(&config);

	gecko_bgapi_classes_init_client_lpn();

	// Initialize coexistence interface. Parameters are taken from HAL config.
	gecko_initCoexHAL();

}

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  switch (evt_id) {
  	// boot event
    case gecko_evt_system_boot_id:
    	// check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
		if (GPIO_PinInGet(PB0_PORT, PB0_PIN) == 0 || GPIO_PinInGet(PB1_PORT, PB1_PIN) == 0) {
			LOG_INFO("factory reset");
			DISPLAY_PRINTF(DISPLAY_ROW_ACTION, ">>>FACTORY RESET<<<");

			/* perform a factory reset by erasing PS storage. This removes all the keys and other settings
			that have been configured for this node */
			gecko_cmd_flash_ps_erase_all();

			// reboot after a small delay - 1 second
			gecko_cmd_hardware_set_soft_timer(2 * ONE_SEC_TICKS_CNT, TIMER_ID_FACTORY_RESET, 1);
		} else {
			struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
			set_device_name(&pAddr->address);

			// Initialize Mesh stack in Node operation mode, wait for initialized event
			gecko_cmd_mesh_node_init();
			LOG_INFO("BOOT DONE");

			// loading persistent data - People Count
			peopleCountPtr = flashLoad(PEOPLE_COUNT_FLASH_ID);
			peopleCount = *peopleCountPtr;

			// loading persistent data - Lights Status
			lightStatePtr = flashLoad(LIGHTS_STATUS_FLASH_ID);
			lightState = *lightStatePtr;

			// loading persistent data - Alert Status
			toggleCountPtr = flashLoad(ALERT_STATUS_FLASH_ID);
			toggleCount = *toggleCountPtr;

			// loading persistent data - Display Message
			displayBuffer = flashLoad(DISPLAY_ALERT_FLASH_ID);
			displayString = uintToStr(displayBuffer);
			LOG_INFO("displayString = %15s", displayString);
		}
		break;

	// node initialized event
    case gecko_evt_mesh_node_initialized_id:
		LOG_INFO("node initialized");

		struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

		if (pData->provisioned) {
			DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISIONED");

			/* Execution according to persistent data */
			// Display previous count of people
			if (peopleCount != 0)
				DISPLAY_PRINTF(DISPLAY_ROW_PEOPLE, "People Inside: %d", peopleCount);
			// Start previous alerts
			if (toggleCount != 0)
				gecko_cmd_hardware_set_soft_timer(3277, FRIEND_ALERT, 0);
			// Display previous alerts
			DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, "%15s", displayString);
			// Retrieve the lights status (LED0)
			lightState ? gpioLed0SetOn() : gpioLed0SetOff();

			LOG_INFO("node is provisioned. address:%x, ivi:%ld", pData->address, pData->ivi);

			mesh_lib_init(malloc,free,8);
			init_models();

			// initialized node as generic client
			gecko_cmd_mesh_generic_client_init();

			// do friend init
			uint16 res;
			res = gecko_cmd_mesh_friend_init()->result;
			if (res) {
				LOG_ERROR("Friend init failed 0x%x", res);
			}

			// enable gpio interrupts
			gpioIntEnable();
		}
		else {
			DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "UNPROVISIONED");
			LOG_INFO("node is unprovisioned");
			LOG_INFO("starting unprovisioned beaconing...");
			// The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
			gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
		}
		break;

	// provisioning started event
	case gecko_evt_mesh_node_provisioning_started_id:
		LOG_INFO("Started provisioning");
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISIONING");
		break;

	// provisioned failed event
	case gecko_evt_mesh_node_provisioned_id:
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISIONED");
		LOG_INFO("node is provisioned. address:%x, ivi:%ld", pData->address, pData->ivi);

		mesh_lib_init(malloc,free,8);
		init_models();

		// initialized node as generic client
		gecko_cmd_mesh_generic_client_init();

		// do friend init
		uint16 res;
		res = gecko_cmd_mesh_friend_init()->result;
		if (res) {
			LOG_ERROR("Friend init failed 0x%x", res);
		}

		// enable gpio interrupts
		gpioIntEnable();
		break;

	// provisioning failed event
	case gecko_evt_mesh_node_provisioning_failed_id:
		LOG_ERROR("provisioning failed, code %x", evt->data.evt_mesh_node_provisioning_failed.result);
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISION FAILED");
		/* start a one-shot timer that will trigger soft reset after small delay */
		gecko_cmd_hardware_set_soft_timer(2 * ONE_SEC_TICKS_CNT, TIMER_ID_RESTART, 1);
		break;

	// hardware soft timer event
    case gecko_evt_hardware_soft_timer_id:
    	switch (evt->data.evt_hardware_soft_timer.handle) {
    		case TIMER_ID_FACTORY_RESET:
    	          // reset the device to finish factory reset
    	          gecko_cmd_system_reset(0);
    	          break;

    		case TIMER_ID_RESTART:
    			// reset the device to finish factory reset
    			gecko_cmd_system_reset(0);
    			break;

    		// do displayUpdate every 1 second to remove charge buildup on LCD
			case DISPLAY_UPDATE:
				// Prevent charge buildup on LCD
				displayUpdate();
				break;

			// logger time stamp
			case LOG_TIME_UPDATE:
				// Increase millisecond count by 10
				msecCount += 10;
				break;

			// handle infrared sensor interrupts after 1 second
			case IR1_TIMEOUT_FLAG:
				ir1ActivationFlag = 1;
				break;

			case IR2_TIMEOUT_FLAG:
				ir2ActivationFlag = 1;
				break;

			// handle vibration interrupts after 1 second
			case VIB_TIMEOUT_FLAG:
				vibActivationFlag = 1;
				break;

			// toggles LED1 (alert led) and buzzer
			case FRIEND_ALERT:
				toggleCount++;
				flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);
				if(toggleCount % 2) {
					GPIO_PinOutSet(BUZZER_PORT, BUZZER_PIN);
					gpioLed1SetOn();
				}
				else {
					GPIO_PinOutClear(BUZZER_PORT, BUZZER_PIN);
					gpioLed1SetOff();

					// Stop timer after 100 toggles or 10 seconds
					if(toggleCount > 100) {
						toggleCount = 0;
						flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);
						gecko_cmd_hardware_set_soft_timer(0, FRIEND_ALERT, 0);

						// remove display alert
						DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, " ");
						displayString = "               ";
						displayBuffer = strToUint(displayString);
						flashStore(DISPLAY_ALERT_FLASH_ID, displayBuffer);
					}
				}
				break;
    	}
    	break;

    // external signal event
	case gecko_evt_system_external_signal_id:
		;

		struct mesh_generic_request req;

		//publish response
		uint16 resp;

		//transaction identifier
		static uint8 trid = 0;

		// PB0 button press
		if (((evt->data.evt_system_external_signal.extsignals) & PB0_FLAG) != 0) {
			// To turn off alert, make toggle count max
			toggleCount = 101;
			flashStore(ALERT_STATUS_FLASH_ID, &toggleCount);

			LOG_INFO("PB0 INT");

			// publish Alert Stop
			req.kind = mesh_generic_request_level;
			req.level = PB0_STOP_ALERT;
			trid++;
			resp = mesh_lib_generic_client_publish(MESH_GENERIC_LEVEL_CLIENT_MODEL_ID, _elem_index, trid, &req, 0, 0, 0);
			if (resp) {
				LOG_ERROR("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
			} else {
				LOG_INFO("request sent, trid = %u", trid);
			}
		}

		// Infrared Sensor 1 external event
		if(((evt->data.evt_system_external_signal.extsignals) & IR1_FLAG) != 0) {
			if(ir1ActivationFlag)
			{
				ir1ActivationFlag = 0;
				gecko_cmd_hardware_set_soft_timer(1 * ONE_SEC_TICKS_CNT, IR1_TIMEOUT_FLAG, 1);

				LOG_INFO("IR1 INT");

				if(ir2Value) {
					// person exiting cave
					ir2Value = 0;

					if(peopleCount)	{
						peopleCount--;
						flashStore(PEOPLE_COUNT_FLASH_ID, &peopleCount);
					}

					if(peopleCount == 0) {
						// turn off lights locally
						gpioLed0SetOff();
						lightState = 0;
						flashStore(LIGHTS_STATUS_FLASH_ID, &lightState);

						// publish lights off data when no person is in cave
						req.kind = mesh_generic_request_on_off;
						req.level = LIGHT_CONTROL_OFF;
						trid++;
						resp = mesh_lib_generic_client_publish(MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID, _elem_index, trid, &req, 0, 0, 0);
						if (resp) {
							LOG_INFO("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
						} else {
							LOG_INFO("request sent, trid = %u", trid);
						}
					}

					DISPLAY_PRINTF(DISPLAY_ROW_PEOPLE, "People Inside: %d", peopleCount);
					LOG_INFO("People Inside: %d", peopleCount);
				}
				else
					// person entering cave - next interrupt generated will be of IR2
					ir1Value = 1;
			}
		}

		// Infrared Sensor 2 external event
		if(((evt->data.evt_system_external_signal.extsignals) & IR2_FLAG) != 0) {
			if(ir2ActivationFlag)
			{
				ir2ActivationFlag = 0;
				gecko_cmd_hardware_set_soft_timer(1 * ONE_SEC_TICKS_CNT, IR2_TIMEOUT_FLAG, 1);

				LOG_INFO("IR2 INT");

				if(ir1Value) {
					// person entering cave
					ir1Value = 0;

					// increase people count
					peopleCount++;
					flashStore(PEOPLE_COUNT_FLASH_ID, &peopleCount);

					// turn on lights
					gpioLed0SetOn();
					lightState = 1;
					flashStore(LIGHTS_STATUS_FLASH_ID, &lightState);

					DISPLAY_PRINTF(DISPLAY_ROW_PEOPLE, "People Inside: %d", peopleCount);
					LOG_INFO("People Inside: %d", peopleCount);

					// publish lights on data when there is someone in cave
					if(peopleCount == 1)
					{
						req.kind = mesh_generic_request_on_off;
						req.level = LIGHT_CONTROL_ON;
						trid++;
						resp = mesh_lib_generic_client_publish(MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID, _elem_index, trid, &req, 0, 0, 0);
						if (resp) {
							LOG_ERROR("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
						} else {
							LOG_INFO("request sent, trid = %u", trid);
						}
					}
				}
				else
					// person exiting cave - next interrupt generated will be of IR1
					ir2Value = 1;
			}
		}

		// Vibration Sensor external event
		if(((evt->data.evt_system_external_signal.extsignals) & VIB_FLAG) != 0) {
			if(vibActivationFlag) {
				vibActivationFlag = 0;
				// Timeout of 5 sec
				gecko_cmd_hardware_set_soft_timer(5 * ONE_SEC_TICKS_CNT, VIB_TIMEOUT_FLAG, 1);

				DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, "  EARTHQUAKE  ");

				// store alert display message in persistent memory
				displayString = "  EARTHQUAKE  ";
				displayBuffer = strToUint(displayString);
				flashStore(DISPLAY_ALERT_FLASH_ID, displayBuffer);

				// Set alarm in case of earthquake
				toggleCount = 0;
				// Alert frequency 200 ms
				gecko_cmd_hardware_set_soft_timer(3277, FRIEND_ALERT, 0);

				LOG_INFO("VIB INT");

				// Publication of alert
				req.kind = mesh_generic_request_level;
				req.level = VIBRATION_ALERT;
				trid++;
				resp = mesh_lib_generic_client_publish(MESH_GENERIC_LEVEL_CLIENT_MODEL_ID, _elem_index, trid, &req, 0, 0, 0);
				if (resp) {
					LOG_ERROR("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
				} else {
					LOG_INFO("request sent, trid = %u", trid);
				}
			}
		}

		// Relay the data coming from one LPN to another LPN
		if(((evt->data.evt_system_external_signal.extsignals) & RELAY_FLAG) != 0) {
			// Publication of alert
			req.kind = mesh_generic_request_level;
			req.level = relayLevel;
			trid++;
			resp = mesh_lib_generic_client_publish(MESH_GENERIC_LEVEL_CLIENT_MODEL_ID, _elem_index, trid, &req, 0, 0, 0);
			if (resp) {
				LOG_ERROR("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
			} else {
				LOG_INFO("request sent, trid = %u", trid);
			}
		}
		break;

	// connection open event
	case gecko_evt_le_connection_opened_id:
		LOG_INFO("evt:gecko_evt_le_connection_opened_id");
		num_connections++;
		DISPLAY_PRINTF(DISPLAY_ROW_CONNECTION, "Connected");
		break;

	// connection close event
    case gecko_evt_le_connection_closed_id:
    	LOG_INFO("evt:gecko_evt_le_connection_closed_id");

		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		}

		if (num_connections > 0) {
			if (--num_connections == 0) {
				DISPLAY_PRINTF(DISPLAY_ROW_CONNECTION, " ");
			}
		}
		break;

#if 0
    case gecko_evt_gatt_server_user_write_request_id:
    	if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
    		/* Set flag to enter to OTA mode */
    		boot_to_dfu = 1;
    		/* Send response to Write Request */
    		gecko_cmd_gatt_server_send_user_write_response(
    				evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

    		/* Close connection to enter to DFU OTA mode */
    		gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
    	}
    	break;
#endif

    // node reset event
    case gecko_evt_mesh_node_reset_id:
    	gecko_cmd_hardware_set_soft_timer(2 * ONE_SEC_TICKS_CNT, TIMER_ID_FACTORY_RESET, 1);
    	break;

    // server status update received
    case gecko_evt_mesh_generic_client_server_status_id:
    	LOG_INFO("evt_mesh_generic_client_server_status_id");
    	mesh_lib_generic_client_event_handler(evt);
    	break;

    default:
      break;
  }
}


/***************************************************************************//**
 * Persistent Data Flash Load function
 ******************************************************************************/
uint8_t* flashLoad(uint8_t flashID) {
	uint16 resp;
	struct gecko_msg_flash_ps_load_rsp_t* flashResponse;
	// array to store actual data, display length taken because that will be longest

	switch (flashID) {
		// loads people count
		case PEOPLE_COUNT_FLASH_ID:
			flashResponse = gecko_cmd_flash_ps_load(PEOPLE_COUNT_FLASH_ADDRESS);
			flashData[0] = flashResponse->value.data[0];
			break;

		// loads toggleCount
		case ALERT_STATUS_FLASH_ID:
			flashResponse = gecko_cmd_flash_ps_load(ALERT_STATUS_FLASH_ADDRESS);
			flashData[0] = flashResponse->value.data[0];
			break;

		// loads display message
		case DISPLAY_ALERT_FLASH_ID:
			flashResponse = gecko_cmd_flash_ps_load(DISPLAY_ALERT_FLASH_ADDRESS);
			for(int i=0; i<DISPLAY_ALERT_DATA_LENGTH; i++)	{
				flashData[i] = flashResponse->value.data[i];
			}
			break;

		// loads lights status
		case LIGHTS_STATUS_FLASH_ID:
			flashResponse = gecko_cmd_flash_ps_load(LIGHTS_STATUS_FLASH_ADDRESS);
			flashData[0] = flashResponse->value.data[0];
			break;
	}

	resp = flashResponse->result;
	if(resp) {
		LOG_ERROR("flash load failed,code %x", resp);
	} else {
		LOG_INFO("flash load success");
	}

	return flashData;
}

/***************************************************************************//**
 * Persistent Data Flash Store function
 ******************************************************************************/
void flashStore(uint8_t flashID, uint8_t *dataPtr) {
	uint16 resp;

	switch (flashID) {
		// stores people count
		case PEOPLE_COUNT_FLASH_ID:
			resp = gecko_cmd_flash_ps_save(PEOPLE_COUNT_FLASH_ADDRESS, PEOPLE_COUNT_DATA_LENGTH, dataPtr)->result;
			break;

		// stores toggleCount
		case ALERT_STATUS_FLASH_ID:
			resp = gecko_cmd_flash_ps_save(ALERT_STATUS_FLASH_ADDRESS, ALERT_STATUS_DATA_LENGTH, dataPtr)->result;
			break;

		// stores display message
		case DISPLAY_ALERT_FLASH_ID:
			resp = gecko_cmd_flash_ps_save(DISPLAY_ALERT_FLASH_ADDRESS, DISPLAY_ALERT_DATA_LENGTH, dataPtr)->result;
			break;

		// stores lights status
		case LIGHTS_STATUS_FLASH_ID:
			resp = gecko_cmd_flash_ps_save(LIGHTS_STATUS_FLASH_ADDRESS, LIGHTS_STATUS_DATA_LENGTH, dataPtr)->result;
			break;
	}

	if (resp) {
		LOG_ERROR("flash store failed,code %x", resp);
	} else {
		LOG_INFO("flash store success");
	}
}

// string to uint8_t buffer
uint8_t* strToUint(char* str) {
	LOG_INFO("strlen = %d", strlen(str));
	for(int i=0; i<strlen(str); i++){
		uintArray[i] = (uint8_t)str[i];

		// to debug
//		LOG_INFO("%c - %d", str[i], uintArray[i]);
	}
	return uintArray;
}

// uint8_t buffer to string
char* uintToStr(uint8_t* buffer) {
//	LOG_INFO("SIZEOF = %d", sizeof(buffer));
	for(int i=0; i<DISPLAY_ALERT_DATA_LENGTH; i++){
		charArray[i] = (char)buffer[i];

		// to debug
//		LOG_INFO("%c - %d", charArray[i], buffer[i]);
	}
	return charArray;
}

