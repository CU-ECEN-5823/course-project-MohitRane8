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
/* People count using IR Sensor */
int16_t peopleCount = 0;
uint8_t ir1Value = 0;
uint8_t ir2Value = 0;

/* Sensors activation flag - to remove false notifications */
uint8_t ir1ActivationFlag = 1;
uint8_t ir2ActivationFlag = 1;
uint8_t vibActivationFlag = 1;

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

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

#if DEVICE_IS_ONOFF_PUBLISHER
  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "5823PUB %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  DISPLAY_PRINTF(DISPLAY_ROW_NAME, "%s", name);
#else
  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "5823SUB %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  DISPLAY_PRINTF(DISPLAY_ROW_NAME, "%s", name);
#endif

  DISPLAY_PRINTF(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x", pAddr-> addr[0], pAddr-> addr[1], pAddr-> addr[2], pAddr-> addr[3], pAddr-> addr[4], pAddr-> addr[5]);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
}

/***************************************************************************//**
 * Initialization of the models supported by this node.
 * This function registers callbacks for each of the supported models.
 ******************************************************************************/
//static void init_models(void)
//{
//  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
//                                           0,
//                                           onoff_request,
//                                           onoff_change);
//}

//static void init_models(void)
//{
//  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
//                                           0,
//                                           pressrelease_request,
//                                           pressrelease_change);
//}

static void init_models(void)
{
  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                           0,
                                           pb0_pressrelease_request,
                                           pb0_pressrelease_change);

  mesh_lib_generic_server_register_handler(MESH_LIGHTING_LIGHTNESS_SERVER_MODEL_ID,
                                           0,
                                           pb1_pressrelease_request,
                                           pb1_pressrelease_change);
}

/***************************************************************************//**
 * Update generic on/off state.
 *
 * @param[in] element_index  Server model element index.
 *
 * @return Status of the update operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
//static errorcode_t onoff_update(uint16_t element_index)
//{
//  struct mesh_generic_state current, target;
//
//  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
//                                        element_index,
//                                        &current,
//                                        &target,
//                                        0);
//}

/***************************************************************************//**
 * Update generic on/off state and publish model state to the network.
 *
 * @param[in] element_index  Server model element index.
 *
 * @return Status of the update and publish operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
//static errorcode_t onoff_update_and_publish(uint16_t element_index)
//{
//  errorcode_t e;
//
//  e = onoff_update(element_index);
//  if (e == bg_err_success) {
//    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
//                                        element_index,
//                                        mesh_generic_state_on_off);
//  }
//
//  return e;
//}

/***************************************************************************//**
 * This function is a handler for generic on/off change event.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] current        Pointer to current state structure.
 * @param[in] target         Pointer to target state structure.
 * @param[in] remaining_ms   Time (in milliseconds) remaining before transition
 *                           from current state to target state is complete.
 ******************************************************************************/
//static void onoff_change(uint16_t model_id,
//                         uint16_t element_index,
//                         const struct mesh_generic_state *current,
//                         const struct mesh_generic_state *target,
//                         uint32_t remaining_ms)
//{
//	LOG_INFO("State Changed");
//}

static void pb0_pressrelease_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
	LOG_INFO("PB0 State Changed");
}

static void pb1_pressrelease_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
	LOG_INFO("PB1 State Changed");
}

//// this works for now because this function is not used anyways, it may not work if it is actually used
//static void onoff_change()
//{
//	LOG_INFO("State Changed");
//}

/***************************************************************************//**
 * This function process the requests for the generic on/off model.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] client_addr    Address of the client model which sent the message.
 * @param[in] server_addr    Address the message was sent to.
 * @param[in] appkey_index   The application key index used in encrypting the request.
 * @param[in] request        Pointer to the request structure.
 * @param[in] transition_ms  Requested transition time (in milliseconds).
 * @param[in] delay_ms       Delay time (in milliseconds).
 * @param[in] request_flags  Message flags. Bitmask of the following:
 *                           - Bit 0: Nonrelayed. If nonzero indicates
 *                                    a response to a nonrelayed request.
 *                           - Bit 1: Response required. If nonzero client
 *                                    expects a response from the server.
 ******************************************************************************/
//static void onoff_request(uint16_t model_id,
//                          uint16_t element_index,
//                          uint16_t client_addr,
//                          uint16_t server_addr,
//                          uint16_t appkey_index,
//                          const struct mesh_generic_request *request,
//                          uint32_t transition_ms,
//                          uint16_t delay_ms,
//                          uint8_t request_flags)
//{
//	if(request->on_off == MESH_GENERIC_ON_OFF_STATE_OFF)
//		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "Button Released");
//	else if(request->on_off == MESH_GENERIC_ON_OFF_STATE_ON)
//		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "Button Pressed");
//
////	onoff_update_and_publish(element_index);
//}

// all parameters in request function are required even if they are not used or it won't work
static void pb0_pressrelease_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	if(request->on_off == MESH_GENERIC_ON_OFF_STATE_OFF)
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PB0 Released");
	else if(request->on_off == MESH_GENERIC_ON_OFF_STATE_ON)
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PB0 Pressed");
}

static void pb1_pressrelease_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	LOG_INFO("PB1 request");
	if(request->lightness == 0x02)
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PB1 Released");
	else if(request->lightness == MESH_GENERIC_ON_OFF_STATE_ON)
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PB1 Pressed");
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
		LOG_INFO("LPN init failed (0x%x)", res);
		return;
	}

	// Configure the lpn with following parameters:
	// - Minimum friend queue length = 2
	// - Poll timeout = 1 seconds
	res = gecko_cmd_mesh_lpn_configure(2, 1 * 1000)->result;
	if (res) {
		LOG_INFO("LPN conf failed (0x%x)", res);
		return;
	}

	LOG_INFO("trying to find friend...");
	res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (res != 0) {
		LOG_INFO("ret.code %x", res);
	}
}


/**
 * See light switch app.c file definition
 */
void gecko_bgapi_classes_init_server_friend(void)
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
	//gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
//	gecko_bgapi_class_mesh_friend_init();
}


/**
 * See main function list in soc-btmesh-switch project file
 */
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

  if( DeviceUsesClientModel() ) {
	  gecko_bgapi_classes_init_client_lpn();
  } else {
	  gecko_bgapi_classes_init_server_friend();
  }

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

}

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  switch (evt_id) {
    case gecko_evt_system_boot_id:
    	// check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
		if (GPIO_PinInGet(PB0_PORT, PB0_PIN) == 0 || GPIO_PinInGet(PB1_PORT, PB1_PIN) == 0) {
			LOG_INFO("factory reset");
			DISPLAY_PRINTF(DISPLAY_ROW_ACTION, ">>>FACTORY RESET<<<");

			/* perform a factory reset by erasing PS storage. This removes all the keys and other settings
			that have been configured for this node */
			gecko_cmd_flash_ps_erase_all();
			// reboot after a small delay
			gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
		} else {

			struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
			set_device_name(&pAddr->address);

			// Initialize Mesh stack in Node operation mode, wait for initialized event
			gecko_cmd_mesh_node_init();
			LOG_INFO("BOOT DONE");
		}
      break;

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
			case DISPLAY_UPDATE:
				// Prevent charge buildup on LCD
				displayUpdate();
				break;
			case LOG_TIME_UPDATE:
				// Increase millisecond count by 10
				msecCount += 10;
				break;

			case IR1_TIMEOUT_FLAG:
				ir1ActivationFlag = 1;
				break;

			case IR2_TIMEOUT_FLAG:
				ir2ActivationFlag = 1;
				break;

			case VIB_TIMEOUT_FLAG:
				vibActivationFlag = 1;
				break;

			case VIBRATION_ALERT:
				;
				static uint8_t toggleCnt = 0;

				if(toggleCnt % 2) {
					toggleCnt++;
					GPIO_PinOutSet(BUZZER_PORT, BUZZER_PIN);
					gpioLed1SetOn();
				}
				else {
					toggleCnt++;
					GPIO_PinOutClear(BUZZER_PORT, BUZZER_PIN);
					gpioLed1SetOff();

					// Stop timer after 100 toggles or 10 seconds
					if(toggleCnt > 100) {
						toggleCnt = 0;
						gecko_cmd_hardware_set_soft_timer(0, VIBRATION_ALERT, 0);
						DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, " ");
					}
				}
				break;

			case TIMER_ID_FRIEND_FIND:
			{
				LOG_INFO("trying to find friend...");
				uint16_t result;
				result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;
				if (result != 0) {
					LOG_INFO("ret.code %x", result);
				}
			}
			break;
    	}
    	break;

#if DEVICE_IS_ONOFF_PUBLISHER
	case gecko_evt_system_external_signal_id:
		;

		struct mesh_generic_request req;

		//publish response
		uint16 resp;

		//transaction identifier
		static uint8 trid = 0;

		// PB0 button press
		if (((evt->data.evt_system_external_signal.extsignals) & PB0_FLAG) != 0) {
			req.kind = mesh_generic_state_on_off;

			if(GPIO_PinInGet(PB0_PORT, PB0_PIN) == 0) {
				LOG_INFO("PB0 Pressed");
				req.on_off = MESH_GENERIC_ON_OFF_STATE_ON;
			}
			else {
				LOG_INFO("PB0 Released");
				req.on_off = MESH_GENERIC_ON_OFF_STATE_OFF;
			}

			trid++;

			resp = mesh_lib_generic_client_publish(MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID, _elem_index, trid, &req, 0, 0, 0);

			if (resp) {
				LOG_INFO("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
			} else {
				LOG_INFO("request sent, trid = %u", trid);
			}
		}

		// PB1 button press
		if(((evt->data.evt_system_external_signal.extsignals) & PB1_FLAG) != 0) {
			req.kind = mesh_lighting_request_lightness_actual;

			if(GPIO_PinInGet(PB1_PORT, PB1_PIN) == 0) {
				LOG_INFO("PB1 Pressed");
				req.lightness = MESH_GENERIC_ON_OFF_STATE_ON;
			}
			else {
				LOG_INFO("PB1 Released");
				req.lightness = 0x02;
			}

			trid++;

			resp = mesh_lib_generic_client_publish(MESH_LIGHTING_LIGHTNESS_CLIENT_MODEL_ID, _elem_index, trid, &req, 0, 0, 0);

			if (resp) {
				LOG_INFO("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
			} else {
				LOG_INFO("request sent, trid = %u", trid);
			}
		}

		// Infrared Sensor 1 external event
		if(((evt->data.evt_system_external_signal.extsignals) & IR1_FLAG) != 0) {
			if(ir1ActivationFlag)
			{
				ir1ActivationFlag = 0;
				LOG_INFO("IR1 FLAG");
				gecko_cmd_hardware_set_soft_timer(1 * 32768, IR1_TIMEOUT_FLAG, 1);

				if(ir2Value) {
					ir2Value = 0;

					if(peopleCount)
						peopleCount--;

					if(peopleCount == 0)
						gpioLed0SetOff();

					DISPLAY_PRINTF(DISPLAY_ROW_PEOPLE, "People Inside: %d", peopleCount);
					LOG_INFO("People Inside: %d", peopleCount);
				}
				else
					ir1Value = 1;
			}
		}

		// Infrared Sensor 2 external event
		if(((evt->data.evt_system_external_signal.extsignals) & IR2_FLAG) != 0) {
			if(ir2ActivationFlag)
			{
				ir2ActivationFlag = 0;
				LOG_INFO("IR2 FLAG");
				gecko_cmd_hardware_set_soft_timer(1 * 32768, IR2_TIMEOUT_FLAG, 1);

				if(ir1Value) {
					ir1Value = 0;
					peopleCount++;
					gpioLed0SetOn();
					DISPLAY_PRINTF(DISPLAY_ROW_PEOPLE, "People Inside: %d", peopleCount);
					LOG_INFO("People Inside: %d", peopleCount);
				}
				else
					ir2Value = 1;
			}
		}

		// Vibration Sensor external event
		if(((evt->data.evt_system_external_signal.extsignals) & VIB_FLAG) != 0) {
			if(vibActivationFlag) {
				vibActivationFlag = 0;

				DISPLAY_PRINTF(DISPLAY_ROW_SENSOR, "EARTHQUAKE");

				// Timeout of 1 sec
				gecko_cmd_hardware_set_soft_timer(1 * 32768, VIB_TIMEOUT_FLAG, 1);

				// Set alarm in case of earthquake
				// Alert frequency 200 ms
				gecko_cmd_hardware_set_soft_timer(3277, VIBRATION_ALERT, 0);

				LOG_INFO("VIB FLAG");
			}
		}

		break;
#endif

    case gecko_evt_mesh_node_initialized_id:
		LOG_INFO("node initialized");

		struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

		if (pData->provisioned) {
			DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISIONED");
			LOG_INFO("node is provisioned. address:%x, ivi:%ld", pData->address, pData->ivi);

			#if DEVICE_IS_ONOFF_PUBLISHER
						mesh_lib_init(malloc,free,8);
			#else
						mesh_lib_init(malloc,free,9);
						init_models();
			#endif

			#if DEVICE_USES_BLE_MESH_CLIENT_MODEL
						gecko_cmd_mesh_generic_client_init();
						// Initialize Low Power Node functionality
//						lpn_init();

						/* trying friend role reversal */
						uint16 res;
						res = gecko_cmd_mesh_friend_init()->result;
						if (res) {
							LOG_INFO("Friend init failed 0x%x", res);
						}

						gpioIntEnable();
			#else
						gecko_cmd_mesh_generic_server_init();
						//Initialize Friend functionality
						LOG_INFO("Friend mode initialization");
//						uint16 res;
//						res = gecko_cmd_mesh_friend_init()->result;
//						if (res) {
//							LOG_INFO("Friend init failed 0x%x", res);
//						}

						/* trying lpn role reversal */
						lpn_init();
			#endif
		}
		else {
			DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "UNPROVISIONED");
			LOG_INFO("node is unprovisioned");
			LOG_INFO("starting unprovisioned beaconing...");
			// The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
			gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
		}
		break;

    case gecko_evt_mesh_node_provisioning_started_id:
    	LOG_INFO("Started provisioning");
    	DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISIONING");
		break;

	case gecko_evt_mesh_node_provisioned_id:
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISIONED");
		LOG_INFO("node is provisioned. address:%x, ivi:%ld", pData->address, pData->ivi);

		#if DEVICE_IS_ONOFF_PUBLISHER
					mesh_lib_init(malloc,free,8);
		#else
					mesh_lib_init(malloc,free,9);
					init_models();
		#endif

		#if DEVICE_USES_BLE_MESH_CLIENT_MODEL
					gecko_cmd_mesh_generic_client_init();
					gpioIntEnable();
		#else
					gecko_cmd_mesh_generic_server_init();
					//Initialize Friend functionality
					LOG_INFO("Friend mode initialization");
					uint16 res;
					res = gecko_cmd_mesh_friend_init()->result;
					if (res) {
						LOG_INFO("Friend init failed 0x%x", res);
					}
		#endif
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		LOG_INFO("provisioning failed, code %x", evt->data.evt_mesh_node_provisioning_failed.result);
		DISPLAY_PRINTF(DISPLAY_ROW_ACTION, "PROVISION FAILED");
		/* start a one-shot timer that will trigger soft reset after small delay */
		gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
		break;

	case gecko_evt_le_connection_opened_id:
		LOG_INFO("evt:gecko_evt_le_connection_opened_id");
		num_connections++;
		DISPLAY_PRINTF(DISPLAY_ROW_CONNECTION, "Connected");

		#if DEVICE_IS_ONOFF_PUBLISHER
		// turn off lpn feature after GATT connection is opened
		gecko_cmd_mesh_lpn_deinit();
		DISPLAY_PRINTF(DISPLAY_ROW_LPN, "LPN off");
		#endif
		break;

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
				#if DEVICE_IS_ONOFF_PUBLISHER
				lpn_init();
				#endif
			}
		}
		break;

    case gecko_evt_mesh_lpn_friendship_established_id:
    	LOG_INFO("friendship established");
    	DISPLAY_PRINTF(DISPLAY_ROW_LPN, "LPN");
    	break;

    case gecko_evt_mesh_lpn_friendship_failed_id:
    	LOG_INFO("friendship failed");
    	DISPLAY_PRINTF(DISPLAY_ROW_LPN, "no friend");
    	// try again in 2 seconds
    	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FRIEND_FIND, 1);
    	break;

    case gecko_evt_mesh_lpn_friendship_terminated_id:
    	LOG_INFO("friendship terminated");
    	DISPLAY_PRINTF(DISPLAY_ROW_LPN, "friend lost");
    	if (num_connections == 0) {
    		// try again in 2 seconds
    		gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FRIEND_FIND, 1);
    	}
    	break;

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

    case gecko_evt_mesh_node_reset_id:
    	gecko_cmd_flash_ps_erase_all();
    	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
    	break;

#if DEVICE_USES_BLE_MESH_SERVER_MODEL
    case gecko_evt_mesh_generic_server_client_request_id:
    	LOG_INFO("evt_mesh_generic_server_client_request_id");
    	mesh_lib_generic_server_event_handler(evt);
    	break;

    case gecko_evt_mesh_generic_server_state_changed_id:
    	LOG_INFO("evt_mesh_generic_server_state_changed_id");
    	mesh_lib_generic_server_event_handler(evt);
    	break;
#endif

    default:
      break;
  }
}
