/***************************************************************************//**
 * @file
 * @brief BT Mesh Device Composition Data
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

/*****************************************************************************
 *  Autogenerated file, do not edit
 ****************************************************************************/
 
#include <stddef.h>
#include <stdint.h>
#include "mesh_memory_config.h"
#include "mesh_sizes.h"
#include "mesh_app_memory_config.h"
const uint8_t __mesh_dcd[] = {
    0xff, 0x02, /* Company ID = 0x02ff */
    0xb0, 0xf0, /* Product ID = 0xf0b0 */
    0x34, 0x12, /* Version Number = 0x1234 */
    0x20, 0x00, /* Capacity of Replay Protection List = 0x0020 */
    0x0f, 0x00, /* Features Bitmask = 0x000f */
    /* Begin Primary Element */
        0x00, 0x00, /* Location = 0x0000 */
        0x04, /* Number of SIG Models = 0x04 */
        0x00, /* Number of Vendor Models = 0x00 */
        /* Begin SIG Models */
        0x00, 0x00, /* Configuration Server */
        0x00, 0x11, /* Sensor Server */
        0x02, 0x11, /* Sensor Client */
        0x01, 0x00, /* Configuration Client */
        /* End SIG Models */
        /* Begin Vendor Models */
        /* End Vendor Models */
    /* End Primary Element */
};

const size_t __mesh_dcd_len = sizeof(__mesh_dcd);
const uint8_t *__mesh_dcd_ptr = __mesh_dcd;

const mesh_memory_config_t __mesh_memory_config = {
  .max_elements = MESH_CFG_MAX_ELEMENTS,
  .max_models = MESH_CFG_MAX_MODELS,
  .max_net_keys = MESH_CFG_MAX_NETKEYS,
  .max_appkeys = MESH_CFG_MAX_APPKEYS,
  .max_devkeys = MESH_CFG_MAX_DEVKEYS,
  .max_friendships = MESH_CFG_MAX_FRIENDSHIPS,
  .max_app_binds = MESH_CFG_MAX_APP_BINDS,
  .max_subscriptions = MESH_CFG_MAX_SUBSCRIPTIONS,
  .max_foundation_model_commands = MESH_CFG_MAX_FOUNDATION_CLIENT_CMDS,
  .net_cache_size = MESH_CFG_NET_CACHE_SIZE,
  .replay_size = MESH_CFG_RPL_SIZE,
  .max_send_segs = MESH_CFG_MAX_SEND_SEGS,
  .max_recv_segs = MESH_CFG_MAX_RECV_SEGS,
  .max_virtual_addresses = MESH_CFG_MAX_VAS,
  .max_provision_sessions = MESH_CFG_MAX_PROV_SESSIONS,
  .max_provision_bearers = MESH_CFG_MAX_PROV_BEARERS,
  .max_gatt_connections = MESH_CFG_MAX_GATT_CONNECTIONS,
  .gatt_txqueue_size = MESH_CFG_GATT_TXQ_SIZE,
  .provisioner_max_ddb_entries = MESH_CFG_MAX_PROVISIONED_DEVICES,
  .provisioner_max_node_net_keys = MESH_CFG_MAX_PROVISIONED_DEVICE_NETKEYS,
  .provisioner_max_node_app_keys = MESH_CFG_MAX_PROVISIONED_DEVICE_APPKEYS,
  .pstore_write_interval_elem_seq = 65536,
  .friend_max_total_cache = MESH_CFG_FRIEND_MAX_TOTAL_CACHE,
  .friend_max_single_cache = MESH_CFG_FRIEND_MAX_SINGLE_CACHE,
  .friend_max_subs_list = MESH_CFG_FRIEND_MAX_SUBS_LIST
};