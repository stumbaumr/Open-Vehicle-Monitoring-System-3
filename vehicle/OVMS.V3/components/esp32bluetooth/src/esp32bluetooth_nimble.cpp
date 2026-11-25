/*
;    Project:       Open Vehicle Monitor System
;    Date:          25th November 2026
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2026       Rainer Stumbaum
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

// NimBLE host for the Bluetooth framework: stack start/stop, advertising,
// connection security (pairing window) and the "bt" bond commands.
//
// The NimBLE host runs in its own task ("ble", core 0). The GAP callback
// below is called in that task.

#include "sdkconfig.h"
#ifdef CONFIG_NIMBLE_ENABLED

#include <string.h>
#include "esp32bluetooth.h"
#include "esp32bluetooth_app_elm327.h"
#include "ovms_command.h"
#include "ovms_config.h"
#include "ovms_events.h"
#include "ovms_peripherals.h"
#include "ovms_log.h"

// NimBLE headers last (they define min() / max() macros):
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#undef min
#undef max

static const char *TAG = "bt";

// Not declared in a NimBLE header:
extern "C" void ble_store_config_init(void);

#define BT_MAX_BONDS                CONFIG_NIMBLE_MAX_BONDS
#define BT_ADV_INTERVAL             0x100     // 160 ms

static uint8_t ovms_manufacturer[4] = { 'O', 'V', 'M', 'S' };

// Current connection (max. 1):
static volatile uint16_t bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
// Bonded peers at connection start, to detect a new bond:
static ble_addr_t bt_conn_known[BT_MAX_BONDS];
static int bt_conn_known_count = 0;

static int bt_gap_event(struct ble_gap_event *event, void *arg);

static void bt_format_addr(char* buf, const ble_addr_t* addr)
  {
  // NimBLE stores the address LSB first
  sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
    addr->val[5], addr->val[4], addr->val[3],
    addr->val[2], addr->val[1], addr->val[0]);
  }

static void bt_advertise()
  {
  if (bt_conn_handle != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active())
    return;

  // Advertising data: flags, TX power, ELM327 service UUID 0xFFF0
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof(fields));
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  ble_uuid16_t svc_uuid;
  svc_uuid.u.type = BLE_UUID_TYPE_16;
  svc_uuid.value = GATTS_SERVICE_UUID_ELM327;
  fields.uuids16 = &svc_uuid;
  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;
  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0)
    {
    ESP_LOGE(TAG, "Setting advertising data failed, rc=%d", rc);
    return;
    }

  // Scan response: device name, manufacturer data "OVMS"
  struct ble_hs_adv_fields rsp;
  memset(&rsp, 0, sizeof(rsp));
  const char* name = ble_svc_gap_device_name();
  rsp.name = (uint8_t*)name;
  rsp.name_len = strlen(name);
  rsp.name_is_complete = 1;
  rsp.mfg_data = ovms_manufacturer;
  rsp.mfg_data_len = sizeof(ovms_manufacturer);
  rc = ble_gap_adv_rsp_set_fields(&rsp);
  if (rc != 0)
    {
    ESP_LOGE(TAG, "Setting scan response data failed, rc=%d (device name too long?)", rc);
    return;
    }

  struct ble_gap_adv_params params;
  memset(&params, 0, sizeof(params));
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  params.itvl_min = BT_ADV_INTERVAL;
  params.itvl_max = BT_ADV_INTERVAL;
  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &params, bt_gap_event, NULL);
  if (rc != 0)
    ESP_LOGE(TAG, "Starting advertising failed, rc=%d", rc);
  else
    ESP_LOGI(TAG, "Advertising as '%s'", name);
  }

static bool bt_conn_was_bonded(const ble_addr_t* addr)
  {
  for (int i = 0; i < bt_conn_known_count; i++)
    {
    if (ble_addr_cmp(&bt_conn_known[i], addr) == 0)
      return true;
    }
  return false;
  }

// Encryption is enabled: check if the peer may use the services.
// A bonded peer (bond existed at connection start) is accepted. A new pairing
// is only accepted while the pairing window is open. NimBLE has no hook to
// reject a pairing request, so a new bond outside of the window is deleted
// and the connection is closed.
static void bt_check_access(uint16_t conn_handle)
  {
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(conn_handle, &desc) != 0)
    return;
  char addr[18];
  bt_format_addr(addr, &desc.peer_id_addr);

  if (bt_conn_was_bonded(&desc.peer_id_addr))
    {
    ESP_LOGI(TAG, "Bonded device %s connected", addr);
    MyBluetoothAppELM327.NimbleAuthorized();
    }
  else if (MyBluetoothAppELM327.PairingWindowRemaining() > 0)
    {
    ESP_LOGI(TAG, "New device %s paired (%s)", addr, desc.sec_state.bonded ? "bonded" : "not bonded");
    MyBluetoothAppELM327.NimbleAuthorized();
    std::string signal("bt.auth.");
    signal.append(addr);
    MyEvents.SignalEvent(signal, NULL);
    }
  else
    {
    ESP_LOGW(TAG, "Pairing rejected for %s (pairing window closed, see 'bt elm327 pair')", addr);
    if (desc.sec_state.bonded)
      ble_store_util_delete_peer(&desc.peer_id_addr);
    ble_gap_terminate(conn_handle, BLE_ERR_AUTH_FAIL);
    }
  }

static int bt_gap_event(struct ble_gap_event *event, void *arg)
  {
  struct ble_gap_conn_desc desc;
  char addr[18];

  switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status != 0)
        {
        ESP_LOGW(TAG, "Connection failed, status=%d", event->connect.status);
        bt_advertise();
        return 0;
        }
      bt_conn_handle = event->connect.conn_handle;
      bt_conn_known_count = 0;
      ble_store_util_bonded_peers(bt_conn_known, &bt_conn_known_count, BT_MAX_BONDS);
      if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0)
        {
        bt_format_addr(addr, &desc.peer_id_addr);
        ESP_LOGI(TAG, "Connection from %s", addr);
        }
      MyBluetoothAppELM327.NimbleConnect(event->connect.conn_handle);
      // Request encryption: a bonded peer encrypts with the stored key,
      // a new peer starts the pairing ("Just Works")
      ble_gap_security_initiate(event->connect.conn_handle);
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      bt_format_addr(addr, &event->disconnect.conn.peer_id_addr);
      ESP_LOGI(TAG, "Disconnected from %s, reason=0x%x", addr, event->disconnect.reason);
      bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
      MyBluetoothAppELM327.NimbleDisconnect();
      bt_advertise();
      return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      bt_advertise();
      return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
      if (event->enc_change.status != 0)
        {
        // For example: the peer has lost its bond, use "bt clear" and pair again
        ESP_LOGW(TAG, "Encryption failed, status=%d", event->enc_change.status);
        return 0;
        }
      bt_check_access(event->enc_change.conn_handle);
      return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
      // A bonded peer wants to pair again (it has deleted its bond):
      // only allowed while the pairing window is open
      if (MyBluetoothAppELM327.PairingWindowRemaining() > 0 &&
          ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0)
        {
        ble_store_util_delete_peer(&desc.peer_id_addr);
        bt_conn_known_count = 0;
        return BLE_GAP_REPEAT_PAIRING_RETRY;
        }
      ESP_LOGW(TAG, "Repeated pairing rejected (pairing window closed, see 'bt elm327 pair')");
      return BLE_GAP_REPEAT_PAIRING_IGNORE;

    case BLE_GAP_EVENT_SUBSCRIBE:
      MyBluetoothAppELM327.NimbleSubscribe(event->subscribe.attr_handle, event->subscribe.cur_notify);
      return 0;

    case BLE_GAP_EVENT_MTU:
      ESP_LOGI(TAG, "MTU %d", event->mtu.value);
      MyBluetoothAppELM327.NimbleMtu(event->mtu.value);
      return 0;

    default:
      return 0;
    }
  }

// Bond store full: only delete the oldest bond while the pairing window is
// open, so a rejected device cannot remove a valid bond.
static int bt_store_status(struct ble_store_status_event *event, void *arg)
  {
  if (event->event_code == BLE_STORE_EVENT_OVERFLOW &&
      MyBluetoothAppELM327.PairingWindowRemaining() == 0)
    {
    ESP_LOGW(TAG, "Bond store full, new bond rejected");
    return BLE_HS_ESTORE_CAP;
    }
  return ble_store_util_status_rr(event, arg);
  }

static void bt_on_sync(void)
  {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0)
    {
    ESP_LOGE(TAG, "No Bluetooth address, rc=%d", rc);
    return;
    }
  bt_advertise();
  }

static void bt_on_reset(int reason)
  {
  ESP_LOGE(TAG, "NimBLE host reset, reason=%d", reason);
  }

static void bt_host_task(void *param)
  {
  // Returns after nimble_port_stop()
  nimble_port_run();
  nimble_port_freertos_deinit();
  }

bool esp32bluetooth::StartStack()
  {
  esp_err_t ret = esp_nimble_hci_and_controller_init();
  if (ret)
    {
    ESP_LOGE(TAG, "init controller failed: %s", esp_err_to_name(ret));
    return false;
    }

  nimble_port_init();

  ble_hs_cfg.reset_cb = bt_on_reset;
  ble_hs_cfg.sync_cb = bt_on_sync;
  ble_hs_cfg.store_status_cb = bt_store_status;

  // ELM327 clients have no passkey input: "Just Works" bonding, access is
  // controlled by the pairing window (see "bt elm327 pair")
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  if (!MyBluetoothAppELM327.NimbleRegister())
    {
    nimble_port_deinit();
    esp_nimble_hci_and_controller_deinit();
    return false;
    }

  std::string name = MyConfig.GetParamValue("bluetooth", "elm327.name", "OBDII");
  if (ble_svc_gap_device_name_set(name.c_str()) != 0)
    ESP_LOGE(TAG, "Invalid device name '%s'", name.c_str());

  ble_store_config_init();

  bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
  nimble_port_freertos_init(bt_host_task);
  return true;
  }

bool esp32bluetooth::StopStack()
  {
  int rc = nimble_port_stop();
  if (rc != 0)
    {
    ESP_LOGE(TAG, "NimBLE host stop failed, rc=%d", rc);
    return false;
    }
  nimble_port_deinit();

  esp_err_t ret = esp_nimble_hci_and_controller_deinit();
  if (ret)
    ESP_LOGE(TAG, "deinit controller failed: %s", esp_err_to_name(ret));

  bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
  MyBluetoothAppELM327.NimbleDisconnect();
  return true;
  }

void bluetooth_status(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyPeripherals->m_esp32bluetooth->IsServiceRunning())
    {
    writer->puts("Bluetooth service is not running");
    return;
    }

  ble_addr_t peers[BT_MAX_BONDS];
  int count = 0;
  ble_store_util_bonded_peers(peers, &count, BT_MAX_BONDS);
  writer->printf("Bluetooth service is running (NimBLE, %d client(s) registered)\n", count);
  for (int k = 0; k < count; k++)
    {
    char addr[18];
    bt_format_addr(addr, &peers[k]);
    writer->printf("  %s\n", addr);
    }
  }

void bluetooth_clear(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyPeripherals->m_esp32bluetooth->IsServiceRunning())
    {
    writer->puts("Error: Bluetooth service is not running");
    return;
    }

  ble_addr_t peers[BT_MAX_BONDS];
  int count = 0;
  ble_store_util_bonded_peers(peers, &count, BT_MAX_BONDS);
  for (int k = 0; k < count; k++)
    {
    char addr[18];
    bt_format_addr(addr, &peers[k]);
    if ((argc == 0) || (strcasecmp(addr, argv[0]) == 0))
      {
      writer->printf("Removing Registration: %s\n", addr);
      // Closes an open connection to the peer and deletes the bond
      ble_gap_unpair(&peers[k]);
      }
    }
  }

#endif // CONFIG_NIMBLE_ENABLED
