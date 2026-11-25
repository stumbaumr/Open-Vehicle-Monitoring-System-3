/*
;    Project:       Open Vehicle Monitor System
;    Date:          24th November 2026
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

#include <string.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "esp32bluetooth_app_elm327.h"
#include "ovms.h"
#include "ovms_command.h"
#include "ovms_config.h"
#include "metrics_standard.h"
#include "ovms_peripherals.h"
#include "pcp.h"
#include "can.h"
#ifdef CONFIG_OVMS_COMP_POLLER
#include "vehicle.h"
#include "vehicle_poller.h"
#endif

#include "ovms_log.h"
static const char *TAG = "bt-app-elm327";

#define ELM_MSG_RESET               1
#define ELM_MSG_DATA                2

#define ELM_QUEUE_SIZE              20
#define ELM_TASK_STACK              3584    // measured max. 2076 bytes (NimBLE, ABRP)
#define ELM_TASK_PRIORITY           5
#define ELM_PAIRING_DEFAULT_SEC     120

#ifdef CONFIG_BLUEDROID_ENABLED
// All attributes require an encrypted (bonded) connection:
#define ELM_PERM_READ               ESP_GATT_PERM_READ_ENCRYPTED
#define ELM_PERM_WRITE              ESP_GATT_PERM_WRITE_ENCRYPTED
#endif

OvmsBluetoothAppELM327 MyBluetoothAppELM327 __attribute__ ((init_priority (8018)));

////////////////////////////////////////////////////////////////////////
// Commands

static void elm327_status(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  MyBluetoothAppELM327.Status(writer);
  }

static void elm327_pair(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  int seconds = ELM_PAIRING_DEFAULT_SEC;
  if (argc > 0)
    {
    seconds = atoi(argv[0]);
    if (seconds < 0 || seconds > 3600)
      {
      writer->puts("Error: seconds must be in the range 0…3600");
      return;
      }
    }
  MyBluetoothAppELM327.OpenPairingWindow(seconds);
  if (seconds == 0)
    {
    writer->puts("Pairing window closed");
    return;
    }
  writer->printf("Pairing window open for %d seconds\n", seconds);
  if (!MyPeripherals->m_esp32bluetooth->IsServiceRunning())
    writer->puts("Note: bluetooth is not running (use 'power bluetooth on')");
  }

////////////////////////////////////////////////////////////////////////
// OvmsBluetoothAppELM327

OvmsBluetoothAppELM327::OvmsBluetoothAppELM327()
  :
#ifdef CONFIG_BLUEDROID_ENABLED
    esp32bluetoothApp("elm327"),
#endif
    m_engine(this)
  {
  ESP_LOGI(TAG, "Initialising Bluetooth ELM327 App (8018)");

  m_queue = NULL;
  m_task = NULL;
  m_notify_handle = 0;
  m_notify_descr_handle = 0;
  m_write_handle = 0;
  m_connected = false;
  m_notifying = false;
  m_congested = false;
  m_pairing_until_us = 0;
  m_stat_rx_bytes = 0;
  m_stat_tx_bytes = 0;
  m_stat_dropped = 0;

#ifdef CONFIG_BLUEDROID_ENABLED
  m_app_id = GATTS_APP_UUID_OVMS_ELM327;
  MyBluetoothGATTS.RegisterApp(this);
#endif
#ifdef CONFIG_NIMBLE_ENABLED
  m_conn_handle = 0xffff;   // BLE_HS_CONN_HANDLE_NONE
  m_mtu = 0;
  m_authorized = false;
#endif

  OvmsCommand* cmd_bt = MyCommandApp.FindCommand("bt");
  if (cmd_bt)
    {
    OvmsCommand* cmd_elm = cmd_bt->RegisterCommand("elm327", "ELM327 adapter emulation", elm327_status);
    cmd_elm->RegisterCommand("status", "Show ELM327 emulation status", elm327_status);
    cmd_elm->RegisterCommand("pair", "Open the pairing window for new devices (0 = close)",
      elm327_pair, "[<seconds>]", 0, 1);
    }
  }

OvmsBluetoothAppELM327::~OvmsBluetoothAppELM327()
  {
  }

#ifdef CONFIG_BLUEDROID_ENABLED

bool OvmsBluetoothAppELM327::GetConnectSecurity(esp_ble_sec_act_t* action)
  {
  // Bonded clients encrypt with the stored key, new clients start pairing
  *action = ESP_BLE_SEC_ENCRYPT_NO_MITM;
  return true;
  }

bool OvmsBluetoothAppELM327::AcceptSecurityRequest(esp_bd_addr_t bda)
  {
  if (PairingWindowRemaining() > 0)
    return true;
  if (IsBonded(bda))
    return true;
  ESP_LOGW(TAG, "Pairing rejected for %02x:%02x:%02x:%02x:%02x:%02x (pairing window closed, see 'bt elm327 pair')",
    bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return false;
  }

bool OvmsBluetoothAppELM327::IsBonded(esp_bd_addr_t bda)
  {
  int count = esp_ble_get_bond_device_num();
  if (count <= 0)
    return false;
  esp_ble_bond_dev_t *list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * count);
  if (!list)
    return false;
  bool found = false;
  esp_ble_get_bond_device_list(&count, list);
  for (int i = 0; i < count && !found; i++)
    {
    found = (memcmp(list[i].bd_addr, bda, sizeof(esp_bd_addr_t)) == 0);
    }
  free(list);
  return found;
  }

int OvmsBluetoothAppELM327::BondCount()
  {
  return esp_ble_get_bond_device_num();
  }

#endif // CONFIG_BLUEDROID_ENABLED

void OvmsBluetoothAppELM327::OpenPairingWindow(int seconds)
  {
  if (seconds <= 0)
    m_pairing_until_us = 0;
  else
    m_pairing_until_us = esp_timer_get_time() + (int64_t)seconds * 1000000;
  }

int OvmsBluetoothAppELM327::PairingWindowRemaining()
  {
  int64_t remain = m_pairing_until_us - esp_timer_get_time();
  return (remain > 0) ? (int)(remain / 1000000) + 1 : 0;
  }

void OvmsBluetoothAppELM327::LoadConfig()
  {
  std::string protocol = MyConfig.GetParamValue("bluetooth", "elm327.protocol", "6");
  m_engine.SetDefaultProtocol(protocol.empty() ? '6' : protocol[0]);
  m_engine.SetMinTimeout(MyConfig.GetParamValueInt("bluetooth", "elm327.timeout", 500));
  std::string services = MyConfig.GetParamValue("bluetooth", "elm327.services", "01,02,03,06,07,09,0A,19,1A,21,22,3E");
  if (!m_engine.SetAllowedServices(services))
    ESP_LOGE(TAG, "Invalid [bluetooth] elm327.services '%s', using previous setting", services.c_str());
  }

void OvmsBluetoothAppELM327::StartTask()
  {
  if (m_queue == NULL)
    {
    m_queue = xQueueCreate(ELM_QUEUE_SIZE, sizeof(elm_msg_t));
    }
  if (m_task == NULL)
    {
    xTaskCreatePinnedToCore(TaskEntry, "OVMS BtElm327", ELM_TASK_STACK, (void*)this,
      ELM_TASK_PRIORITY, &m_task, CORE(1));
    }
  }

#ifdef CONFIG_BLUEDROID_ENABLED

void OvmsBluetoothAppELM327::EventRegistered(esp_ble_gatts_cb_param_t::gatts_reg_evt_param *reg)
  {
  StartTask();

  m_service_id.is_primary = true;
  m_service_id.id.inst_id = 0x00;
  m_service_id.id.uuid.len = ESP_UUID_LEN_16;
  m_service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_ELM327;
  ESP_LOGI(TAG, "Creating service %04x on interface %d",
    m_service_id.id.uuid.uuid.uuid16, m_gatts_if);
  esp_ble_gatts_create_service(m_gatts_if, &m_service_id, GATTS_NUM_HANDLE_ELM327);
  }

// Service setup sequence: create service → add notify characteristic → add its
// CCCD descriptor → add write characteristic → start service.

void OvmsBluetoothAppELM327::EventCreate(esp_ble_gatts_cb_param_t::gatts_add_attr_tab_evt_param *attrtab)
  {
  esp_bt_uuid_t uuid;
  uuid.len = ESP_UUID_LEN_16;
  uuid.uuid.uuid16 = GATTS_CHAR_UUID_ELM327_NOTIFY;
  esp_err_t ret = esp_ble_gatts_add_char(m_service_handle, &uuid,
    ELM_PERM_READ,
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
    NULL, NULL);
  if (ret)
    ESP_LOGE(TAG, "add notify char failed, error code = %x", ret);
  }

void OvmsBluetoothAppELM327::EventAddChar(esp_ble_gatts_cb_param_t::gatts_add_char_evt_param *addchar)
  {
  if (addchar->status != ESP_GATT_OK)
    {
    ESP_LOGE(TAG, "add char failed, status = %d", addchar->status);
    return;
    }

  if (addchar->char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_ELM327_NOTIFY)
    {
    m_notify_handle = addchar->attr_handle;
    esp_bt_uuid_t uuid;
    uuid.len = ESP_UUID_LEN_16;
    uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    esp_err_t ret = esp_ble_gatts_add_char_descr(m_service_handle, &uuid,
      ELM_PERM_READ | ELM_PERM_WRITE, NULL, NULL);
    if (ret)
      ESP_LOGE(TAG, "add char descr failed, error code = %x", ret);
    }
  else if (addchar->char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_ELM327_WRITE)
    {
    m_write_handle = addchar->attr_handle;
    esp_ble_gatts_start_service(m_service_handle);
    }
  }

void OvmsBluetoothAppELM327::EventAddCharDescr(esp_ble_gatts_cb_param_t::gatts_add_char_descr_evt_param *adddescr)
  {
  m_notify_descr_handle = adddescr->attr_handle;

  esp_bt_uuid_t uuid;
  uuid.len = ESP_UUID_LEN_16;
  uuid.uuid.uuid16 = GATTS_CHAR_UUID_ELM327_WRITE;
  esp_err_t ret = esp_ble_gatts_add_char(m_service_handle, &uuid,
    ELM_PERM_WRITE,
    ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
    NULL, NULL);
  if (ret)
    ESP_LOGE(TAG, "add write char failed, error code = %x", ret);
  }

void OvmsBluetoothAppELM327::EventConnect(esp_ble_gatts_cb_param_t::gatts_connect_evt_param *connect)
  {
  m_connected = true;
  m_notifying = false;
  m_congested = false;
  m_mtu = 0;
  Enqueue(ELM_MSG_RESET, NULL);
  }

void OvmsBluetoothAppELM327::EventDisconnect(esp_ble_gatts_cb_param_t::gatts_disconnect_evt_param *disconnect)
  {
  m_connected = false;
  m_notifying = false;
  m_congested = false;
  }

void OvmsBluetoothAppELM327::EventCongest(esp_ble_gatts_cb_param_t::gatts_congest_evt_param *congest)
  {
  m_congested = congest->congested;
  }

void OvmsBluetoothAppELM327::EventRead(esp_ble_gatts_cb_param_t::gatts_read_evt_param *read)
  {
  esp_gatt_rsp_t rsp;
  memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
  rsp.attr_value.handle = read->handle;
  if (read->handle == m_notify_descr_handle)
    {
    rsp.attr_value.len = 2;
    rsp.attr_value.value[0] = m_notifying ? 0x01 : 0x00;
    rsp.attr_value.value[1] = 0x00;
    }
  else
    {
    // The adapter output is only sent as notifications
    rsp.attr_value.len = 0;
    }
  esp_ble_gatts_send_response(m_gatts_if, read->conn_id, read->trans_id, ESP_GATT_OK, &rsp);
  }

void OvmsBluetoothAppELM327::EventWrite(esp_ble_gatts_cb_param_t::gatts_write_evt_param *write)
  {
  if (write->is_prep)
    {
    // Prepared (long) writes are not needed for ELM327 commands
    ESP_LOGW(TAG, "Prepared write ignored (%d bytes)", write->len);
    }
  else if (write->handle == m_notify_descr_handle && write->len == 2)
    {
    uint16_t value = write->value[1] << 8 | write->value[0];
    m_notifying = (value & 0x0001) != 0;
    ESP_LOGI(TAG, "Notifications %s", m_notifying ? "enabled" : "disabled");
    }
  else if (write->handle == m_write_handle || write->handle == m_notify_handle)
    {
    m_stat_rx_bytes += write->len;
    Enqueue(ELM_MSG_DATA, new std::string((const char*)write->value, write->len));
    }

  if (write->need_rsp)
    {
    esp_ble_gatts_send_response(m_gatts_if, write->conn_id, write->trans_id, ESP_GATT_OK, NULL);
    }
  }

#endif // CONFIG_BLUEDROID_ENABLED

void OvmsBluetoothAppELM327::Enqueue(int type, std::string* data)
  {
  elm_msg_t msg;
  msg.type = type;
  msg.data = data;
  if (m_queue == NULL || xQueueSend(m_queue, &msg, 0) != pdTRUE)
    {
    m_stat_dropped++;
    ESP_LOGW(TAG, "Input queue full, data dropped");
    if (data)
      delete data;
    }
  }

void OvmsBluetoothAppELM327::TaskEntry(void* param)
  {
  ((OvmsBluetoothAppELM327*)param)->Task();
  }

void OvmsBluetoothAppELM327::Task()
  {
  elm_msg_t msg;
  for (;;)
    {
    if (xQueueReceive(m_queue, &msg, portMAX_DELAY) != pdTRUE)
      continue;
    if (msg.type == ELM_MSG_RESET)
      {
      LoadConfig();
      m_engine.Reset();
      }
    else if (msg.type == ELM_MSG_DATA && msg.data)
      {
      ESP_LOGD(TAG, "rx: %s", msg.data->c_str());
      std::string output = m_engine.Input(msg.data->data(), msg.data->size());
      delete msg.data;
      if (!output.empty())
        {
        ESP_LOGD(TAG, "tx: %s", output.c_str());
        Send(output);
        }
      }
    }
  }

#ifdef CONFIG_BLUEDROID_ENABLED

void OvmsBluetoothAppELM327::Send(const std::string& data)
  {
  // Notification payload is limited to MTU - 3 bytes (default MTU 23)
  size_t chunk_max = (m_mtu > 3) ? m_mtu - 3 : 20;
  const char* p = data.data();
  size_t len = data.size();

  while (len > 0)
    {
    if (!m_connected || !m_notifying)
      {
      ESP_LOGD(TAG, "Output dropped: client not subscribed");
      return;
      }
    for (int wait = 0; m_congested && wait < 100; wait++)
      vTaskDelay(pdMS_TO_TICKS(10));

    size_t chunk = (len < chunk_max) ? len : chunk_max;
    esp_err_t err = ESP_FAIL;
    for (int retry = 0; retry < 5; retry++)
      {
      err = esp_ble_gatts_send_indicate(m_gatts_if, m_conn_id, m_notify_handle,
        chunk, (uint8_t*)p, false);
      if (err == ESP_OK)
        break;
      vTaskDelay(pdMS_TO_TICKS(20));
      }
    if (err != ESP_OK)
      {
      ESP_LOGW(TAG, "Notification failed: %s", esp_err_to_name(err));
      return;
      }
    m_stat_tx_bytes += chunk;
    p += chunk;
    len -= chunk;
    }
  }

#endif // CONFIG_BLUEDROID_ENABLED

int OvmsBluetoothAppELM327::Request(uint32_t txid, uint32_t rxid, bool ext29,
                                    const std::string& request, std::string& response,
                                    int timeout_ms, uint32_t& rxid_used)
  {
#ifdef CONFIG_OVMS_COMP_POLLER
  if (!MyVehicleFactory.m_currentvehicle)
    {
    ESP_LOGW(TAG, "No vehicle module loaded");
    return ELM327_REQ_UNAVAILABLE;
    }
  std::string busname = MyConfig.GetParamValue("bluetooth", "elm327.bus", "can1");
  canbus* bus = (canbus*)MyPcpApp.FindDeviceByName(busname.c_str());
  if (!bus || bus->m_mode != CAN_MODE_ACTIVE)
    {
    ESP_LOGW(TAG, "CAN bus %s not available or not in active mode", busname.c_str());
    return ELM327_REQ_UNAVAILABLE;
    }
  OvmsPoller* poller = MyPollers.GetPoller(bus, true);
  if (!poller)
    return ELM327_REQ_UNAVAILABLE;

  int rc = poller->PollSingleRequest(txid, rxid, request, response, timeout_ms,
    ext29 ? ISOTP_EXTFRAME : ISOTP_STD);
  rxid_used = rxid;
  ESP_LOGD(TAG, "Request %" PRIx32 "/%" PRIx32 " %s: rc=%d, %d bytes",
    txid, rxid, hexencode(request).c_str(), rc, (int)response.size());
  return rc;
#else
  return ELM327_REQ_UNAVAILABLE;
#endif
  }

bool OvmsBluetoothAppELM327::GetVoltage(float& volts)
  {
  if (!StandardMetrics.ms_v_bat_12v_voltage->IsDefined())
    return false;
  volts = StandardMetrics.ms_v_bat_12v_voltage->AsFloat();
  return true;
  }

void OvmsBluetoothAppELM327::Status(OvmsWriter* writer)
  {
  writer->printf("Bluetooth:      %s\n",
    MyPeripherals->m_esp32bluetooth->IsServiceRunning() ? "running" : "not running (use 'power bluetooth on')");
  writer->printf("Device name:    %s\n", MyConfig.GetParamValue("bluetooth", "elm327.name", "OBDII").c_str());
  writer->printf("Bonded devices: %d\n", BondCount());
  int remain = PairingWindowRemaining();
  if (remain > 0)
    writer->printf("Pairing window: open (%d s)\n", remain);
  else
    writer->puts("Pairing window: closed");
  writer->printf("Client:         %s%s (MTU %d)\n",
    m_connected ? "connected" : "not connected",
    m_notifying ? ", subscribed" : "",
    m_mtu ? m_mtu : 23);
  std::string busname = MyConfig.GetParamValue("bluetooth", "elm327.bus", "can1");
  canbus* bus = (canbus*)MyPcpApp.FindDeviceByName(busname.c_str());
  writer->printf("CAN bus:        %s (%s)\n", busname.c_str(),
    !bus ? "not found" : (bus->m_mode == CAN_MODE_ACTIVE) ? "active" : "not active");
  writer->printf("Services:       %s\n",
    MyConfig.GetParamValue("bluetooth", "elm327.services", "01,02,03,06,07,09,0A,19,1A,21,22,3E").c_str());
  writer->printf("Statistics:     %" PRIu32 " commands, %" PRIu32 " requests, %" PRIu32 " errors, %" PRIu32 " blocked\n",
    m_engine.m_stat_commands, m_engine.m_stat_requests, m_engine.m_stat_errors, m_engine.m_stat_blocked);
  writer->printf("                %" PRIu32 " bytes rx, %" PRIu32 " bytes tx, %" PRIu32 " dropped\n",
    m_stat_rx_bytes, m_stat_tx_bytes, m_stat_dropped);
  }
