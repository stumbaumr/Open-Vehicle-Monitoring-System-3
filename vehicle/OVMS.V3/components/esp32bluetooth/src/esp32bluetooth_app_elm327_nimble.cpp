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

// ELM327 BLE adapter emulation: NimBLE GATT service.
// The connection security is handled in esp32bluetooth_nimble.cpp.

#include "sdkconfig.h"
#ifdef CONFIG_NIMBLE_ENABLED

#include <string.h>
#include "esp32bluetooth_app_elm327.h"
#include "ovms_peripherals.h"
#include "ovms_log.h"

// NimBLE headers last (they define min() / max() macros):
#include "host/ble_hs.h"
#undef min
#undef max

static const char *TAG = "bt-app-elm327";

#define ELM_MSG_RESET               1
#define ELM_MSG_DATA                2

// GATT service table (filled at runtime, C++ has no C99 compound literals).
// All characteristics require an encrypted connection.
static ble_uuid16_t elm_svc_uuid;
static ble_uuid16_t elm_notify_uuid;
static ble_uuid16_t elm_write_uuid;
static struct ble_gatt_chr_def elm_chrs[3];
static struct ble_gatt_svc_def elm_svcs[2];

static int elm_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
  {
  return MyBluetoothAppELM327.NimbleAccess(conn_handle, attr_handle, ctxt);
  }

bool OvmsBluetoothAppELM327::NimbleRegister()
  {
  StartTask();

  elm_svc_uuid.u.type = BLE_UUID_TYPE_16;
  elm_svc_uuid.value = GATTS_SERVICE_UUID_ELM327;
  elm_notify_uuid.u.type = BLE_UUID_TYPE_16;
  elm_notify_uuid.value = GATTS_CHAR_UUID_ELM327_NOTIFY;
  elm_write_uuid.u.type = BLE_UUID_TYPE_16;
  elm_write_uuid.value = GATTS_CHAR_UUID_ELM327_WRITE;

  memset(elm_chrs, 0, sizeof(elm_chrs));
  elm_chrs[0].uuid = &elm_notify_uuid.u;
  elm_chrs[0].access_cb = elm_access_cb;
  elm_chrs[0].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY;
  elm_chrs[0].val_handle = &m_notify_handle;
  elm_chrs[1].uuid = &elm_write_uuid.u;
  elm_chrs[1].access_cb = elm_access_cb;
  elm_chrs[1].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC;
  elm_chrs[1].val_handle = &m_write_handle;

  memset(elm_svcs, 0, sizeof(elm_svcs));
  elm_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
  elm_svcs[0].uuid = &elm_svc_uuid.u;
  elm_svcs[0].characteristics = elm_chrs;

  int rc = ble_gatts_count_cfg(elm_svcs);
  if (rc == 0)
    rc = ble_gatts_add_svcs(elm_svcs);
  if (rc != 0)
    {
    ESP_LOGE(TAG, "Registering service %04x failed, rc=%d", GATTS_SERVICE_UUID_ELM327, rc);
    return false;
    }
  return true;
  }

void OvmsBluetoothAppELM327::NimbleConnect(uint16_t conn_handle)
  {
  m_conn_handle = conn_handle;
  m_connected = true;
  m_notifying = false;
  m_authorized = false;
  m_mtu = 0;
  Enqueue(ELM_MSG_RESET, NULL);
  }

void OvmsBluetoothAppELM327::NimbleDisconnect()
  {
  m_conn_handle = BLE_HS_CONN_HANDLE_NONE;
  m_connected = false;
  m_notifying = false;
  m_authorized = false;
  }

void OvmsBluetoothAppELM327::NimbleAuthorized()
  {
  m_authorized = true;
  }

void OvmsBluetoothAppELM327::NimbleSubscribe(uint16_t attr_handle, bool notify)
  {
  if (attr_handle != m_notify_handle)
    return;
  m_notifying = notify;
  ESP_LOGI(TAG, "Notifications %s", m_notifying ? "enabled" : "disabled");
  }

void OvmsBluetoothAppELM327::NimbleMtu(uint16_t mtu)
  {
  m_mtu = mtu;
  }

int OvmsBluetoothAppELM327::NimbleAccess(uint16_t conn_handle, uint16_t attr_handle,
                                         struct ble_gatt_access_ctxt *ctxt)
  {
  // The stack checks the encryption, the pairing window check is done
  // after the encryption (see esp32bluetooth_nimble.cpp):
  if (!m_authorized)
    return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;

  switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
      // The adapter output is only sent as notifications
      return 0;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      {
      uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
      if (len == 0)
        return 0;
      std::string* data = new std::string(len, '\0');
      uint16_t copied = 0;
      if (ble_hs_mbuf_to_flat(ctxt->om, &(*data)[0], len, &copied) != 0)
        {
        delete data;
        return BLE_ATT_ERR_UNLIKELY;
        }
      data->resize(copied);
      m_stat_rx_bytes += copied;
      Enqueue(ELM_MSG_DATA, data);
      return 0;
      }

    default:
      return BLE_ATT_ERR_UNLIKELY;
    }
  }

void OvmsBluetoothAppELM327::Send(const std::string& data)
  {
  // Notification payload is limited to MTU - 3 bytes (default MTU 23)
  size_t chunk_max = (m_mtu > 3) ? m_mtu - 3 : 20;
  const char* p = data.data();
  size_t len = data.size();

  while (len > 0)
    {
    if (!m_connected || !m_notifying || !m_authorized)
      {
      ESP_LOGD(TAG, "Output dropped: client not subscribed");
      return;
      }

    size_t chunk = (len < chunk_max) ? len : chunk_max;
    int rc = BLE_HS_ENOMEM;
    // Retry while the buffers are in use (max. 1 second)
    for (int retry = 0; retry < 50; retry++)
      {
      struct os_mbuf *om = ble_hs_mbuf_from_flat(p, chunk);
      if (om)
        {
        // The stack frees the mbuf, also on errors:
        rc = ble_gattc_notify_custom(m_conn_handle, m_notify_handle, om);
        if (rc == 0)
          break;
        }
      if (!m_connected)
        return;
      vTaskDelay(pdMS_TO_TICKS(20));
      }
    if (rc != 0)
      {
      ESP_LOGW(TAG, "Notification failed, rc=%d", rc);
      return;
      }
    m_stat_tx_bytes += chunk;
    p += chunk;
    len -= chunk;
    }
  }

int OvmsBluetoothAppELM327::BondCount()
  {
  if (!MyPeripherals->m_esp32bluetooth->IsServiceRunning())
    return 0;
  int count = 0;
  if (ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count) != 0)
    return 0;
  return count;
  }

#endif // CONFIG_NIMBLE_ENABLED
