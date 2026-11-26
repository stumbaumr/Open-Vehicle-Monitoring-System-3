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

#ifndef __ESP32BLUETOOTH_APP_ELM327_H__
#define __ESP32BLUETOOTH_APP_ELM327_H__

// ELM327 BLE adapter emulation.
//
// Provides the GATT service of common ELM327 BLE adapters (service 0xFFF0,
// notify characteristic 0xFFF1, write characteristic 0xFFF2) and forwards
// OBD2/UDS requests to the vehicle CAN bus using the OVMS poller.
//
// New devices can only bond while the pairing window is open.

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp32bluetooth.h"
#ifdef CONFIG_BLUEDROID_ENABLED
#include "esp32bluetooth_gatts.h"
#endif
#ifdef CONFIG_NIMBLE_ENABLED
// Do not include the NimBLE headers here: they define min() / max() macros,
// which break the C++ standard library headers.
struct ble_gatt_access_ctxt;
#endif
#include "elm327_engine.h"
#include "ovms_mutex.h"

#define GATTS_APP_UUID_OVMS_ELM327        0x40
#define GATTS_SERVICE_UUID_ELM327         0xfff0
#define GATTS_CHAR_UUID_ELM327_NOTIFY     0xfff1
#define GATTS_CHAR_UUID_ELM327_WRITE      0xfff2
#define GATTS_NUM_HANDLE_ELM327           8

class OvmsWriter;

class OvmsBluetoothAppELM327 :
#ifdef CONFIG_BLUEDROID_ENABLED
  public esp32bluetoothApp,
#endif
  public Elm327Engine::Transport
  {
  public:
    OvmsBluetoothAppELM327();
    ~OvmsBluetoothAppELM327();

#ifdef CONFIG_BLUEDROID_ENABLED
  public:
    // esp32bluetoothApp:
    bool GetConnectSecurity(esp_ble_sec_act_t* action);
    bool AcceptSecurityRequest(esp_bd_addr_t bda);
    void EventRegistered(esp_ble_gatts_cb_param_t::gatts_reg_evt_param *reg);
    void EventCreate(esp_ble_gatts_cb_param_t::gatts_add_attr_tab_evt_param *attrtab);
    void EventAddChar(esp_ble_gatts_cb_param_t::gatts_add_char_evt_param *addchar);
    void EventAddCharDescr(esp_ble_gatts_cb_param_t::gatts_add_char_descr_evt_param *adddescr);
    void EventConnect(esp_ble_gatts_cb_param_t::gatts_connect_evt_param *connect);
    void EventDisconnect(esp_ble_gatts_cb_param_t::gatts_disconnect_evt_param *disconnect);
    void EventRead(esp_ble_gatts_cb_param_t::gatts_read_evt_param *read);
    void EventWrite(esp_ble_gatts_cb_param_t::gatts_write_evt_param *write);
    void EventCongest(esp_ble_gatts_cb_param_t::gatts_congest_evt_param *congest);
#endif

#ifdef CONFIG_NIMBLE_ENABLED
  public:
    // Called by the NimBLE host (esp32bluetooth_nimble.cpp):
    bool NimbleRegister();
    void NimbleConnect(uint16_t conn_handle);
    void NimbleDisconnect();
    void NimbleAuthorized();
    void NimbleSubscribe(uint16_t attr_handle, bool notify);
    void NimbleMtu(uint16_t mtu);
    int NimbleAccess(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt);
#endif

  public:
    // Elm327Engine::Transport:
    int Request(uint32_t txid, uint32_t rxid, bool ext29,
                const std::string& request, std::string& response,
                int timeout_ms, uint32_t& rxid_used);
    bool GetVoltage(float& volts);
    void RequestDone(uint32_t txid, uint8_t sid, uint16_t pid, int pidlen,
                     int rc, size_t len);

  public:
    void OpenPairingWindow(int seconds);
    int PairingWindowRemaining();
    void Status(OvmsWriter* writer);
    void ShowRequests(OvmsWriter* writer, int maxlines);
    void ClearRequests();

  private:
    typedef struct
      {
      int type;
      std::string* data;
      } elm_msg_t;

    static void TaskEntry(void* param);
    void Task();
    void StartTask();
    void LoadConfig();
    void Send(const std::string& data);
    void Enqueue(int type, std::string* data);
    int BondCount();
#ifdef CONFIG_BLUEDROID_ENABLED
    bool IsBonded(esp_bd_addr_t bda);
#endif

  private:
    Elm327Engine m_engine;
    OvmsMutex m_reqstats_mutex;
    Elm327RequestStats m_reqstats;
    Elm327RequestStats::Entry* m_reqstats_table;
    QueueHandle_t m_queue;
    TaskHandle_t m_task;

    uint16_t m_notify_handle;
    uint16_t m_notify_descr_handle;
    uint16_t m_write_handle;

    volatile bool m_connected;
    volatile bool m_notifying;
    volatile bool m_congested;
    int64_t m_pairing_until_us;

#ifdef CONFIG_NIMBLE_ENABLED
    volatile uint16_t m_conn_handle;
    volatile uint16_t m_mtu;
    volatile bool m_authorized;
#endif

    uint32_t m_stat_rx_bytes;
    uint32_t m_stat_tx_bytes;
    uint32_t m_stat_dropped;
  };

extern OvmsBluetoothAppELM327 MyBluetoothAppELM327;

#endif //#ifndef __ESP32BLUETOOTH_APP_ELM327_H__
