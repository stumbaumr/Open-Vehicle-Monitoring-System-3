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

#ifndef __ELM327_ENGINE_H__
#define __ELM327_ENGINE_H__

// ELM327 command interpreter emulation.
//
// The engine implements the ELM327 text protocol (AT commands, hex requests,
// response formatting) and forwards OBD2/UDS requests to a Transport, which
// performs the actual ISO-TP request on the vehicle CAN bus.
//
// The engine has no dependencies on the OVMS framework or ESP-IDF, so it can
// be unit tested natively (see ../tests).

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>

// Transport result codes (compatible with POLLSINGLE_* codes):
#define ELM327_REQ_OK               0     // success, response is valid
#define ELM327_REQ_TIMEOUT          -1    // no response within timeout -> "NO DATA"
#define ELM327_REQ_TXFAILURE        -2    // CAN transmission failure -> "CAN ERROR"
#define ELM327_REQ_NORESPONSE       -3    // no response -> "NO DATA"
#define ELM327_REQ_UNAVAILABLE      -4    // bus/vehicle not available -> "UNABLE TO CONNECT"
                                          // > 0: UDS negative response code (NRC)

class Elm327Engine
  {
  public:
    class Transport
      {
      public:
        virtual ~Transport() {}

        // Perform a single ISO-TP request.
        //  txid/rxid:  CAN IDs (rxid 0 = functional/broadcast request)
        //  ext29:      true = 29 bit CAN IDs
        //  request:    service ID + parameters (binary)
        //  response:   on success: response data following the service ID and PID echo
        //  rxid_used:  on success/NRC: CAN ID the response was received from (0 = unknown)
        //  returns:    ELM327_REQ_* code or NRC (> 0)
        virtual int Request(uint32_t txid, uint32_t rxid, bool ext29,
                            const std::string& request, std::string& response,
                            int timeout_ms, uint32_t& rxid_used) = 0;

        // Read the auxiliary battery voltage, return false if not available.
        virtual bool GetVoltage(float& volts) = 0;
      };

  public:
    Elm327Engine(Transport* transport);
    ~Elm327Engine();

  public:
    // Configuration (persistent across ATZ/ATD):
    void SetDefaultProtocol(char protocol);           // '6'…'9' (used for ATSP0 / auto)
    void SetMinTimeout(int timeout_ms);               // lower limit for request timeouts
    bool SetAllowedServices(const std::string& list); // "01,09,22" or "*" (all)
    bool IsServiceAllowed(uint8_t sid) const;

  public:
    // Reset to power-on state (ATZ), clears the input buffer.
    void Reset();

    // Feed raw input bytes, returns the output for all completed command lines.
    std::string Input(const char* data, size_t len);

    // Process one command line (without CR), returns the complete output
    // including echo, response lines and the prompt.
    std::string ProcessLine(const std::string& line);

  public:
    // Statistics:
    uint32_t m_stat_commands;
    uint32_t m_stat_requests;
    uint32_t m_stat_errors;
    uint32_t m_stat_blocked;

  protected:
    void SetDefaults();
    bool ProcessAT(const std::string& cmd, std::vector<std::string>& out);
    void ProcessRequest(const std::string& hex, std::vector<std::string>& out);
    void FormatResponse(uint32_t rxid, const std::string& data, std::vector<std::string>& out);
    std::string FormatHeader(uint32_t id) const;
    std::string FormatBytes(const uint8_t* data, size_t len) const;
    bool IsExt29() const;
    char EffectiveProtocol() const;
    void GetIds(uint32_t& txid, uint32_t& rxid) const;
    std::string ProtocolName(char protocol) const;

  protected:
    Transport* m_transport;

    // configuration:
    char m_default_protocol;
    int m_min_timeout_ms;
    bool m_allowed[256];

    // ELM327 state:
    bool m_echo;
    bool m_linefeeds;
    bool m_spaces;
    bool m_headers;
    bool m_caf;
    char m_protocol;            // '0' = auto
    uint8_t m_timeout;          // ATST value (unit 4 ms)
    int m_header_len;           // ATSH digits: 0 (not set), 3, 6 or 8
    uint32_t m_header;          // ATSH value
    uint8_t m_priority;         // ATCP value (29 bit)
    bool m_cra_set;
    uint32_t m_cra;             // ATCRA value

    std::string m_inbuf;
    std::string m_last;
  };

#endif //#ifndef __ELM327_ENGINE_H__
