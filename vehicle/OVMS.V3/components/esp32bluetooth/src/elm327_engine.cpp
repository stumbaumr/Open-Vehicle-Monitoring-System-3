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

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "elm327_engine.h"

#define ELM327_VERSION          "ELM327 v1.5"
#define ELM327_DEVICE_ID        "OVMS ELM327 BLE BRIDGE"
#define ELM327_MAX_LINE         256
#define ELM327_DEFAULT_TIMEOUT  0x32      // ATST default (x 4 ms = 200 ms)
#define ELM327_DEFAULT_PRIORITY 0x18      // ATCP default

// Default 11 bit functional (broadcast) request ID, and the response ID used for display:
#define OBD_FUNC_ID_11          0x7df
#define OBD_RESP_ID_11          0x7e8
// Default 29 bit functional request ID, and the response ID of the first ECU:
#define OBD_FUNC_ID_29          0x18db33f1
#define OBD_RESP_ID_29          0x18daf110

// Number of PID bytes echoed in the response, must match the OVMS poller
// definitions (POLL_TYPE_HAS_16BIT_PID / POLL_TYPE_HAS_NO_PID):
static int PidLength(uint8_t sid)
  {
  switch (sid)
    {
    case 0x05:  // Mode 05 read oxygen sensor monitoring test results
    case 0x22:  // UDS ReadDataByIdentifier
    case 0x24:  // UDS ReadScalingDataByIdentifier
    case 0x2a:  // ZombieVerterVCU
    case 0x2e:  // UDS WriteDataByIdentifier
    case 0x2f:  // UDS InputOutputControlByIdentifier
      return 2;
    case 0x03:  // Mode 03 read emission-related DTC
    case 0x04:  // Mode 04 clear emission-related DTC
    case 0x07:  // Mode 07 read driving cycle emission-related DTC
    case 0x0a:  // Mode 0A read permanent DTC
    case 0x14:  // UDS ClearDiagnosticInformation
    case 0x18:  // VW request type 18
    case 0x23:  // UDS ReadMemoryByAddress
      return 0;
    default:
      return 1;
    }
  }

static bool IsHexDigit(char c)
  {
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
  }

static bool ParseHex(const std::string& s, uint32_t& value)
  {
  if (s.empty() || s.size() > 8)
    return false;
  value = 0;
  for (char c : s)
    {
    if (!IsHexDigit(c))
      return false;
    value = (value << 4) | (uint32_t)((c <= '9') ? (c - '0') : (c - 'A' + 10));
    }
  return true;
  }

static bool StartsWith(const std::string& s, const char* prefix)
  {
  return s.compare(0, strlen(prefix), prefix) == 0;
  }

Elm327Engine::Elm327Engine(Transport* transport)
  {
  m_transport = transport;
  m_default_protocol = '6';
  m_min_timeout_ms = 500;
  SetAllowedServices("01,02,03,06,07,09,0A,19,1A,21,22,3E");
  m_stat_commands = 0;
  m_stat_requests = 0;
  m_stat_errors = 0;
  m_stat_blocked = 0;
  Reset();
  }

Elm327Engine::~Elm327Engine()
  {
  }

void Elm327Engine::SetDefaultProtocol(char protocol)
  {
  if (protocol >= '6' && protocol <= '9')
    m_default_protocol = protocol;
  }

void Elm327Engine::SetMinTimeout(int timeout_ms)
  {
  if (timeout_ms > 0)
    m_min_timeout_ms = timeout_ms;
  }

bool Elm327Engine::SetAllowedServices(const std::string& list)
  {
  bool allowed[256];
  memset(allowed, 0, sizeof(allowed));

  std::string item;
  std::string src = list + ",";
  for (char c : src)
    {
    if (c == ',' || c == ' ' || c == ';')
      {
      if (item.empty())
        continue;
      if (item == "*")
        {
        memset(allowed, 1, sizeof(allowed));
        }
      else
        {
        uint32_t sid;
        if (item.size() > 2 || !ParseHex(item, sid))
          return false;
        allowed[sid] = true;
        }
      item.clear();
      }
    else
      {
      item += (char)toupper((unsigned char)c);
      }
    }

  memcpy(m_allowed, allowed, sizeof(m_allowed));
  return true;
  }

bool Elm327Engine::IsServiceAllowed(uint8_t sid) const
  {
  return m_allowed[sid];
  }

void Elm327Engine::SetDefaults()
  {
  m_echo = true;
  m_linefeeds = false;
  m_spaces = true;
  m_headers = false;
  m_caf = true;
  m_protocol = '0';
  m_timeout = ELM327_DEFAULT_TIMEOUT;
  m_header_len = 0;
  m_header = 0;
  m_priority = ELM327_DEFAULT_PRIORITY;
  m_cra_set = false;
  m_cra = 0;
  }

void Elm327Engine::Reset()
  {
  SetDefaults();
  m_inbuf.clear();
  m_last.clear();
  }

std::string Elm327Engine::Input(const char* data, size_t len)
  {
  std::string output;
  for (size_t i = 0; i < len; i++)
    {
    char c = data[i];
    if (c == '\r')
      {
      output += ProcessLine(m_inbuf);
      m_inbuf.clear();
      }
    else if (c == '\n' || c == 0)
      {
      // ignore
      }
    else if (m_inbuf.size() < ELM327_MAX_LINE)
      {
      m_inbuf += c;
      }
    }
  return output;
  }

std::string Elm327Engine::ProcessLine(const std::string& line)
  {
  std::vector<std::string> out;
  const char* eol = m_linefeeds ? "\r\n" : "\r";
  std::string output;

  if (m_echo)
    {
    // The adapter echoes the received characters, including the CR
    output = line;
    output += '\r';
    }

  // Normalize: upper case, remove spaces and control characters:
  std::string cmd;
  for (char c : line)
    {
    if ((unsigned char)c > ' ')
      cmd += (char)toupper((unsigned char)c);
    }

  if (cmd.empty())
    {
    // Empty line: repeat the last command
    cmd = m_last;
    }

  if (!cmd.empty())
    {
    m_stat_commands++;
    m_last = cmd;
    if (StartsWith(cmd, "AT"))
      {
      if (!ProcessAT(cmd.substr(2), out))
        out.push_back("?");
      }
    else if (StartsWith(cmd, "ST"))
      {
      // STN extensions are not supported
      out.push_back("?");
      }
    else
      {
      ProcessRequest(cmd, out);
      }
    }

  // The state may have changed (ATL, ATZ), so use the new line end for the response:
  eol = m_linefeeds ? "\r\n" : "\r";
  for (const std::string& l : out)
    {
    output += l;
    output += eol;
    }
  output += eol;
  output += ">";
  return output;
  }

std::string Elm327Engine::ProtocolName(char protocol) const
  {
  switch (protocol)
    {
    case '1': return "SAE J1850 PWM";
    case '2': return "SAE J1850 VPW";
    case '3': return "ISO 9141-2";
    case '4': return "ISO 14230-4 (KWP 5BAUD)";
    case '5': return "ISO 14230-4 (KWP FAST)";
    case '6': return "ISO 15765-4 (CAN 11/500)";
    case '7': return "ISO 15765-4 (CAN 29/500)";
    case '8': return "ISO 15765-4 (CAN 11/250)";
    case '9': return "ISO 15765-4 (CAN 29/250)";
    case 'A': return "SAE J1939 (CAN 29/250)";
    case 'B': return "USER1 (CAN 11/125)";
    case 'C': return "USER2 (CAN 11/50)";
    default:  return "AUTO";
    }
  }

char Elm327Engine::EffectiveProtocol() const
  {
  if (m_protocol != '0')
    return m_protocol;
  // Auto: a 29 bit header selects the 29 bit variant of the default protocol
  bool ext29 = (m_header_len >= 6);
  switch (m_default_protocol)
    {
    case '6': return ext29 ? '7' : '6';
    case '7': return ext29 ? '7' : '6';
    case '8': return ext29 ? '9' : '8';
    case '9': return ext29 ? '9' : '8';
    default:  return m_default_protocol;
    }
  }

bool Elm327Engine::IsExt29() const
  {
  char p = EffectiveProtocol();
  return (p == '7' || p == '9' || p == 'A');
  }

bool Elm327Engine::ProcessAT(const std::string& cmd, std::vector<std::string>& out)
  {
  // Commands without arguments:
  if (cmd == "Z" || cmd == "WS")
    {
    SetDefaults();
    out.push_back("");
    out.push_back("");
    out.push_back(ELM327_VERSION);
    return true;
    }
  if (cmd == "D")
    {
    SetDefaults();
    out.push_back("OK");
    return true;
    }
  if (cmd == "I")
    {
    out.push_back(ELM327_VERSION);
    return true;
    }
  if (cmd == "@1")
    {
    out.push_back(ELM327_DEVICE_ID);
    return true;
    }
  if (cmd == "E0" || cmd == "E1")
    {
    m_echo = (cmd[1] == '1');
    out.push_back("OK");
    return true;
    }
  if (cmd == "L0" || cmd == "L1")
    {
    m_linefeeds = (cmd[1] == '1');
    out.push_back("OK");
    return true;
    }
  if (cmd == "S0" || cmd == "S1")
    {
    m_spaces = (cmd[1] == '1');
    out.push_back("OK");
    return true;
    }
  if (cmd == "H0" || cmd == "H1")
    {
    m_headers = (cmd[1] == '1');
    out.push_back("OK");
    return true;
    }
  if (cmd == "CAF0" || cmd == "CAF1")
    {
    m_caf = (cmd[3] == '1');
    out.push_back("OK");
    return true;
    }
  if (cmd == "AR")
    {
    m_cra_set = false;
    out.push_back("OK");
    return true;
    }
  if (cmd == "DPN")
    {
    std::string p(1, EffectiveProtocol());
    out.push_back((m_protocol == '0') ? "A" + p : p);
    return true;
    }
  if (cmd == "DP")
    {
    std::string name = ProtocolName(EffectiveProtocol());
    out.push_back((m_protocol == '0') ? "AUTO, " + name : name);
    return true;
    }
  if (cmd == "RV")
    {
    float volts = 0;
    if (!m_transport || !m_transport->GetVoltage(volts))
      volts = 0;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fV", volts);
    out.push_back(buf);
    return true;
    }
  if (cmd == "IGN")
    {
    out.push_back("ON");
    return true;
    }

  // Commands with arguments:
  if (StartsWith(cmd, "SP") || StartsWith(cmd, "TP"))
    {
    std::string arg = cmd.substr(2);
    if (arg.size() == 2 && arg[0] == 'A')
      arg = arg.substr(1);
    if (arg.size() != 1 || !(IsHexDigit(arg[0]) && arg[0] <= 'C'))
      return false;
    m_protocol = arg[0];
    out.push_back("OK");
    return true;
    }
  if (StartsWith(cmd, "SH"))
    {
    std::string arg = cmd.substr(2);
    uint32_t value;
    if ((arg.size() != 3 && arg.size() != 6 && arg.size() != 8) || !ParseHex(arg, value))
      return false;
    m_header = value;
    m_header_len = (int)arg.size();
    out.push_back("OK");
    return true;
    }
  if (StartsWith(cmd, "CP"))
    {
    uint32_t value;
    if (!ParseHex(cmd.substr(2), value) || value > 0x1f)
      return false;
    m_priority = (uint8_t)value;
    out.push_back("OK");
    return true;
    }
  if (StartsWith(cmd, "CRA"))
    {
    std::string arg = cmd.substr(3);
    uint32_t value;
    if (arg.empty() || arg.find('X') != std::string::npos)
      {
      // reset / wildcard filter: use the default response ID
      m_cra_set = false;
      }
    else if ((arg.size() == 3 || arg.size() == 8) && ParseHex(arg, value))
      {
      m_cra = value;
      m_cra_set = true;
      }
    else
      {
      return false;
      }
    out.push_back("OK");
    return true;
    }
  if (StartsWith(cmd, "ST"))
    {
    uint32_t value;
    if (!ParseHex(cmd.substr(2), value) || value > 0xff)
      return false;
    m_timeout = (value == 0) ? ELM327_DEFAULT_TIMEOUT : (uint8_t)value;
    out.push_back("OK");
    return true;
    }

  // Commands accepted without effect (handled by the OVMS CAN framework or not applicable):
  static const char* const ignored[] =
    {
    "AT0", "AT1", "AT2", "AL", "NL", "BI", "PC", "LP", "M0", "M1", "R0", "R1",
    "V0", "V1", "D0", "D1", "CFC0", "CFC1", "CEA", "CF", "CM", "FCSH", "FCSD",
    "FCSM", "PP", "PB", "KW", "IB", "IIA", "SI", "FI", "SW", "WM", "SR", "SS",
    "TA", "CSM", "JE", "JS", "JHF", "JTM",
    NULL
    };
  for (int i = 0; ignored[i]; i++)
    {
    if (StartsWith(cmd, ignored[i]))
      {
      out.push_back("OK");
      return true;
      }
    }

  // Unsupported (e.g. monitoring modes ATMA/ATMR/ATMT, ATBD, ATCS):
  return false;
  }

void Elm327Engine::GetIds(uint32_t& txid, uint32_t& rxid) const
  {
  if (!IsExt29())
    {
    txid = (m_header_len == 3) ? (m_header & 0x7ff) : OBD_FUNC_ID_11;
    if (m_cra_set)
      rxid = m_cra;
    else if (txid == OBD_FUNC_ID_11)
      rxid = 0;   // broadcast, the poller listens to 0x7e8…0x7ef
    else
      rxid = txid + 8;
    }
  else
    {
    if (m_header_len == 8)
      txid = m_header & 0x1fffffff;
    else if (m_header_len == 6)
      txid = ((uint32_t)m_priority << 24) | m_header;
    else
      txid = OBD_FUNC_ID_29;
    if (m_cra_set)
      rxid = m_cra;
    else if (((txid >> 16) & 0xff) == 0xdb)
      rxid = OBD_RESP_ID_29;  // functional request: listen to the first ECU
    else
      rxid = (txid & 0xffff0000) | ((txid & 0xff) << 8) | ((txid >> 8) & 0xff);
    }
  }

void Elm327Engine::ProcessRequest(const std::string& hex, std::vector<std::string>& out)
  {
  for (char c : hex)
    {
    if (!IsHexDigit(c))
      {
      out.push_back("?");
      return;
      }
    }

  // An odd number of digits: the last digit is the expected number of responses (ignored)
  std::string digits = hex.substr(0, hex.size() & ~((size_t)1));
  if (digits.empty())
    {
    out.push_back("?");
    return;
    }

  std::string bytes;
  for (size_t i = 0; i < digits.size(); i += 2)
    {
    uint32_t value;
    ParseHex(digits.substr(i, 2), value);
    bytes += (char)value;
    }

  if (!m_caf)
    {
    // Raw mode: the client sends the ISO-TP single frame PCI byte
    uint8_t pci = (uint8_t)bytes[0];
    size_t len = pci & 0x0f;
    if ((pci >> 4) != 0 || len == 0 || len > bytes.size() - 1)
      {
      out.push_back("?");
      return;
      }
    bytes = bytes.substr(1, len);
    }

  uint8_t sid = (uint8_t)bytes[0];
  size_t pidlen = PidLength(sid);
  if (bytes.size() < 1 + pidlen)
    {
    // The poller needs the complete PID
    out.push_back("?");
    return;
    }

  uint32_t txid, rxid;
  GetIds(txid, rxid);
  uint32_t rxid_display = rxid ? rxid : (IsExt29() ? OBD_RESP_ID_29 : OBD_RESP_ID_11);
  m_stat_requests++;

  if (!IsServiceAllowed(sid))
    {
    // Simulate a negative response "serviceNotSupported"
    m_stat_blocked++;
    std::string nrc;
    nrc += (char)0x7f;
    nrc += (char)sid;
    nrc += (char)0x11;
    FormatResponse(rxid_display, nrc, out);
    return;
    }

  if (!m_transport)
    {
    m_stat_errors++;
    out.push_back("UNABLE TO CONNECT");
    return;
    }

  int timeout_ms = m_timeout * 4;
  if (timeout_ms < m_min_timeout_ms)
    timeout_ms = m_min_timeout_ms;

  std::string response;
  uint32_t rxid_used = 0;
  int rc = m_transport->Request(txid, rxid, IsExt29(), bytes, response, timeout_ms, rxid_used);
  if (rxid_used)
    rxid_display = rxid_used;

  if (rc == ELM327_REQ_OK)
    {
    std::string full;
    full += (char)(sid + 0x40);
    full += bytes.substr(1, pidlen);
    full += response;
    FormatResponse(rxid_display, full, out);
    }
  else if (rc > 0)
    {
    std::string nrc;
    nrc += (char)0x7f;
    nrc += (char)sid;
    nrc += (char)rc;
    FormatResponse(rxid_display, nrc, out);
    }
  else
    {
    m_stat_errors++;
    switch (rc)
      {
      case ELM327_REQ_TXFAILURE:
        out.push_back("CAN ERROR");
        break;
      case ELM327_REQ_UNAVAILABLE:
        out.push_back("UNABLE TO CONNECT");
        break;
      default:
        out.push_back("NO DATA");
        break;
      }
    }
  }

std::string Elm327Engine::FormatBytes(const uint8_t* data, size_t len) const
  {
  std::string s;
  char buf[4];
  for (size_t i = 0; i < len; i++)
    {
    if (i > 0 && m_spaces)
      s += ' ';
    snprintf(buf, sizeof(buf), "%02X", data[i]);
    s += buf;
    }
  return s;
  }

std::string Elm327Engine::FormatHeader(uint32_t id) const
  {
  char buf[16];
  if (!IsExt29())
    {
    snprintf(buf, sizeof(buf), "%03X", (unsigned)(id & 0x7ff));
    return buf;
    }
  uint8_t b[4] = { (uint8_t)(id >> 24), (uint8_t)(id >> 16), (uint8_t)(id >> 8), (uint8_t)id };
  return FormatBytes(b, 4);
  }

void Elm327Engine::FormatResponse(uint32_t rxid, const std::string& data, std::vector<std::string>& out)
  {
  const uint8_t* d = (const uint8_t*)data.data();
  size_t n = data.size();
  char buf[8];
  // Like a real adapter, data lines end with a space if spaces are on:
  std::string sep = m_spaces ? " " : "";

  if (m_caf && !m_headers)
    {
    if (n <= 7)
      {
      out.push_back(FormatBytes(d, n) + sep);
      return;
      }
    // Multi frame response: length line, then numbered frame lines
    snprintf(buf, sizeof(buf), "%03X", (unsigned)(n & 0xfff));
    out.push_back(buf);
    size_t pos = 0;
    for (unsigned idx = 0; pos < n; idx++)
      {
      size_t chunk = (idx == 0) ? 6 : 7;
      if (chunk > n - pos)
        chunk = n - pos;
      snprintf(buf, sizeof(buf), "%X: ", idx & 0x0f);
      out.push_back(buf + FormatBytes(d + pos, chunk) + sep);
      pos += chunk;
      }
    return;
    }

  // Headers on or raw mode: show the CAN frames including the ISO-TP PCI bytes
  std::string prefix;
  if (m_headers)
    {
    prefix = FormatHeader(rxid);
    if (m_spaces)
      prefix += ' ';
    }

  uint8_t frame[8];
  if (n <= 7)
    {
    frame[0] = (uint8_t)n;
    memcpy(frame + 1, d, n);
    out.push_back(prefix + FormatBytes(frame, n + 1) + sep);
    return;
    }

  frame[0] = 0x10 | (uint8_t)((n >> 8) & 0x0f);
  frame[1] = (uint8_t)(n & 0xff);
  memcpy(frame + 2, d, 6);
  out.push_back(prefix + FormatBytes(frame, 8) + sep);
  size_t pos = 6;
  for (unsigned seq = 1; pos < n; seq++)
    {
    size_t chunk = (n - pos > 7) ? 7 : n - pos;
    frame[0] = 0x20 | (uint8_t)(seq & 0x0f);
    memcpy(frame + 1, d + pos, chunk);
    out.push_back(prefix + FormatBytes(frame, chunk + 1) + sep);
    pos += chunk;
    }
  }
