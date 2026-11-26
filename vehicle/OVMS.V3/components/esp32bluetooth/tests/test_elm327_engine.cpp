// test_elm327_engine.cpp — Native laptop tests for the ELM327 command interpreter.
//
// A mock transport records each request and returns a configured response,
// so the tests check the AT command handling, the CAN ID routing and the
// ELM327 response formatting without hardware.
//
// Run:  make   (from the tests/ directory)

#include "elm327_engine.h"

#include <cstdio>
#include <string>

int tests_run = 0;
int tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else       { printf("  FAIL: %s\n", msg); } \
} while(0)

#define CHECK_EQ(actual, expected, msg) do { \
    std::string a_ = (actual), e_ = (expected); \
    tests_run++; \
    if (a_ == e_) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n    expected: \"%s\"\n    actual:   \"%s\"\n", \
                  msg, escape(e_).c_str(), escape(a_).c_str()); } \
} while(0)

static std::string escape(const std::string& s) {
    std::string r;
    for (char c : s) {
        if (c == '\r') r += "\\r";
        else if (c == '\n') r += "\\n";
        else r += c;
    }
    return r;
}

static std::string bin(std::initializer_list<int> bytes) {
    std::string s;
    for (int b : bytes) s += (char)b;
    return s;
}

class MockTransport : public Elm327Engine::Transport {
public:
    // last request:
    int calls = 0;
    uint32_t txid = 0, rxid = 0;
    bool ext29 = false;
    std::string request;
    int timeout_ms = 0;
    // configured result:
    int rc = ELM327_REQ_OK;
    std::string response;
    uint32_t rxid_used = 0;
    bool volt_ok = true;
    float volts = 12.62f;

    int Request(uint32_t t, uint32_t r, bool e, const std::string& req,
                std::string& resp, int to, uint32_t& used) override {
        calls++;
        txid = t; rxid = r; ext29 = e; request = req; timeout_ms = to;
        resp = response;
        used = rxid_used;
        return rc;
    }
    bool GetVoltage(float& v) override {
        v = volts;
        return volt_ok;
    }
    // request statistics:
    Elm327RequestStats stats;
    Elm327RequestStats::Entry table[3];
    uint32_t now = 0;
    MockTransport() { stats.Init(table, 3); }
    void RequestDone(uint32_t t, uint8_t sid, uint16_t pid, int pidlen,
                     int r, size_t len) override {
        stats.Record(t, sid, pid, pidlen, r, len, now);
    }
};

// Helper: engine with echo off, as most clients configure it
static void quiet(Elm327Engine& e) {
    e.ProcessLine("ATE0");
}

// ---------------------------------------------------------------------------

void test_reset_and_echo() {
    printf("\ntest_reset_and_echo\n");
    MockTransport t;
    Elm327Engine e(&t);
    CHECK_EQ(e.ProcessLine("ATZ"), "ATZ\r\r\rELM327 v1.5\r\r>", "ATZ with echo");
    CHECK_EQ(e.ProcessLine("ATE0"), "ATE0\rOK\r\r>", "ATE0 is echoed, then echo off");
    CHECK_EQ(e.ProcessLine("ATI"), "ELM327 v1.5\r\r>", "ATI without echo");
    CHECK_EQ(e.ProcessLine("atl1"), "OK\r\n\r\n>", "lower case ATL1, response uses CR LF");
    CHECK_EQ(e.ProcessLine("ATD"), "OK\r\r>", "ATD restores defaults (linefeeds off)");
    CHECK_EQ(e.ProcessLine("ATXYZ"), "ATXYZ\r?\r\r>", "unknown AT command, echo on again after ATD");
    e.ProcessLine("ATL1");
    CHECK_EQ(e.ProcessLine("ATI"), "ATI\rELM327 v1.5\r\n\r\n>", "echo is the raw input with CR, also with ATL1");
}

void test_input_buffering() {
    printf("\ntest_input_buffering\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    CHECK_EQ(e.Input("AT", 2), "", "partial line gives no output");
    CHECK_EQ(e.Input("I\r", 2), "ELM327 v1.5\r\r>", "line completed by second chunk");
    CHECK_EQ(e.Input("ATS0\r\nATH1\r", 11), "OK\r\r>OK\r\r>", "two commands in one chunk, LF ignored");
}

void test_info_commands() {
    printf("\ntest_info_commands\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    CHECK_EQ(e.ProcessLine("ATRV"), "12.6V\r\r>", "ATRV voltage");
    t.volt_ok = false;
    CHECK_EQ(e.ProcessLine("ATRV"), "0.0V\r\r>", "ATRV without voltage");
    CHECK_EQ(e.ProcessLine("ATDP"), "AUTO, ISO 15765-4 (CAN 11/500)\r\r>", "ATDP auto");
    CHECK_EQ(e.ProcessLine("ATDPN"), "A6\r\r>", "ATDPN auto");
    CHECK_EQ(e.ProcessLine("ATSP7"), "OK\r\r>", "ATSP7");
    CHECK_EQ(e.ProcessLine("ATDP"), "ISO 15765-4 (CAN 29/500)\r\r>", "ATDP fixed 29 bit");
    CHECK_EQ(e.ProcessLine("ATDPN"), "7\r\r>", "ATDPN fixed");
    CHECK_EQ(e.ProcessLine("ATSPZ"), "?\r\r>", "invalid protocol");
    CHECK_EQ(e.ProcessLine("ATMA"), "?\r\r>", "monitor all is not supported");
    CHECK_EQ(e.ProcessLine("STDI"), "?\r\r>", "STN command is not supported");
    CHECK_EQ(e.ProcessLine("AT AT 1"), "OK\r\r>", "ATAT1 with spaces is accepted");
}

void test_single_frame_request() {
    printf("\ntest_single_frame_request\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    t.response = bin({0xBE, 0x3F, 0xA8, 0x13});
    CHECK_EQ(e.ProcessLine("0100"), "41 00 BE 3F A8 13 \r\r>", "mode 01 PID 00, headers off");
    CHECK(t.txid == 0x7df && t.rxid == 0 && !t.ext29, "broadcast 7DF, rxid 0, 11 bit");
    CHECK(t.request == bin({0x01, 0x00}), "request bytes 01 00");
    CHECK(t.timeout_ms == 500, "timeout: minimum 500 ms (ATST default 200 ms)");

    CHECK_EQ(e.ProcessLine("01 00 1"), "41 00 BE 3F A8 13 \r\r>", "spaces and response count digit");
    CHECK(t.request == bin({0x01, 0x00}), "response count digit is removed");

    CHECK_EQ(e.ProcessLine(""), "41 00 BE 3F A8 13 \r\r>", "empty line repeats last command");
    CHECK(t.calls == 3, "repeat sends request again");

    e.ProcessLine("ATH1");
    CHECK_EQ(e.ProcessLine("0100"), "7E8 06 41 00 BE 3F A8 13 \r\r>", "headers on: ID and PCI byte");
    e.ProcessLine("ATS0");
    CHECK_EQ(e.ProcessLine("0100"), "7E806410" "0BE3FA813\r\r>", "headers on, spaces off");
}

void test_header_routing() {
    printf("\ntest_header_routing\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    t.response = bin({0x01, 0x02});

    e.ProcessLine("ATSH7E4");
    e.ProcessLine("220101");
    CHECK(t.txid == 0x7e4 && t.rxid == 0x7ec, "ATSH 7E4 -> rxid 7EC");
    CHECK(t.request == bin({0x22, 0x01, 0x01}), "UDS 22 0101 request");

    e.ProcessLine("ATCRA 7BB");
    e.ProcessLine("ATSH 79B");
    e.ProcessLine("2101");
    CHECK(t.txid == 0x79b && t.rxid == 0x7bb, "ATCRA sets rxid");

    e.ProcessLine("ATCRA");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7a3, "ATCRA without argument resets the filter");

    e.ProcessLine("ATCRA 7AX");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7a3, "ATCRA wildcard passing the default rxid keeps it");
    e.ProcessLine("ATCRA 7EX");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7e0, "ATCRA wildcard not passing the default rxid: lowest ID");
    e.ProcessLine("ATAR");

    e.ProcessLine("ATST 64");
    e.ProcessLine("2101");
    CHECK(t.timeout_ms == 500, "ATST 64 (400 ms) is raised to the 500 ms minimum");
    e.ProcessLine("ATST C8");
    e.ProcessLine("2101");
    CHECK(t.timeout_ms == 800, "ATST C8 -> 200 x 4 ms");
}

void test_can_filter() {
    printf("\ntest_can_filter\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    t.response = bin({0x01});

    e.ProcessLine("ATSH 7E0");
    CHECK_EQ(e.ProcessLine("ATCF 7EC"), "OK\r\r>", "ATCF accepted");
    e.ProcessLine("2101");
    CHECK(t.txid == 0x7e0 && t.rxid == 0x7ec, "ATCF without ATCM: exact filter");

    CHECK_EQ(e.ProcessLine("ATCM 7FF"), "OK\r\r>", "ATCM accepted");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7ec, "ATCF + ATCM 7FF: exact filter");

    e.ProcessLine("ATCF 7E8");
    e.ProcessLine("ATCM 7F0");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7e8, "partial mask passing the default rxid keeps it");

    e.ProcessLine("ATSH 7DF");
    e.ProcessLine("ATCF 7E8");
    e.ProcessLine("ATCM 7F8");
    e.ProcessLine("0100");
    CHECK(t.rxid == 0, "broadcast with a filter passing 7E8…7EF stays a broadcast");
    e.ProcessLine("ATCM 7FF");
    e.ProcessLine("ATCF 7EC");
    e.ProcessLine("0100");
    CHECK(t.rxid == 0x7ec, "broadcast with an exact filter listens to that ID");

    e.ProcessLine("ATAR");
    e.ProcessLine("ATSH 7E0");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7e8, "ATAR resets the filter");

    CHECK_EQ(e.ProcessLine("ATCF 7EX"), "?\r\r>", "ATCF does not accept wildcards");
    CHECK_EQ(e.ProcessLine("ATCFC1"), "OK\r\r>", "ATCFC1 (flow control) is not a filter");
    e.ProcessLine("ATCRA XXX");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7e8, "ATCRA XXX passes all IDs: default rxid");
    e.ProcessLine("ATCF 7EC");
    e.ProcessLine("ATCM 000");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7e8, "ATCM 000 passes all IDs: default rxid");
    e.ProcessLine("ATAR");
    CHECK_EQ(e.ProcessLine("ATCM 7F"), "?\r\r>", "ATCM needs 3 or 8 digits");

    e.ProcessLine("ATSP7");
    e.ProcessLine("ATSH DA10F1");
    e.ProcessLine("ATCF 18DAF110");
    e.ProcessLine("ATCM 1FFFFFFF");
    e.ProcessLine("0100");
    CHECK(t.ext29 && t.rxid == 0x18daf110, "29 bit filter");
    e.ProcessLine("ATD");
    e.ProcessLine("ATE0");
    e.ProcessLine("ATSH 7E4");
    e.ProcessLine("2101");
    CHECK(t.rxid == 0x7ec, "ATD resets the filter");
}

void test_29bit_routing() {
    printf("\ntest_29bit_routing\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    t.response = bin({0x12});

    e.ProcessLine("ATSP7");
    e.ProcessLine("0100");
    CHECK(t.txid == 0x18db33f1 && t.rxid == 0x18daf110 && t.ext29, "29 bit functional request");

    e.ProcessLine("ATSH DA10F1");
    e.ProcessLine("0100");
    CHECK(t.txid == 0x18da10f1 && t.rxid == 0x18daf110, "6 digit header uses priority 18");

    e.ProcessLine("ATCP 17");
    e.ProcessLine("ATSH FC007B");
    e.ProcessLine("ATCRA 17FE007B");
    e.ProcessLine("22028C");
    CHECK(t.txid == 0x17fc007b && t.rxid == 0x17fe007b, "VW style 29 bit IDs with ATCP and ATCRA");

    e.ProcessLine("ATH1");
    t.rxid_used = 0x17fe007b;
    CHECK_EQ(e.ProcessLine("22028C"), "17 FE 00 7B 04 62 02 8C 12 \r\r>", "29 bit header display");
}

void test_auto_protocol_29bit_header() {
    printf("\ntest_auto_protocol_29bit_header\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    t.response = bin({0x12});
    e.ProcessLine("ATSH 18DA44F1");
    e.ProcessLine("221E3B");
    CHECK(t.ext29 && t.txid == 0x18da44f1 && t.rxid == 0x18daf144, "auto protocol uses 29 bit for 8 digit header");
    CHECK_EQ(e.ProcessLine("ATDPN"), "A7\r\r>", "ATDPN shows 29 bit variant");
}

void test_multi_frame_response() {
    printf("\ntest_multi_frame_response\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    // 09 02 VIN response: 49 02 01 + 17 characters = 20 bytes
    t.response = std::string("\x01" "1D4GP00R55B123456", 18);
    CHECK_EQ(e.ProcessLine("0902"),
             "014\r"
             "0: 49 02 01 31 44 34 \r"
             "1: 47 50 30 30 52 35 35 \r"
             "2: 42 31 32 33 34 35 36 \r\r>",
             "multi frame, headers off");

    e.ProcessLine("ATH1");
    CHECK_EQ(e.ProcessLine("0902"),
             "7E8 10 14 49 02 01 31 44 34 \r"
             "7E8 21 47 50 30 30 52 35 35 \r"
             "7E8 22 42 31 32 33 34 35 36 \r\r>",
             "multi frame, headers on");

    e.ProcessLine("ATH0");
    e.ProcessLine("ATS0");
    t.response = std::string(10, '\xAA');   // 62 01 01 + 10 bytes = 13 bytes
    CHECK_EQ(e.ProcessLine("ATSH7E4"), "OK\r\r>", "set header");
    CHECK_EQ(e.ProcessLine("220101"),
             "00D\r"
             "0: 620101AAAAAA\r"
             "1: AAAAAAAAAAAAAA\r\r>",
             "multi frame, spaces off, short last frame");
}

void test_negative_and_errors() {
    printf("\ntest_negative_and_errors\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    e.ProcessLine("ATSH7E4");

    t.rc = 0x31;
    CHECK_EQ(e.ProcessLine("22F190"), "7F 22 31 \r\r>", "NRC requestOutOfRange");

    t.rc = ELM327_REQ_TIMEOUT;
    CHECK_EQ(e.ProcessLine("220101"), "NO DATA\r\r>", "timeout");
    t.rc = ELM327_REQ_NORESPONSE;
    CHECK_EQ(e.ProcessLine("220101"), "NO DATA\r\r>", "no response");
    t.rc = ELM327_REQ_TXFAILURE;
    CHECK_EQ(e.ProcessLine("220101"), "CAN ERROR\r\r>", "transmit failure");
    t.rc = ELM327_REQ_UNAVAILABLE;
    CHECK_EQ(e.ProcessLine("220101"), "UNABLE TO CONNECT\r\r>", "bus not available");

    int calls = t.calls;
    CHECK_EQ(e.ProcessLine("22"), "?\r\r>", "UDS 22 without DID is rejected");
    CHECK_EQ(e.ProcessLine("01G0"), "?\r\r>", "invalid hex digit");
    CHECK(t.calls == calls, "invalid requests do not reach the transport");
}

void test_service_allowlist() {
    printf("\ntest_service_allowlist\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    e.ProcessLine("ATSH7E0");

    CHECK_EQ(e.ProcessLine("1101"), "7F 11 11 \r\r>", "ECU reset is blocked by default");
    CHECK_EQ(e.ProcessLine("2E010203"), "7F 2E 11 \r\r>", "write data is blocked by default");
    CHECK_EQ(e.ProcessLine("04"), "7F 04 11 \r\r>", "clear DTC is blocked by default");
    CHECK(t.calls == 0, "blocked requests do not reach the transport");
    CHECK(e.m_stat_blocked == 3, "blocked counter");

    CHECK(!e.SetAllowedServices("01,XYZ"), "invalid list is rejected");
    CHECK(!e.IsServiceAllowed(0x11), "invalid list keeps the old setting");
    CHECK(e.SetAllowedServices("*"), "allow all");
    CHECK(e.IsServiceAllowed(0x11) && e.IsServiceAllowed(0x2e), "all services allowed");
    CHECK(e.SetAllowedServices("22"), "allow only 22");
    CHECK(e.IsServiceAllowed(0x22) && !e.IsServiceAllowed(0x01), "only 22 allowed");
}

void test_request_stats() {
    printf("\ntest_request_stats\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    e.ProcessLine("ATSH7E4");

    t.now = 10;
    t.response = bin({0x01, 0x02, 0x03});
    e.ProcessLine("220320");
    t.now = 11;
    e.ProcessLine("220320");
    t.rc = 0x31;
    e.ProcessLine("22200E");
    t.rc = ELM327_REQ_TIMEOUT;
    e.ProcessLine("22200E");
    CHECK(t.stats.Count() == 2, "one entry per request");
    const Elm327RequestStats::Entry& a = t.stats.Get(0);
    CHECK(a.txid == 0x7e4 && a.sid == 0x22 && a.pid == 0x0320 && a.pidlen == 2, "entry key");
    CHECK(a.count == 2 && a.ok == 2 && a.nrc == 0 && a.errors == 0, "OK counters");
    CHECK(a.last_rc == ELM327_REQ_OK && a.last_len == 3 && a.last_time == 11, "last result");
    const Elm327RequestStats::Entry& b = t.stats.Get(1);
    CHECK(b.count == 2 && b.ok == 0 && b.nrc == 1 && b.errors == 1, "NRC and error counters");
    CHECK(b.last_rc == ELM327_REQ_TIMEOUT && b.last_len == 0, "last error result");

    e.ProcessLine("ATSH7DF");
    t.rc = ELM327_REQ_OK;
    e.ProcessLine("010D");
    const Elm327RequestStats::Entry& c = t.stats.Get(2);
    CHECK(c.txid == 0x7df && c.sid == 0x01 && c.pid == 0x0D && c.pidlen == 1, "OBD2 entry");

    e.ProcessLine("1003");
    CHECK(t.stats.Count() == 3 && t.stats.m_unlisted == 1, "full table: request not listed");
    CHECK(t.stats.m_total == 6, "total counter");

    t.stats.Clear(20);
    CHECK(t.stats.Count() == 0 && t.stats.m_total == 0 && t.stats.m_since == 20, "clear");
    e.ProcessLine("1003");
    const Elm327RequestStats::Entry& d = t.stats.Get(0);
    CHECK(d.sid == 0x10 && d.count == 1 && d.errors == 0 && d.last_rc == ELM327_REQ_BLOCKED,
          "blocked request is recorded");

    int calls = t.calls;
    e.ProcessLine("22");
    CHECK(t.stats.m_total == 1 && t.calls == calls, "invalid request is not recorded");
}

void test_raw_mode() {
    printf("\ntest_raw_mode\n");
    MockTransport t;
    Elm327Engine e(&t);
    quiet(e);
    t.response = bin({0x0B, 0xB8});
    e.ProcessLine("ATCAF0");
    CHECK_EQ(e.ProcessLine("02010C"), "04 41 0C 0B B8 \r\r>", "CAF0: PCI byte in request and response");
    CHECK(t.request == bin({0x01, 0x0C}), "PCI byte removed from request");
    CHECK_EQ(e.ProcessLine("05010C"), "?\r\r>", "CAF0: PCI length larger than data");
}

int main() {
    printf("=== ELM327 engine tests ===\n");

    test_reset_and_echo();
    test_input_buffering();
    test_info_commands();
    test_single_frame_request();
    test_header_routing();
    test_can_filter();
    test_29bit_routing();
    test_auto_protocol_29bit_header();
    test_multi_frame_response();
    test_negative_and_errors();
    test_service_allowlist();
    test_raw_mode();
    test_request_stats();

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
