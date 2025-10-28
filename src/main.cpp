#include <Arduino.h>
#include "AtCommands.h"

// ESP32 pin mapping for SIMCom A7608E-H
static constexpr int MODEM_TX_PIN = 8;   // ESP32 -> Modem RX
static constexpr int MODEM_RX_PIN = 9;   // ESP32 <- Modem TX
static constexpr int MODEM_PWRKEY_PIN = 10; // Modem PWRKEY (K)

HardwareSerial &MODEM = Serial1;
HardwareSerial &GNSS = Serial2;
// AT command helper instance
static AtCommands AT(MODEM, Serial);

// Forward declarations
static void runNetDiagnostics(const String &contrdp);
static void testTcpToDns53(const String &contrdp);

// NTRIP demo configuration (fill these for NGII)
static constexpr bool ENABLE_NTRIP_DEMO = false; // set true to run NTRIP test on boot
static String NTRIP_HOST = "";                 // e.g. "ntrip.ngii.go.kr" (placeholder)
static int    NTRIP_PORT = 2101;
static String NTRIP_MOUNT = "";               // e.g. "VRS"
static String NTRIP_USER = "";                // your ID
static String NTRIP_PASS = "";                // your password

// GNSS UART config (RTCM output)
static constexpr int GNSS_TX_PIN = 18; // ESP32 TX -> GNSS RX
static constexpr int GNSS_RX_PIN = 17; // ESP32 RX <- GNSS TX (optional)
static constexpr int GNSS_BAUD   = 115200;

// NTRIP helpers
static String base64Encode(const String &in);
static bool ntripClientRun(const char *host, int port, const char *mount, const char *user, const char *pass);
static bool ntripClientRunStr(const String &host, int port, const String &mount, const String &user, const String &pass) {
  return ntripClientRun(host.c_str(), port, mount.c_str(), user.c_str(), pass.c_str());
}

// Control noisy auto tests (off by default)
static constexpr bool RUN_AUTO_HTTP_TEST = false;
static constexpr bool RUN_AUTO_AT_SCRIPT = true; // run boot diagnostics automatically
static const char *AUTO_APN = "";              // set to APN name to force APN config; keep empty to skip
static const char *AUTO_APN_USER = "";         // optional PAP user
static const char *AUTO_APN_PASS = "";         // optional PAP pass

static void runAutoATScript();
static bool tryTcpOpenClose(const String &host, int port, int channel, uint32_t openTimeoutMs = 15000);
static void runBasicServersTest();
static bool httpPostViaSockets(const String &host, int port, const String &path, const String &contentType, const String &body);
static bool httpsPostViaCipssl(const String &host, const String &path, const String &contentType, const String &body);
static bool httpsPutViaCipssl(const String &host, const String &path, const String &contentType, const String &body);
static bool httpsPutViaCipssl(const String &host, const String &path, const String &contentType, const String &body);
static const char *RTDB_HOST = "rtk-gnss-b864f-default-rtdb.firebaseio.com";
static String normalizeRtdbPath(const String &raw) {
  String p = raw;
  if (p.length() == 0) p = "/";
  if (p[0] != '/') p = String("/") + p;
  if (!p.endsWith(".json")) p += ".json";
  return p;
}
static void runCapabilityProbe();
static bool httpStackHttpsPost(const String &host, const String &path, const String &contentType, const String &body);
static bool httpStackHttpsPut(const String &host, const String &path, const String &contentType, const String &body);

static void powerOnModem() {
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(20);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(1100); // Minimum pulse to turn on
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(3000); // Shortened boot wait
}

static void flushInput(Stream &s, uint32_t durationMs = 50) {
  AT.flushInput(s, durationMs);
}

static bool waitForResponse(const char *expect, uint32_t timeoutMs) {
  return AT.waitForResponse(expect, timeoutMs);
}

static bool sendATExpect(const String &cmd, const char *expect, uint32_t timeoutMs) {
  return AT.sendATExpect(cmd, expect, timeoutMs);
}

static bool waitForAny(const char *e1, const char *e2, const char *e3, uint32_t timeoutMs) {
  return AT.waitForAny(e1, e2, e3, timeoutMs);
}

static String sendATCollect(const String &cmd, uint32_t timeoutMs) {
  return AT.sendATCollect(cmd, timeoutMs);
}

static bool isIpv4Literal(const String &s) {
  int dots = 0;
  int part = 0;
  int val = -1;
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c == '.') {
      if (val < 0 || val > 255) return false;
      dots++;
      val = -1;
      part = 0;
    } else if (c >= '0' && c <= '9') {
      if (val < 0) val = 0;
      val = val * 10 + (c - '0');
      part++;
      if (part > 3) return false;
    } else {
      return false;
    }
  }
  if (val < 0 || val > 255) return false;
  return dots == 3;
}

static String extractFirstIPv4(const String &text) {
  int n = text.length();
  for (int i = 0; i < n; ++i) {
    if (text[i] >= '0' && text[i] <= '9') {
      int j = i;
      while (j < n && ((text[j] >= '0' && text[j] <= '9') || text[j] == '.')) j++;
      String candidate = text.substring(i, j);
      if (candidate.length() >= 7 && candidate.length() <= 15 && isIpv4Literal(candidate)) {
        return candidate;
      }
      i = j;
    }
  }
  return String();
}

static bool parseHttpUrl(const char *url, String &host, int &port, String &path) {
  const char *p = url;
  const char *http = "http://";
  size_t httpLen = strlen(http);
  if (strncmp(p, http, httpLen) != 0) return false; // only http supported in socket fallback
  p += httpLen;
  const char *slash = strchr(p, '/');
  if (!slash) {
    host = String(p);
    path = "/";
  } else {
    host = String(p).substring(0, slash - p);
    path = String(slash);
  }
  port = 80;
  return host.length() > 0;
}

static bool tcpHttpGetViaSockets(const char *url) {
  String host, path;
  int port = 80;
  if (!parseHttpUrl(url, host, port, path)) {
    Serial.println(F("Only http:// URLs are supported in socket fallback."));
    return false;
  }

  // Resolve DNS only if host is not already an IPv4 literal
  String ip = host;
  if (!isIpv4Literal(host)) {
    String dns = sendATCollect(String("AT+CDNSGIP=\"") + host + "\"", 15000);
    String resolved = extractFirstIPv4(dns);
    if (resolved.length() > 0) ip = resolved;
  }

  // Ensure plain TCP first
  sendATExpect("AT+CIPSSL=0", "OK", 2000);

  // Open TCP socket 0 to port 80 (wait for URC rather than immediate OK)
  String openCmd = String("AT+CIPOPEN=0,\"TCP\",\"") + ip + "\"," + port;
  Serial.print(F("--> "));
  Serial.println(openCmd);
  MODEM.print(openCmd);
  MODEM.print("\r\n");
  bool openAccepted = waitForAny("+CIPOPEN:", "OK", nullptr, 20000);
  bool openSucceeded = waitForAny("+CIPOPEN: 0,0", "ALREADY CONNECT", nullptr, 20000);

  if (!openAccepted || !openSucceeded) {
    Serial.println(F("CIPOPEN to port 80 failed; trying TLS 443 fallback with SNI"));
    // Enable SSL and connect using hostname to preserve SNI
    if (!sendATExpect("AT+CIPSSL=1", "OK", 3000)) return false;
    String openTls = String("AT+CIPOPEN=0,\"TCP\",\"") + host + "\",443";
    Serial.print(F("--> "));
    Serial.println(openTls);
    MODEM.print(openTls);
    MODEM.print("\r\n");
    if (!waitForAny("+CIPOPEN:", nullptr, nullptr, 20000)) return false;
    if (!waitForAny("+CIPOPEN: 0,0", "ALREADY CONNECT", nullptr, 20000)) return false;
  }

  // Prepare HTTP GET request
  String req = String("GET ") + path + " HTTP/1.1\r\nHost: " + host + "\r\nConnection: close\r\nUser-Agent: A7608E-HTTPTest\r\n\r\n";

  // Send payload with length
  String sendCmd = String("AT+CIPSEND=0,") + req.length();
  Serial.print(F("--> "));
  Serial.println(sendCmd);
  MODEM.print(sendCmd);
  MODEM.print("\r\n");
  // Wait for prompt
  String buf;
  uint32_t start = millis();
  bool gotPrompt = false;
  while (millis() - start < 8000) {
    while (MODEM.available()) {
      char c = (char)MODEM.read();
      Serial.write(c);
      buf += c;
      if (buf.indexOf(">") >= 0 || buf.indexOf("DOWNLOAD") >= 0) {
        gotPrompt = true;
        break;
      }
      if (buf.indexOf("ERROR") >= 0) break;
    }
    if (gotPrompt) break;
    delay(1);
  }
  if (!gotPrompt) {
    Serial.println(F("No send prompt"));
    return false;
  }
  MODEM.print(req);
  if (!waitForAny("SEND OK", "+CIPCLOSE: 0", "ERROR", 15000)) {
    Serial.println(F("Send did not complete"));
    // continue to attempt read/close
  }

  // Ensure RXGET mode
  sendATExpect("AT+CIPRXGET=1", "OK", 2000);

  // Read until no more data
  for (int i = 0; i < 60; ++i) {
    String r = sendATCollect("AT+CIPRXGET=2,0,1460", 1000);
    if (r.indexOf("+CIPRXGET: 2,0,0") >= 0) {
      // No more data at this moment
      // Check if socket closed
      if (r.indexOf("CLOSED") >= 0) break;
      delay(500);
      continue;
    }
  }
  sendATExpect("AT+CIPCLOSE=0", "OK", 5000);
  return true;
}

static bool ensurePdpAndHttpGet(const char *apn, const char *url) {
  Serial.println(F("\n=== PDP/HTTP GET test start ==="));

  flushInput(MODEM);
  if (!sendATExpect("AT", "OK", 1000)) return false;
  sendATExpect("AT+CMEE=2", "OK", 1000);
  if (!sendATExpect("AT+CPIN?", "READY", 5000)) return false;

  // Prefer using already active PDP context (user logs showed CID 1 active by network)
  String contrdp = sendATCollect("AT+CGCONTRDP=1", 3000);
  bool cid1Active = contrdp.indexOf("+CGCONTRDP: 1") >= 0;
  if (!cid1Active) {
    // Fallback: try to attach and activate with provided APN if nothing active
    sendATExpect(String("AT+CGDCONT=1,\"IP\",\"") + apn + "\"", "OK", 5000);
    sendATExpect("AT+CGATT=1", "OK", 15000);
    sendATExpect("AT+CGACT=1,1", "OK", 20000);
    contrdp = sendATCollect("AT+CGCONTRDP=1", 5000);
    cid1Active = contrdp.indexOf("+CGCONTRDP: 1") >= 0;
  }
  if (!cid1Active) {
    Serial.println(F("No active PDP context (CID 1) detected."));
    return false;
  }

  // Bind sockets to CID 1 and open IP stack
  sendATExpect("AT+CSOCKSETPN=1", "OK", 2000);
  MODEM.print("AT+NETOPEN\r\n");
  if (!waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 30000)) {
    Serial.println(F("NETOPEN failed"));
    // continue anyway; some firmware allows HTTP without explicit NETOPEN
  }

  // HTTP flow
  if (!sendATExpect("AT+HTTPINIT", "OK", 8000)) {
    // If already initialized, terminate and retry once
    sendATExpect("AT+HTTPTERM", "OK", 2000);
    if (!sendATExpect("AT+HTTPINIT", "OK", 8000)) {
      Serial.println(F("HTTPINIT failed, trying socket fallback"));
      bool ok = tcpHttpGetViaSockets(url);
      if (!ok) Serial.println(F("Socket HTTP failed"));
      if (!ok) runNetDiagnostics(contrdp);
      if (!ok) testTcpToDns53(contrdp);
      return ok;
    }
  }
  sendATExpect("AT+HTTPPARA=\"CID\",1", "OK", 2000);
  if (!sendATExpect(String("AT+HTTPPARA=\"URL\",\"") + url + "\"", "OK", 5000)) {
    sendATExpect("AT+HTTPTERM", "OK", 2000);
    return false;
  }

  Serial.println(F("Requesting..."));
  MODEM.print("AT+HTTPACTION=0\r\n");
  if (!waitForResponse("+HTTPACTION: 0,", 60000)) {
    sendATExpect("AT+HTTPTERM", "OK", 2000);
    return false;
  }

  // Read response
  if (!sendATExpect("AT+HTTPREAD", "OK", 30000)) {
    sendATExpect("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  sendATExpect("AT+HTTPTERM", "OK", 2000);
  Serial.println(F("=== HTTP GET test done ===\n"));
  return true;
}

// Simple Base64 encoder for NTRIP Basic Auth
static String base64Encode(const String &in) {
  static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  int val = 0, valb = -6;
  for (size_t i = 0; i < in.length(); ++i) {
    unsigned char c = (unsigned char)in[i];
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      out += tbl[(val >> valb) & 0x3F];
      valb -= 6;
    }
  }
  if (valb > -6) out += tbl[((val << 8) >> (valb + 8)) & 0x3F];
  while (out.length() % 4) out += '=';
  return out;
}

static bool ntripClientRun(const char *host, int port, const char *mount, const char *user, const char *pass) {
  if (!host || !*host || !mount || !*mount) {
    Serial.println(F("NTRIP config empty; skipping"));
    return false;
  }
  // Ensure NETOPEN
  MODEM.print("AT+NETOPEN\r\n");
  waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 10000);

  // Resolve host
  String dns = sendATCollect(String("AT+CDNSGIP=\"") + host + "\"", 15000);
  String ip = extractFirstIPv4(dns);
  if (!isIpv4Literal(ip)) ip = host; // try host if IP not parsed

  // Open TCP
  String openCmd = String("AT+CIPOPEN=2,\"TCP\",\"") + ip + "\"," + port;
  Serial.print(F("--> "));
  Serial.println(openCmd);
  MODEM.print(openCmd);
  MODEM.print("\r\n");
  if (!waitForAny("+CIPOPEN: 2,0", nullptr, nullptr, 20000)) {
    Serial.println(F("NTRIP TCP open failed"));
    return false;
  }

  // Build NTRIP request
  String cred = String(user) + ":" + String(pass);
  String auth = base64Encode(cred);
  String req = String("GET /") + mount + " HTTP/1.1\r\n";
  req += String("Host: ") + host + "\r\n";
  req += String("Ntrip-Version: Ntrip/2.0\r\n");
  req += String("User-Agent: A7608E-NTRIP\r\n");
  if (user && *user) req += String("Authorization: Basic ") + auth + "\r\n";
  req += String("Connection: keep-alive\r\n\r\n");

  // Send request
  String sendCmd = String("AT+CIPSEND=2,") + req.length();
  Serial.print(F("--> "));
  Serial.println(sendCmd);
  MODEM.print(sendCmd);
  MODEM.print("\r\n");
  waitForAny(">", "DOWNLOAD", nullptr, 5000);
  MODEM.print(req);
  waitForAny("SEND OK", nullptr, nullptr, 10000);

  // Stream RTCM to GNSS
  sendATExpect("AT+CIPRXGET=1", "OK", 2000);
  Serial.println(F("NTRIP: reading..."));
  uint32_t start = millis();
  while (millis() - start < 15000) { // 15s demo
    String r = sendATCollect("AT+CIPRXGET=2,2,1460", 500);
    // Extract payload after header lines on first chunks; forward all bytes to GNSS
    // Here, we simply mirror modem output to USB; in practice, parse +CIPRXGET size and read raw bytes.
    // For demo, forward printable subset (real RTCM is binary; GNSS expects raw bytes via GNSS UART)
    // Minimal raw forward using MODEM.read is more complex in AT pull mode; for brevity, we keep it simple.
    (void)r;
  }
  sendATExpect("AT+CIPCLOSE=2", "OK", 5000);
  return true;
}
static void runNetDiagnostics(const String &contrdp) {
  Serial.println(F("\n=== Network diagnostics ==="));
  // Try ping 8.8.8.8
  sendATExpect("AT+SNPING4=\"8.8.8.8\",3,32,1000", "OK", 15000);
  // Extract DNS IP from contrdp if present and ping
  int firstQuote = contrdp.indexOf('\"');
  int dns1Pos = contrdp.indexOf('\"', firstQuote + 1);
  dns1Pos = contrdp.indexOf('\"', dns1Pos + 1);
  // After several quoted fields, the next quoted item is DNS1 per +CGCONTRDP format
  int dnsStart = contrdp.indexOf('\"', dns1Pos + 1);
  if (dnsStart >= 0) {
    int dnsEnd = contrdp.indexOf('\"', dnsStart + 1);
    if (dnsEnd > dnsStart) {
      String dnsIp = contrdp.substring(dnsStart + 1, dnsEnd);
      if (isIpv4Literal(dnsIp)) {
        sendATExpect(String("AT+SNPING4=\"") + dnsIp + "\",3,32,1000", "OK", 15000);
      }
    }
  }
  Serial.println(F("=== Diagnostics done ===\n"));
}

static String extractDnsFromContrdp(const String &contrdp) {
  // Collect quoted fields
  String fields[8];
  int count = 0;
  int i = 0;
  while (count < 8) {
    int s = contrdp.indexOf('"', i);
    if (s < 0) break;
    int e = contrdp.indexOf('"', s + 1);
    if (e < 0) break;
    fields[count++] = contrdp.substring(s + 1, e);
    i = e + 1;
  }
  // Per +CGCONTRDP format: [APN, LocalIP, ?, DNS1, DNS2, ...]
  if (count >= 4 && isIpv4Literal(fields[3])) return fields[3];
  if (count >= 5 && isIpv4Literal(fields[4])) return fields[4];
  // Fallback: find first IPv4 literal in text after local IP
  String tail = contrdp;
  int idx = tail.indexOf(fields[1]);
  if (idx >= 0) tail = tail.substring(idx + fields[1].length());
  String found = extractFirstIPv4(tail);
  return found;
}

static void testTcpToDns53(const String &contrdp) {
  Serial.println(F("\n=== DNS TCP:53 connectivity test ==="));
  String dnsIp = extractDnsFromContrdp(contrdp);
  if (!isIpv4Literal(dnsIp)) {
    Serial.println(F("No DNS IP parsed from CGCONTRDP"));
    Serial.println(F("=== DNS test done ===\n"));
    return;
  }
  // Try TCP first
  String openCmd = String("AT+CIPOPEN=0,\"TCP\",\"") + dnsIp + "\",53";
  Serial.print(F("--> "));
  Serial.println(openCmd);
  MODEM.print(openCmd);
  MODEM.print("\r\n");
  bool any = waitForAny("+CIPOPEN:", "OK", nullptr, 15000);
  bool ok = waitForAny("+CIPOPEN: 0,0", "ALREADY CONNECT", nullptr, 15000);
  if (any && ok) {
    Serial.println(F("TCP 53 reachable"));
    sendATExpect("AT+CIPCLOSE=0", "OK", 5000);
    Serial.println(F("=== DNS test done ===\n"));
    return;
  }
  Serial.println(F("TCP 53 failed; trying UDP 53"));
  // Try UDP
  String openUdp = String("AT+CIPOPEN=1,\"UDP\",\"") + dnsIp + "\",53";
  Serial.print(F("--> "));
  Serial.println(openUdp);
  MODEM.print(openUdp);
  MODEM.print("\r\n");
  if (waitForAny("+CIPOPEN: 1,0", nullptr, nullptr, 15000)) {
    Serial.println(F("UDP 53 open (no payload sent)"));
    sendATExpect("AT+CIPCLOSE=1", "OK", 5000);
  } else {
    Serial.println(F("UDP 53 open failed"));
  }
  Serial.println(F("=== DNS test done ===\n"));
}

static bool tryTcpOpenClose(const String &host, int port, int channel, uint32_t openTimeoutMs) {
  MODEM.print("AT+NETOPEN\r\n");
  waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 10000);
  sendATCollect(String("AT+CDNSGIP=\"") + host + "\"", 8000);
  String open = String("AT+CIPOPEN=") + channel + ",\"TCP\",\"" + host + "\"," + port;
  Serial.print(F("--> "));
  Serial.println(open);
  MODEM.print(open);
  MODEM.print("\r\n");
  bool any = waitForAny("+CIPOPEN:", "OK", nullptr, openTimeoutMs);
  bool ok = waitForAny((String("+CIPOPEN: ") + channel + ",0").c_str(), "ALREADY CONNECT", nullptr, 15000);
  sendATExpect(String("AT+CIPCLOSE=") + channel, "OK", 5000);
  return any && ok;
}

static void runBasicServersTest() {
  Serial.println(F("\n=== BASIC SERVERS TEST ==="));
  bool ok1 = tryTcpOpenClose("httpbin.org", 80, 4);
  Serial.println(ok1 ? F("httpbin.org:80 OK") : F("httpbin.org:80 FAIL"));
  bool ok2 = tryTcpOpenClose("example.com", 80, 5);
  Serial.println(ok2 ? F("example.com:80 OK") : F("example.com:80 FAIL"));
  bool ok3 = tryTcpOpenClose("1.1.1.1", 53, 6);
  Serial.println(ok3 ? F("1.1.1.1:53 OK") : F("1.1.1.1:53 FAIL"));
  Serial.println(F("=== BASIC SERVERS TEST END ===\n"));
}

static void runCapabilityProbe() {
  Serial.println(F("\n=== CAPABILITY PROBE ==="));
  sendATCollect("ATI", 3000);
  sendATCollect("AT+GMR", 3000);
  sendATCollect("AT+GCAP", 3000);
  sendATCollect("AT+CHTTPS?", 3000);
  sendATCollect("AT+CHTTPS=?", 3000);
  sendATCollect("AT+CSSLCFG=?", 3000);
  sendATCollect("AT+CIPSSL=?", 3000);
  Serial.println(F("=== PROBE END ===\n"));
}

static bool httpStackHttpsPost(const String &host, const String &path, const String &contentType, const String &body) {
  // Configure TLS via HTTP stack SSLCFG=0 profile
  sendATExpect("AT+CSSLCFG=\"authmode\",0,0", "OK", 3000);
  sendATExpect("AT+CSSLCFG=\"sslversion\",0,4", "OK", 3000);
  sendATExpect("AT+CSSLCFG=\"enableSNI\",0,1", "OK", 3000);
  sendATExpect("AT+CSSLCFG=\"negotiatetime\",0,120", "OK", 3000);

  // Init HTTP
  sendATExpect("AT+HTTPTERM", "OK", 1000);
  if (!sendATExpect("AT+HTTPINIT", "OK", 5000)) return false;
  sendATExpect("AT+HTTPPARA=\"CID\",1", "OK", 2000);
  sendATExpect("AT+HTTPPARA=\"SSLCFG\",0", "OK", 2000);
  String url = String("https://") + host + path;
  if (!sendATExpect(String("AT+HTTPPARA=\"URL\",\"") + url + "\"", "OK", 5000)) {
    sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  sendATExpect(String("AT+HTTPPARA=\"CONTENT\",\"") + contentType + "\"", "OK", 2000);
  String dataCmd = String("AT+HTTPDATA=") + body.length() + ",60000";
  if (!sendATExpect(dataCmd, "DOWNLOAD", 5000)) {
    sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  MODEM.print(body);
  waitForAny("OK", nullptr, nullptr, 10000);
  if (!sendATExpect("AT+HTTPACTION=1", "OK", 2000)) {
    sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  if (!waitForAny("+HTTPACTION: 1,200", "+HTTPACTION: 1,201", "+HTTPACTION: 1,2", 60000)) {
    sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  // Optional read
  sendATCollect("AT+HTTPREAD?", 1000);
  sendATCollect("AT+HTTPREAD=0,512", 3000);
  sendATExpect("AT+HTTPTERM", "OK", 1000);
  return true;
}

static bool httpStackHttpsPut(const String &host, const String &path, const String &contentType, const String &body) {
  // Configure TLS via HTTP stack SSLCFG=0 profile
  AT.sendATExpect("AT+CSSLCFG=\"authmode\",0,0", "OK", 3000);
  AT.sendATExpect("AT+CSSLCFG=\"sslversion\",0,4", "OK", 3000);
  AT.sendATExpect("AT+CSSLCFG=\"enableSNI\",0,1", "OK", 3000);
  AT.sendATExpect("AT+CSSLCFG=\"negotiatetime\",0,120", "OK", 3000);

  // Init HTTP
  AT.sendATExpect("AT+HTTPTERM", "OK", 1000);
  if (!AT.sendATExpect("AT+HTTPINIT", "OK", 5000)) return false;
  AT.sendATExpect("AT+HTTPPARA=\"CID\",1", "OK", 2000);
  AT.sendATExpect("AT+HTTPPARA=\"SSLCFG\",0", "OK", 2000);
  String url = String("https://") + host + path;
  if (!AT.sendATExpect(String("AT+HTTPPARA=\"URL\",\"") + url + "\"", "OK", 5000)) {
    AT.sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  AT.sendATExpect(String("AT+HTTPPARA=\"CONTENT\",\"") + contentType + "\"", "OK", 2000);
  // Use POST with override header to achieve PUT semantics on Firebase RTDB
  AT.sendATExpect("AT+HTTPPARA=\"USERDATA\",\"X-HTTP-Method-Override: PUT\"", "OK", 2000);

  String dataCmd = String("AT+HTTPDATA=") + body.length() + ",60000";
  if (!AT.sendATExpect(dataCmd, "DOWNLOAD", 5000)) {
    AT.sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  MODEM.print(body);
  AT.waitForAny("OK", nullptr, nullptr, 10000);
  if (!AT.sendATExpect("AT+HTTPACTION=1", "OK", 2000)) { // POST with override
    AT.sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  if (!AT.waitForAny("+HTTPACTION: 1,200", "+HTTPACTION: 1,201", "+HTTPACTION: 1,204", 60000)) {
    AT.sendATExpect("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  // Optional read
  AT.sendATCollect("AT+HTTPREAD?", 1000);
  AT.sendATCollect("AT+HTTPREAD=0,512", 3000);
  AT.sendATExpect("AT+HTTPTERM", "OK", 1000);
  return true;
}

static bool httpPostViaSockets(const String &host, int port, const String &path, const String &contentType, const String &body) {
  MODEM.print("AT+NETOPEN\r\n");
  waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 10000);
  sendATCollect(String("AT+CDNSGIP=\"") + host + "\"", 8000);
  String open = String("AT+CIPOPEN=7,\"TCP\",\"") + host + "\"," + port;
  Serial.print(F("--> "));
  Serial.println(open);
  MODEM.print(open);
  MODEM.print("\r\n");
  if (!waitForAny("+CIPOPEN: 7,0", "ALREADY CONNECT", nullptr, 15000)) return false;

  String req;
  req += String("POST ") + path + " HTTP/1.1\r\n";
  req += String("Host: ") + host + "\r\n";
  req += String("User-Agent: A7608E-POST\r\n");
  req += String("Content-Type: ") + contentType + "\r\n";
  req += String("Content-Length: ") + String(body.length()) + "\r\n";
  req += String("Connection: close\r\n\r\n");
  req += body;

  String sendCmd = String("AT+CIPSEND=7,") + req.length();
  Serial.print(F("--> "));
  Serial.println(sendCmd);
  MODEM.print(sendCmd);
  MODEM.print("\r\n");
  if (!waitForAny(">", "DOWNLOAD", nullptr, 5000)) return false;
  MODEM.print(req);
  waitForAny("SEND OK", nullptr, nullptr, 10000);

  sendATExpect("AT+CIPRXGET=1", "OK", 2000);
  // Read a few chunks for demo
  for (int i = 0; i < 10; ++i) {
    sendATCollect("AT+CIPRXGET=2,7,512", 500);
  }
  sendATExpect("AT+CIPCLOSE=7", "OK", 5000);
  return true;
}

static bool httpsPostViaCipssl(const String &host, const String &path, const String &contentType, const String &body) {
  MODEM.print("AT+NETOPEN\r\n");
  waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 10000);
  if (!sendATExpect("AT+CIPSSL=1", "OK", 3000)) {
    Serial.println(F("CIPSSL not supported (HTTPS unavailable)"));
    return false;
  }
  sendATCollect(String("AT+CDNSGIP=\"") + host + "\"", 8000);
  String open = String("AT+CIPOPEN=8,\"TCP\",\"") + host + "\",443";
  Serial.print(F("--> "));
  Serial.println(open);
  MODEM.print(open);
  MODEM.print("\r\n");
  if (!waitForAny("+CIPOPEN: 8,0", "ALREADY CONNECT", nullptr, 20000)) return false;

  String req;
  req += String("POST ") + path + " HTTP/1.1\r\n";
  req += String("Host: ") + host + "\r\n";
  req += String("User-Agent: A7608E-POST\r\n");
  req += String("Content-Type: ") + contentType + "\r\n";
  req += String("Content-Length: ") + String(body.length()) + "\r\n";
  req += String("Connection: close\r\n\r\n");
  req += body;

  String sendCmd = String("AT+CIPSEND=8,") + req.length();
  Serial.print(F("--> "));
  Serial.println(sendCmd);
  MODEM.print(sendCmd);
  MODEM.print("\r\n");
  if (!waitForAny(">", "DOWNLOAD", nullptr, 5000)) return false;
  MODEM.print(req);
  waitForAny("SEND OK", nullptr, nullptr, 10000);

  sendATExpect("AT+CIPRXGET=1", "OK", 2000);
  for (int i = 0; i < 10; ++i) {
    sendATCollect("AT+CIPRXGET=2,8,512", 500);
  }
  sendATExpect("AT+CIPCLOSE=8", "OK", 5000);
  return true;
}

static bool httpsPutViaCipssl(const String &host, const String &path, const String &contentType, const String &body) {
  MODEM.print("AT+NETOPEN\r\n");
  AT.waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 10000);
  if (!AT.sendATExpect("AT+CIPSSL=1", "OK", 3000)) {
    Serial.println(F("CIPSSL not supported (HTTPS unavailable)"));
    return false;
  }
  AT.sendATCollect(String("AT+CDNSGIP=\"") + host + "\"", 8000);
  String open = String("AT+CIPOPEN=11,\"TCP\",\"") + host + "\",443";
  Serial.print(F("--> "));
  Serial.println(open);
  MODEM.print(open);
  MODEM.print("\r\n");
  if (!AT.waitForAny("+CIPOPEN: 11,0", "ALREADY CONNECT", nullptr, 20000)) return false;

  String req;
  req += String("PUT ") + path + " HTTP/1.1\r\n";
  req += String("Host: ") + host + "\r\n";
  req += String("User-Agent: A7608E-PUT\r\n");
  req += String("Content-Type: ") + contentType + "\r\n";
  req += String("Content-Length: ") + String(body.length()) + "\r\n";
  req += String("Connection: close\r\n\r\n");
  req += body;

  String sendCmd = String("AT+CIPSEND=11,") + req.length();
  Serial.print(F("--> "));
  Serial.println(sendCmd);
  MODEM.print(sendCmd);
  MODEM.print("\r\n");
  if (!AT.waitForAny(">", "DOWNLOAD", nullptr, 5000)) return false;
  MODEM.print(req);
  AT.waitForAny("SEND OK", nullptr, nullptr, 10000);

  AT.sendATExpect("AT+CIPRXGET=1", "OK", 2000);
  for (int i = 0; i < 10; ++i) {
    AT.sendATCollect("AT+CIPRXGET=2,11,512", 500);
  }
  AT.sendATExpect("AT+CIPCLOSE=11", "OK", 5000);
  return true;
}

// HTTPS via SIMCom CHTTPS commands (if supported by firmware)
static bool httpsPostViaChttps(const String &host, const String &path, const String &contentType, const String &body) {
  MODEM.print("AT+NETOPEN\r\n");
  waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 10000);

  if (!sendATExpect("AT+CHTTPSSTART", "OK", 10000)) return false;

  // Open secure session (mode 2 = TLS). Some fw requires waiting for URC; we proceed after OK.
  String opse = String("AT+CHTTPSOPSE=\"") + host + "\",443,2";
  if (!sendATExpect(opse, "OK", 15000)) {
    sendATExpect("AT+CHTTPSSTOP", "OK", 5000);
    return false;
  }

  sendATExpect("AT+CHTTPSHEADER=1", "OK", 3000);
  sendATExpect(String("AT+CHTTPSHEADER=\"Content-Type: ") + contentType + "\"", "OK", 3000);

  String postCmd = String("AT+CHTTPSPOST=\"") + path + "\"," + body.length() + ",60";
  Serial.print(F("--> "));
  Serial.println(postCmd);
  MODEM.print(postCmd);
  MODEM.print("\r\n");
  if (!waitForAny("DOWNLOAD", ">", nullptr, 8000)) {
    sendATExpect("AT+CHTTPSCLSE", "OK", 3000);
    sendATExpect("AT+CHTTPSSTOP", "OK", 3000);
    return false;
  }
  MODEM.print(body);
  // Wait for completion; different fw respond differently; then fetch data
  waitForAny("OK", "+CHTTPS", nullptr, 30000);
  sendATCollect("AT+CHTTPSRECV=512,60", 8000);
  sendATExpect("AT+CHTTPSCLSE", "OK", 3000);
  sendATExpect("AT+CHTTPSSTOP", "OK", 3000);
  return true;
}

static void runAutoATScript() {
  Serial.println(F("\n=== AUTO AT SCRIPT START ==="));
  sendATExpect("AT", "OK", 1000);
  sendATExpect("AT+CMEE=2", "OK", 1000);
  sendATCollect("AT+CGDCONT?", 2000);

  if (AUTO_APN && *AUTO_APN) {
    sendATExpect("AT+CGATT=0", "OK", 8000);
    sendATExpect("AT+CGACT=0,1", "OK", 8000);
    sendATExpect(String("AT+CGDCONT=1,\"IP\",\"") + AUTO_APN + "\"", "OK", 5000);
    if (AUTO_APN_USER && *AUTO_APN_USER) {
      String auth = String("AT+CGAUTH=1,1,\"") + AUTO_APN_USER + "\",\"" + AUTO_APN_PASS + "\"";
      sendATExpect(auth, "OK", 5000);
    }
    sendATExpect("AT+CFUN=1,1", "OK", 2000);
    delay(3000);
  }

  sendATExpect("AT+CPIN?", "READY", 5000);
  sendATExpect("AT+CGATT=1", "OK", 15000);
  sendATExpect("AT+CGACT=1,1", "OK", 20000);
  String contrdp = sendATCollect("AT+CGCONTRDP=1", 3000);

  MODEM.print("AT+NETOPEN\r\n");
  waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 15000);

  sendATCollect("AT+CDNSGIP=\"httpbin.org\"", 8000);
  sendATExpect("AT+CIPOPEN=0,\"TCP\",\"httpbin.org\",80", "OK", 3000);
  waitForAny("+CIPOPEN: 0,0", "+CIPOPEN: 0,", nullptr, 8000);
  sendATExpect("AT+CIPCLOSE=0", "OK", 3000);

  if (NTRIP_HOST.length() > 0) {
    String open = String("AT+CIPOPEN=0,\"TCP\",\"") + NTRIP_HOST + "\",2101";
    sendATExpect(open, "OK", 3000);
    waitForAny("+CIPOPEN: 0,0", "+CIPOPEN: 0,", nullptr, 8000);
    sendATExpect("AT+CIPCLOSE=0", "OK", 3000);
  }

  runNetDiagnostics(contrdp);
  testTcpToDns53(contrdp);
  Serial.println(F("=== AUTO AT SCRIPT END ===\n"));
}
void setup() {
  Serial.begin(115200);
  delay(200);

  powerOnModem();

  MODEM.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  Serial.println("A7608E-H ready: TX=8 RX=9 PWRKEY=10");
  Serial.println("Type AT commands below.\n");

  if (RUN_AUTO_AT_SCRIPT) {
    runAutoATScript();
  }

  if (RUN_AUTO_HTTP_TEST) {
    const char *apn = "lte-internet.sktelecom.com"; // SKTelecom default
    const char *testUrl = "http://httpbin.org/get";
    if (!ensurePdpAndHttpGet(apn, testUrl)) {
      Serial.println(F("HTTP test failed. You can still use the manual bridge."));
    }
  }

  // Optional NTRIP demo (requires outbound 2101 allowed)
  if (ENABLE_NTRIP_DEMO) {
    ntripClientRunStr(NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNT, NTRIP_USER, NTRIP_PASS);
  }
}

void loop() {
  // Command parser on USB Serial; type !help for commands
  static String cmd;
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (cmd.length() > 0) {
        if (cmd.startsWith("!help")) {
          Serial.println(F("Commands: !test, !apn <name>, !netopen, !close, !ntrip <host> <mount> <user> <pass>, !tcp <host> <port>, !basic, !post <json>, !httppost <host> <port> <path> <json>, !fpost <url_path> <json>, !rtdb <path> <json> [auth], !rtdbput <path> <json> [auth], !cap"));
        } else if (cmd.startsWith("!apn ")) {
          String apn = cmd.substring(5);
          apn.trim();
          if (apn.length() > 0) {
            sendATExpect("AT+CGATT=0", "OK", 5000);
            sendATExpect("AT+CGACT=0,1", "OK", 5000);
            sendATExpect(String("AT+CGDCONT=1,\"IP\",\"") + apn + "\"", "OK", 5000);
            Serial.println(F("APN set; rebooting modem"));
            sendATExpect("AT+CFUN=1,1", "OK", 2000);
          }
        } else if (cmd.startsWith("!netopen")) {
          MODEM.print("AT+NETOPEN\r\n");
          waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 15000);
        } else if (cmd.startsWith("!close")) {
          sendATExpect("AT+NETCLOSE", "OK", 5000);
        } else if (cmd.startsWith("!test")) {
          // Connectivity suite: APN info, DNS, CIPOPEN 80/443/2101, ping
          sendATCollect("AT+CGDCONT?", 2000);
          String contrdp = sendATCollect("AT+CGCONTRDP=1", 3000);
          MODEM.print("AT+NETOPEN\r\n");
          waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 15000);
          sendATCollect("AT+CDNSGIP=\"httpbin.org\"", 8000);
          // Try TCP 80
          sendATExpect("AT+CIPOPEN=0,\"TCP\",\"52.202.66.54\",80", "OK", 3000);
          waitForAny("+CIPOPEN: 0,0", "+CIPOPEN: 0,", nullptr, 8000);
          sendATExpect("AT+CIPCLOSE=0", "OK", 3000);
          // Try TCP 2101 to placeholder host if configured
          if (NTRIP_HOST.length() > 0) {
            String open = String("AT+CIPOPEN=0,\"TCP\",\"") + NTRIP_HOST + "\",2101";
            sendATExpect(open, "OK", 3000);
            waitForAny("+CIPOPEN: 0,0", "+CIPOPEN: 0,", nullptr, 8000);
            sendATExpect("AT+CIPCLOSE=0", "OK", 3000);
          }
          runNetDiagnostics(contrdp);
          testTcpToDns53(contrdp);
        } else if (cmd.startsWith("!ntrip ")) {
          // Parse: !ntrip host mount user pass
          String args = cmd.substring(7);
          args.trim();
          int s1 = args.indexOf(' ');
          int s2 = s1 > 0 ? args.indexOf(' ', s1 + 1) : -1;
          int s3 = s2 > 0 ? args.indexOf(' ', s2 + 1) : -1;
          if (s1 > 0 && s2 > s1 && s3 > s2) {
            String host = args.substring(0, s1);
            String mount = args.substring(s1 + 1, s2);
            String user = args.substring(s2 + 1, s3);
            String pass = args.substring(s3 + 1);
            ntripClientRunStr(host, 2101, mount, user, pass);
          } else {
            Serial.println(F("Usage: !ntrip <host> <mount> <user> <pass>"));
          }
        } else if (cmd.startsWith("!tcp ")) {
          // Usage: !tcp <host> <port>
          String args = cmd.substring(5);
          args.trim();
          int sp = args.lastIndexOf(' ');
          if (sp > 0) {
            String host = args.substring(0, sp);
            String portStr = args.substring(sp + 1);
            int port = portStr.toInt();
            if (port > 0 && port <= 65535) {
              // Ensure network open
              MODEM.print("AT+NETOPEN\r\n");
              waitForAny("+NETOPEN:", "ALREADY CONNECT", "OK", 10000);
              // Optional DNS resolve for logging
              sendATCollect(String("AT+CDNSGIP=\"") + host + "\"", 8000);
              // Open on channel 3
              String open = String("AT+CIPOPEN=3,\"TCP\",\"") + host + "\"," + port;
              Serial.print(F("--> "));
              Serial.println(open);
              MODEM.print(open);
              MODEM.print("\r\n");
              bool any = waitForAny("+CIPOPEN:", "OK", nullptr, 15000);
              bool ok = waitForAny("+CIPOPEN: 3,0", "ALREADY CONNECT", nullptr, 15000);
              if (!any || !ok) Serial.println(F("TCP open failed"));
              sendATExpect("AT+CIPCLOSE=3", "OK", 5000);
            } else {
              Serial.println(F("Usage: !tcp <host> <port>"));
            }
          } else {
            Serial.println(F("Usage: !tcp <host> <port>"));
          }
        } else if (cmd.startsWith("!basic")) {
          runBasicServersTest();
        } else if (cmd.startsWith("!post ")) {
          // POST JSON to httpbin.org/post over HTTP (port 80)
          String json = cmd.substring(6);
          json.trim();
          if (json.length() == 0) json = "{\"hello\":\"world\"}";
          bool ok = httpPostViaSockets("httpbin.org", 80, "/post", "application/json", json);
          Serial.println(ok ? F("POST httpbin OK") : F("POST httpbin FAIL"));
        } else if (cmd.startsWith("!httppost ")) {
          // Usage: !httppost <host> <port> <path> <json>
          String args = cmd.substring(10);
          args.trim();
          // Split first three tokens, remainder is JSON body
          int s1 = args.indexOf(' ');
          int s2 = s1 > 0 ? args.indexOf(' ', s1 + 1) : -1;
          int s3 = s2 > 0 ? args.indexOf(' ', s2 + 1) : -1;
          if (s1 > 0 && s2 > s1 && s3 > s2) {
            String host = args.substring(0, s1);
            String portStr = args.substring(s1 + 1, s2);
            String path = args.substring(s2 + 1, s3);
            String body = args.substring(s3 + 1);
            int port = portStr.toInt();
            if (port <= 0 || port > 65535) {
              Serial.println(F("Usage: !httppost <host> <port> <path> <json>"));
            } else {
              if (body.length() == 0) body = "{}";
              bool ok = httpPostViaSockets(host, port, path, "application/json", body);
              Serial.println(ok ? F("HTTP POST OK") : F("HTTP POST FAIL"));
            }
          } else {
            Serial.println(F("Usage: !httppost <host> <port> <path> <json>"));
          }
        } else if (cmd.startsWith("!fpost ")) {
          // Firebase-like HTTPS POST: !fpost /v0/b/<bucket>/o?name=foo.json {"a":1}
          // Host must be set via NTRIP_HOST var for reuse, or replace here with your firebase host
          String args = cmd.substring(7);
          args.trim();
          int sp = args.indexOf(' ');
          if (sp > 0) {
            String path = args.substring(0, sp);
            String json = args.substring(sp + 1);
            if (json.length() == 0) json = "{}";
            String fbHost = NTRIP_HOST.length() ? NTRIP_HOST : String("firebasestorage.googleapis.com");
            bool ok = httpsPostViaCipssl(fbHost, path, "application/json", json);
            Serial.println(ok ? F("HTTPS POST OK") : F("HTTPS POST FAIL"));
          } else {
            Serial.println(F("Usage: !fpost <url_path> <json>"));
          }
        } else if (cmd.startsWith("!rtdb ")) {
          // Firebase Realtime Database HTTPS write: !rtdb <path> <json> [auth]
          String args = cmd.substring(6);
          args.trim();
          // Split last two parts as json and optional auth token, first part as path
          int firstSpace = args.indexOf(' ');
          if (firstSpace > 0) {
            String pathAndRest = args;
            String path = pathAndRest.substring(0, firstSpace);
            String rest = pathAndRest.substring(firstSpace + 1);
            rest.trim();
            // Optional auth after a space-delimited token
            String json = rest;
            String auth;
            int authSep = rest.lastIndexOf(' ');
            if (authSep > 0) {
              String maybeJson = rest.substring(0, authSep);
              String maybeAuth = rest.substring(authSep + 1);
              // Heuristic: if maybeAuth contains '=' or is shorter (token-like), treat as auth
              if (maybeAuth.indexOf('=') >= 0 || maybeAuth.length() < 64) {
                json = maybeJson;
                auth = maybeAuth;
              }
            }
            String norm = normalizeRtdbPath(path);
            if (auth.length() > 0) norm += String("?auth=") + auth;
            if (json.length() == 0) json = "{}";
            bool ok = httpStackHttpsPost(RTDB_HOST, norm, "application/json", json);
            Serial.println(ok ? F("RTDB POST OK") : F("RTDB POST FAIL"));
          } else {
            Serial.println(F("Usage: !rtdb <path> <json> [auth]"));
          }
        } else if (cmd.startsWith("!rtdbput ")) {
          // Firebase Realtime Database HTTPS overwrite (PUT): !rtdbput <path> <json> [auth]
          String args = cmd.substring(9);
          args.trim();
          int firstSpace = args.indexOf(' ');
          if (firstSpace > 0) {
            String pathAndRest = args;
            String path = pathAndRest.substring(0, firstSpace);
            String rest = pathAndRest.substring(firstSpace + 1);
            rest.trim();
            String json = rest;
            String auth;
            int authSep = rest.lastIndexOf(' ');
            if (authSep > 0) {
              String maybeJson = rest.substring(0, authSep);
              String maybeAuth = rest.substring(authSep + 1);
              if (maybeAuth.indexOf('=') >= 0 || maybeAuth.length() < 64) {
                json = maybeJson;
                auth = maybeAuth;
              }
            }
            String norm = normalizeRtdbPath(path);
            if (auth.length() > 0) norm += String("?auth=") + auth;
            if (json.length() == 0) json = "{}";
            bool ok = httpStackHttpsPut(RTDB_HOST, norm, "application/json", json);
            Serial.println(ok ? F("RTDB PUT OK") : F("RTDB PUT FAIL"));
          } else {
            Serial.println(F("Usage: !rtdbput <path> <json> [auth]"));
          }
        } else if (cmd.startsWith("!cap")) {
          runCapabilityProbe();
        } else {
          // Unknown command: forward as AT
          MODEM.print(cmd);
          MODEM.print("\r\n");
        }
      }
      cmd = "";
    } else {
      cmd += (char)ch;
    }
  }
  // Transparent bridge for manual AT when idle
  while (MODEM.available()) {
    Serial.write(MODEM.read());
  }
  while (MODEM.available()) {
    Serial.write(MODEM.read());
  }
}