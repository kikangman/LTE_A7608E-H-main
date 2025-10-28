#include "AtCommands.h"

void AtCommands::flushInput(Stream &s, uint32_t durationMs) {
  uint32_t start = millis();
  while (millis() - start < durationMs) {
    while (s.available()) {
      (void)s.read();
      start = millis();
    }
    delay(1);
  }
}

bool AtCommands::waitForResponse(const char *expect, uint32_t timeoutMs) {
  String buf;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (modem_.available()) {
      char c = (char)modem_.read();
      debug_.write(c);
      buf += c;
      if (buf.indexOf(expect) >= 0) return true;
      if (buf.indexOf("ERROR") >= 0) return false;
    }
    delay(1);
  }
  return false;
}

bool AtCommands::sendATExpect(const String &cmd, const char *expect, uint32_t timeoutMs) {
  debug_.print(F("--> "));
  debug_.println(cmd);
  modem_.print(cmd);
  modem_.print("\r\n");
  return waitForResponse(expect, timeoutMs);
}

bool AtCommands::waitForAny(const char *e1, const char *e2, const char *e3, uint32_t timeoutMs) {
  String buf;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (modem_.available()) {
      char c = (char)modem_.read();
      debug_.write(c);
      buf += c;
      if ((e1 && buf.indexOf(e1) >= 0) || (e2 && buf.indexOf(e2) >= 0) || (e3 && buf.indexOf(e3) >= 0)) return true;
      if (buf.indexOf("ERROR") >= 0) return false;
    }
    delay(1);
  }
  return false;
}

String AtCommands::sendATCollect(const String &cmd, uint32_t timeoutMs) {
  debug_.print(F("--> "));
  debug_.println(cmd);
  modem_.print(cmd);
  modem_.print("\r\n");
  String buf;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (modem_.available()) {
      char c = (char)modem_.read();
      debug_.write(c);
      buf += c;
    }
    delay(1);
  }
  return buf;
}


