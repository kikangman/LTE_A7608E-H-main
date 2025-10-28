#pragma once

#include <Arduino.h>

class AtCommands {
public:
  AtCommands(Stream &modem, Stream &debug) : modem_(modem), debug_(debug) {}

  void flushInput(Stream &s, uint32_t durationMs = 50);
  bool waitForResponse(const char *expect, uint32_t timeoutMs);
  bool sendATExpect(const String &cmd, const char *expect, uint32_t timeoutMs);
  bool waitForAny(const char *e1, const char *e2, const char *e3, uint32_t timeoutMs);
  String sendATCollect(const String &cmd, uint32_t timeoutMs);

private:
  Stream &modem_;
  Stream &debug_;
};


