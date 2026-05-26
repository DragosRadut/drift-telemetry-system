#pragma once

#include <Arduino.h>
#include "config.h"

inline String basePath() {
  return String("/devices/") + DEVICE_ID;
}

inline String statusPath() {
  return basePath() + "/status";
}

inline String livePath() {
  return basePath() + "/live";
}

inline String commandsPath() {
  return basePath() + "/commands";
}

inline String configPath() {
  return basePath() + "/config";
}

inline String metaPath() {
  return basePath() + "/meta";
}

inline String runsPath() {
  return basePath() + "/runs";
}

inline String makeRunId() {
  return String("run-") + String(millis());
}
