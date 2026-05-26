#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>

extern UserAuth user_auth;
extern FirebaseApp app;
extern WiFiClientSecure ssl;
extern AsyncClientClass aClient;
extern RealtimeDatabase Database;

void connectWiFi();
void initFirebase();

void publishMetaOnce();
void publishStatus();
void publishLive();
void publishRunSample();
void resetRunAccumulators();

void applyCommand(const String& action, uint32_t requestId);
void pollCommands();
void pollConfig();
