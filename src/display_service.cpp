#include "display_service.h"

#include <Wire.h>
#include <WiFi.h>

#include "config.h"
#include "models.h"

// =====================
// Display instance
// =====================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void drawStateScreen(const String& title, const String& line1, const String& line2, const String& line3) {
  if (!oledOk) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.println("----------------");
  if (line1.length()) display.println(line1);
  if (line2.length()) display.println(line2);
  if (line3.length()) display.println(line3);
  display.display();
}

void refreshOLED() {
  if (!oledOk) return;

  if (currentState == STATE_ERROR) {
    drawStateScreen("ERROR", errorText.length() ? errorText : "Unknown error");
    return;
  }

  if (currentState == STATE_CALIBRATING) return;

  if (currentState == STATE_RUNNING) {
    unsigned long sec = (millis() - recordStartMs) / 1000;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RUNNING");
    display.println("----------------");
    display.printf("T:%lus Y:%.1f\n", sec, yawRateDps);
    display.printf("G:%.2f D:%s\n", latG, drifting ? "Y" : "N");
    display.printf("WiFi:%s", wifiOk ? "OK" : "NO");
    display.display();
    return;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(stateToString(currentState));
  display.println("----------------");
  display.printf("Yaw: %.1f dps\n", yawRateDps);
  display.printf("LatG: %.2f g\n", latG);
  display.printf("WiFi:%s FB:%s\n", wifiOk ? "Y" : "N", firebaseReady ? "Y" : "N");
  display.display();
}
