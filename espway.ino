#include <Wire.h>
#include <ESP8266WiFi.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define ENABLE_FOTA

#include "mpu6050.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const int MPU_ADDR = 0x68;
mpuconfig gMpuConfig = {
    .lowpass = 3,
    .sampleRateDivider = 0,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true,
    .intActiveLow = false,
    .intOpenDrain = false,
    .beta = 0.05f
};

volatile float roll, pitch;

const bool TRACK_UPDATE_FREQUENCY = true;
unsigned int gNumCalculations = 0;
unsigned long gLastReportTime = 0;

volatile bool gDataAvailable = false;
void dataAvailable() {
    gDataAvailable = true;
}

void onRequest(AsyncWebServerRequest *request){
    //Handle Unknown Request
    request->send(404);
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client,
    AwsEventType type, void * arg, uint8_t *data, size_t len) {
    // Handle binary WebSocket data from client

    if (type == WS_EVT_CONNECT) {
        client->ping();
        return;
    }
    if (type != WS_EVT_DATA) return;

    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode != WS_BINARY) return;
    if (len > 0) digitalWrite(LED_BUILTIN, data[0]);
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESPway");
    SPIFFS.begin();

    ws.onEvent(onEvent);
    server.addHandler(&ws);

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // Handle any other requests
    server.onNotFound(onRequest);

    server.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 12; i <= 15; ++i) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }

    Wire.begin(0, 5);
    Wire.setClock(400000L);
    if (mpuSetup(MPU_ADDR, &gMpuConfig) != 0) {
        Serial.println("Setup failed");
        return;
    }

    attachInterrupt(4, dataAvailable, RISING);

#ifdef ENABLE_FOTA
    ArduinoOTA.onStart([]() {
        SPIFFS.end();

        // Disable client connections
        ws.enable(false);

        // Advertise connected clients what's going on
        ws.textAll("OTA Update Started");

        // Close them
        ws.closeAll();
    });
    ArduinoOTA.begin();
#endif
}

void loop() {
    while (true) {//!gDataAvailable) {
#ifdef ENABLE_FOTA
        ArduinoOTA.handle();
#endif
        yield();
    }
    gDataAvailable = false;

    int16_t buf[6];
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdateQuaternion(&gMpuConfig, buf);

    roll = rollAngleTaylor();
    pitch = pitchAngleTaylor();

    if (TRACK_UPDATE_FREQUENCY) {
        if (++gNumCalculations == 1000) {
            gNumCalculations = 0;
            unsigned long time = millis();
            Serial.println(1000000L / (time - gLastReportTime));
            gLastReportTime = time;
        }
    } else {
        Serial.print(100.0f * roll);
        Serial.print(',');
        Serial.println(100.0f * pitch);
    }
}

