#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define ENABLE_FOTA
#define SERIAL_DEBUG

#include "mpu6050.h"
#include "motor.h"

#include "indexhtml.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const int MPU_ADDR = 0x68;
mpuconfig gMpuConfig = {
    .lowpass = 3,
    .sampleRateDivider = 1,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true,
    .intActiveLow = false,
    .intOpenDrain = false,
    .beta = 0.05f
};
quaternion gQuat = { 1.0, 0.0, 0.0, 0.0 };

motor motorLeft = {
    .pwmPin = 13,
    .directionPin = 16,
    .reverse = false
};
motor motorRight = {
    .pwmPin = 12,
    .directionPin = 14,
    .reverse = false
};

volatile float roll, pitch;

const bool TRACK_UPDATE_FREQUENCY = false;
unsigned int gNumCalculations = 0;
unsigned long gLastReportTime = 0;

const unsigned int ORIENTATION_SEND_DT = 50;
unsigned long gLastOrientationTime = 0;

volatile bool gDataAvailable = false;
void dataAvailable() {
    gDataAvailable = true;
}

void onRequest(AsyncWebServerRequest *request){
    //Handle Unknown Request
    request->send(404);
}

void sendLedStatus(AsyncWebSocketClient *client) {
    uint8_t data[] = { digitalRead(LED_BUILTIN) };
    client->binary(data, 1);
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client,
    AwsEventType type, void * arg, uint8_t *data, size_t len) {
    return;
    // Handle binary WebSocket data from client

    if (type == WS_EVT_CONNECT) {
        sendLedStatus(client);
        return;
    }
    if (type != WS_EVT_DATA) return;

    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode != WS_BINARY) return;
    if (len > 0) digitalWrite(LED_BUILTIN, data[0]);
    sendLedStatus(client);
}

void setup() {
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    // WiFi AP setup
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESPway");

    // Server setup
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse_P(200,
            "text/html", indexHtml);
        response->addHeader("Connection", "close");
        request->send(response);
    });
    // Handle any other requests
    server.onNotFound(onRequest);
    server.begin();

    // IMU setup
    Wire.begin(0, 5);
    Wire.setClock(400000L);
    if (mpuSetup(MPU_ADDR, &gMpuConfig) != 0) {
#ifdef SERIAL_DEBUG
        Serial.println("Setup failed");
#endif
        return;
    }
    attachInterrupt(4, dataAvailable, RISING);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Motor setup
    setupMotorPwm(motorLeft);
    setupMotorPwm(motorRight);

#ifdef ENABLE_FOTA
    ArduinoOTA.onStart([]() {
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
    while (!gDataAvailable) {
#ifdef ENABLE_FOTA
        ArduinoOTA.handle();
#endif
        yield();
    }
    gDataAvailable = false;
    unsigned long time = millis();

    int16_t buf[6];
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdateQuaternion(&gMpuConfig, buf, &gQuat);

    roll = rollAngleTaylor(&gQuat);
    pitch = pitchAngleTaylor(&gQuat);

    if (TRACK_UPDATE_FREQUENCY) {
        if (++gNumCalculations == 1000) {
            gNumCalculations = 0;
#ifdef SERIAL_DEBUG
            Serial.println(1000000L / (time - gLastReportTime));
#endif
            gLastReportTime = time;
        }
    } else {
        if (time - gLastOrientationTime > ORIENTATION_SEND_DT) {
            gLastOrientationTime = time;
            float scale = 1000.0f;
            int16_t data[] = { scale * gQuat.q0,
                               scale * gQuat.q1,
                               scale * gQuat.q2,
                               scale * gQuat.q3 };
            ws.binaryAll((uint8_t *)data, 8);
        }
#ifdef SERIAL_DEBUG
        Serial.print(100.0f * roll);
        Serial.print(',');
        Serial.println(100.0f * pitch);
#endif
    }
}

