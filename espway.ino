#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define ENABLE_FOTA
#define SERIAL_DEBUG

#include "mpu6050.h"
#include "motor.h"

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

const bool TRACK_UPDATE_FREQUENCY = true;
unsigned int gNumCalculations = 0;
unsigned long gLastReportTime = 0;
extern const char *indexHtml PROGMEM;

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
#ifdef SERIAL_DEBUG
            Serial.println(1000000L / (time - gLastReportTime));
#endif
            gLastReportTime = time;
        }
    } else {
#ifdef SERIAL_DEBUG
        Serial.print(100.0f * roll);
        Serial.print(',');
        Serial.println(100.0f * pitch);
#endif
    }
}

const char *indexHtml PROGMEM = R"(
<!doctype html>

<html>
    <head>
        <meta charset='utf-8' />
        <meta name='viewport' content='width=device-width, initial-scale=1' />

        <title>ESPway</title>

        <style>
            html {
                font-size: 32px;
            }

            body {
                font-family: sans-serif;
                text-align: center;
                margin: 0 auto;
            }

            button {
                font-size: 2rem;
                width: 4rem;
                height: 3rem;
            }
        </style>
    </head>

    <body>
        <h1>Led is <br /><span id='ledStatus'></span></h1>
        <button id='btnOn'>on</button>
        <button id='btnOff'>off</button>

        <script>
            (function() {
                'use strict'

                let byId = id => document.getElementById(id)

                function main() {
                    let ws = new WebSocket('ws://' + location.host + '/ws')
                    ws.binaryType = 'arraybuffer'

                    ws.addEventListener('message', e => {
                        let arr = new Uint8Array(e.data)
                        if (arr[0] == 1) {
                            byId('ledStatus').textContent = 'off'
                        } else {
                            byId('ledStatus').textContent = 'on'
                        }
                    })

                    let sendBytes = bytes => ws.send((new Uint8Array(bytes)).buffer)

                    byId('btnOn').addEventListener('click', () => sendBytes([0]))
                    byId('btnOff').addEventListener('click', () => sendBytes([1]))
                }

                window.addEventListener('load', main)
            })()
        </script>
    </body>
</html>
)";

