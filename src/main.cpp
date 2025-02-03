#include <Arduino.h>
#include <mcp_can.h>

#define CAN0_INT 21  // Set INT to pin 2
MCP_CAN CAN0(5);   // Set CS to pin 10

struct Speed {
    uint16_t engine;
    uint16_t vehicle;
};

Speed speed;

void SetupMCP(byte speed, byte clock) {
    if (CAN0.begin(MCP_ANY, speed, clock) == CAN_OK) {
        if (CAN0.setMode(MCP_NORMAL) ==
            CAN_OK) {  // Set operation mode to normal so the MCP2515 sends acks to received data.
            Serial.println("MCP2515 Initialized Successfully!");
        } else {
            Serial.println("Error Set Mode to MCP_NORMAL...");
        }
    } else {
        Serial.println("Error Initializing MCP2515...");
    }
    pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
void PrintRAWData(unsigned char len, unsigned char data[8]) {
    for (int i = 0; i < len; i++) {
        Serial.print("0x");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
}

void SpeedEngine(const unsigned char data[8]) {
    uint8_t byte1 = data[3];
    uint8_t byte2 = data[4];
    uint16_t big_endian_value = (256 * byte1) + byte2;
    speed.engine = (int) (big_endian_value * 0.25);
}

void SpeedVehicle(const unsigned char data[8]) {
    speed.vehicle = (int) data[3];
}

void Speed(const unsigned char data[8]) {
    if (data[2] == 0x0C) {
        Serial.println("RPM");
        SpeedEngine(data);
    } else if (data[2] == 0x0D) {
        Serial.println("KMH");
        SpeedVehicle(data);
    }
}

void RequestCAN(uint16_t id, uint8_t data[8]) {
    if (CAN0.sendMsgBuf(id, 0, 8, data) == CAN_OK) {
        Serial.print("");
    } else {
        char msgString[128];
        sprintf(msgString, "Error PID 0x%.3lX sending request!", id);
        Serial.println(msgString);
    }
}

void RequestSpeedEngine() {
    // ID запроса (настройте в зависимости от вашего устройства)
    uint16_t requestID = 0x7DF;
    uint8_t requestData[8] = {0x02, 0x01, 0x0C, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

    RequestCAN(requestID, requestData);
}

void RequestSpeedVehicle() {
    uint16_t requestID = 0x7DF;
    uint8_t requestData[8] = {0x02, 0x01, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

    RequestCAN(requestID, requestData);
}

void readCANResponse() {
    // Проверка, есть ли данные в буфере CAN
    if (!digitalRead(CAN0_INT)) {
        // Буфер не пуст, читаем данные
        long unsigned int responseID;
        unsigned char responseLen = 0;
        unsigned char responseData[8];

        // Чтение данных из буфера
        if (CAN0.readMsgBuf(&responseID, &responseLen, responseData) == CAN_OK) {
            if (responseID == 0x7E8) {
                Speed(responseData);
            }
            Serial.println();
        } else {
            Serial.println("Error reading response!");
        }
    }
}

void setup() {
    Serial.begin(115200);
    SetupMCP(CAN_500KBPS, MCP_8MHZ);
}

void loop() {
    RequestSpeedEngine();
    readCANResponse();
    RequestSpeedVehicle();
    readCANResponse();

    Serial.print("RPM: ");
    Serial.println(speed.engine);
    Serial.print("Vehicle: ");
    Serial.println(speed.vehicle);

    delay(1000);
}
