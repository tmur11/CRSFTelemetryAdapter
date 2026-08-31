#include "HWTelemetry.h"

// 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31

// XeRun XR10 Pro Elite G2S
// FE 01 00 03 30 5C 17 06 00 2F 30 01 00 F5 04 4B 00 FF FF 25 FF 22 FF FF FF FF FF FF FF FF 83 95
// XeRun XR8 PRO G3
// FE 01 00 03 30 5C 17 06 00 00 00 00 00 00 00 55 00 00 00 15 00 12 00 00 07 FF FF FF FF FF 22 D7
// FE 01 00 03 30 5C 17 06 00 07 07 00 00 A8 02 4B 00 03 00 18 00 13 00 00 07 FF FF FF FF FF 32 A1
// FE 01 00 03 30 5C 17 06 00 61 61 00 00 3D 1A 49 00 CB 00 13 00 11 00 00 07 FF FF FF FF FF E9 D5
//
// FE 01 00 03 30 5C 17 06 00 64 00 02 00 00 00 ... Brake 100%, 0rpm
// FE 01 00 03 30 5C 17 06 00 1E 20 01 00 20 03 ... Forward, Throttle 30%, 8000rpm
// FE 01 00 03 30 5C 17 06 00 12 0F 01 00 DF 01 ... Reverse, Throttle -18%, 4800rpm
//
// Byte 0: Startbyte is always 0xFE
// Byte 01-07: First 4 bytes are maybe control bytes. A 0x01000330 marks a telemetry data package.
// Byte 9:  Throttle input value from receiver.
// Byte 10: Throttle output value from ESC. Maybe different to throttle input because of ESC settings. For example: max reverse force.
// Byte 11: Direction value. Is 0x00 if neutral, 0x01 if forward or reverse and 0x02 if brake.
// Byte 13 and 14: RPM value. Needs to be multiplicated by 10. Maybe needs another multiplicator if motor has more then 2 poles.
// Byte 15: Battery voltage. Needs to be divided by 10.
// Byte 17 and 18: Current. Needs to be divided by 10.
// Byte 19: ESC Temperature.
// Byte 21: Motor Temperature.
// Byte 30 and 31: Checksum.
//
// Used CRC RevEng to find out what they used for checksum calculation
// reveng -w 16 -s FE010003305C170600070700004A034B000400180013000007FFFFFFFFFF10E0 FE010003305C1706000808000028034B000400180013000007FFFFFFFFFF329F FE010003305C170600070700004E024B000300180013000007FFFFFFFFFF98F6 FE010003305C1706000000000038064B000000130011000007FFFFFFFFFFA6C8
// width=16  poly=0x8005  init=0xffff  refin=true  refout=true  xorout=0x0000  check=0x4b37  residue=0x0000  name="CRC-16/MODBUS"

HWTelemetryClass HWTelemetry;

HWTelemetryClass::HWTelemetryClass(void)
{

}

HWTelemetryClass::~HWTelemetryClass(void)
{

}

void HWTelemetryClass::begin(void)
{
    Serial.begin(115200);
    begin(Serial);
}

void HWTelemetryClass::begin(Stream &s)
{
    HWTelemetryStream = &s;
    while(HWTelemetryStream->available())
        HWTelemetryStream->read();
}



// Nouveau processInput non bloquant


void HWTelemetryClass::processInput(void)
{

    // Safety check
    if (HWTelemetryStream == nullptr) {
    return;
    }

    static bool syncing = false;
    static uint8_t buffer[HWT_PACKAGE_SIZE];
    static uint8_t index = 0;

    while (HWTelemetryStream->available()) {
        uint8_t b = HWTelemetryStream->read();

        if (!syncing) {
            if (b == HWT_PACKAGE_HEADER) {
                syncing = true;
                index = 0;
                buffer[index++] = b;
            }
        } else {
            buffer[index++] = b;

            if (index == HWT_PACKAGE_SIZE) {
                syncing = false;
                index = 0;

                // Vérifier les bytes de contrôle
                uint32_t ctrlBytes = 0;
                ctrlBytes |= buffer[1];
                ctrlBytes |= buffer[2] << 8;
                ctrlBytes |= buffer[3] << 16;
                ctrlBytes |= buffer[4] << 24;

                // Vérifier le CRC
                uint16_t checksum = 0;
                checksum |= buffer[30];
                checksum |= buffer[31] << 8;

                if (ctrlBytes == 0x30030001 &&
                    checksum == calcCRC16(buffer, HWT_PACKAGE_SIZE - 2,
                                          CRC16_MODBUS_POLYNOME,
                                          CRC16_MODBUS_INITIAL,
                                          CRC16_MODBUS_XOR_OUT,
                                          CRC16_MODBUS_REV_IN,
                                          CRC16_MODBUS_REV_OUT,
                                          CRC_YIELD_DISABLED)) {

                    // Trame valide → Mettre à jour les valeurs
                    throttleIn = buffer[9];
                    throttleOut = buffer[10];
                    direction = buffer[11];

                    rpm = buffer[13] | (buffer[14] << 8);
                    // voltage = buffer[15];   ancien sur un octet
					voltage = buffer[15] | (buffer[16] << 8); // Combine l'octet 15 et 16
                    // current = buffer[17];  ancien sur un octet
					current = buffer[17] | (buffer[18] << 8); // Combine l'octet 17 et 18
                    escTemp = buffer[19] | (buffer[20] << 8);
                    motorTemp = buffer[21] | (buffer[22] << 8);
					becCurrent = buffer[23]; // <-- 1. AJOUTEZ CETTE LIGNE

                    if (currentCallback) currentCallback();
                }

                // Sinon : trame invalide → on garde les anciennes valeurs
            }
        }
    }
}




uint8_t HWTelemetryClass::getThrottle(bool input)
{
    uint8_t throttle = throttleIn;
    if(input)
        throttle = throttleOut;

    return throttle;
}

uint8_t HWTelemetryClass::getDirection(void)
{
    return direction;
}

int16_t HWTelemetryClass::getRPM(void)
{
    return rpm * 10 / (motorPoles / 2);
}


float HWTelemetryClass::getVoltage(void)
{
    return voltage / 10.0;
}

float HWTelemetryClass::getCurrent(void)
{
    return current / 10.0;
}


float HWTelemetryClass::getBECCurrent(void)
{
    return becCurrent / 10.0;
}

uint8_t HWTelemetryClass::getESCTemperature(void)
{
    return escTemp;
}

uint8_t HWTelemetryClass::getMotorTemperature(void)
{
    return motorTemp;
}

float HWTelemetryClass::getSpeed(void)
{
    float speed = ((rpm / gearRatio) * wheelSize * PI * 60) / 100000.0;
    return speed / (motorPoles / 2);
}

void HWTelemetryClass::setMotorPoles(uint8_t poles)
{
    if(poles < 2 || poles > 100) {
        return;
    }
    if((poles % 2) != 0) {
        return;
    }
    motorPoles = poles;
}

void HWTelemetryClass::setGearRatio(float ratio)
{
    if(ratio < 0.01 || ratio > 100.0) {
        return;
    }
    gearRatio = ratio;
}

void HWTelemetryClass::setWheelSize(uint16_t size)
{
    if(size < 10 || size > 1000) {
        return;
    }
    wheelSize = size;
}