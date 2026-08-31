#include <WiFi.h>
#include <Arduino.h>
#include <HWTelemetry.h>
#include <AlfredoCRSF.h>

// Used for calculating program execution time. (Accurate battery usage calculations)
unsigned long  duration;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int ch[16];
    uint16_t RPM; //RPM
    float Voltage; //Volts
    float Current; //Amps
    float Speed;
    float BECCurrent;
    double BatteryCap;
    int16_t ESCTemperature; //deg_C
    int16_t MotorTemperature; //deg_C
    uint8_t ThrottleOut;
    uint8_t ThrottleIn;
    uint8_t Direction;
} struct_message;

// Create a struct_message called myData
struct_message myData;



HardwareSerial HWSerial(0);  // UART0
HardwareSerial CRSFSerial(1); //UART1
AlfredoCRSF crsf;


void hwtCallback() {
  myData.RPM = HWTelemetry.getRPM();
  myData.Voltage = HWTelemetry.getVoltage();
  myData.Current = HWTelemetry.getCurrent();
  myData.ESCTemperature = HWTelemetry.getESCTemperature();
  myData.MotorTemperature = HWTelemetry.getMotorTemperature();
  //myData.Speed = HWTelemetry.getSpeed();
  myData.BECCurrent = HWTelemetry.getBECCurrent();
}


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  // WiFi.mode(WIFI_STA);
  
  // UART0 sur GPIO3 (RX actif et TX desactivé avec -1 )
  HWSerial.begin(115200, SERIAL_8N1, RX, TX);
  HWSerial.setTimeout(50);

  HWTelemetry.begin(HWSerial);
  HWTelemetry.attach(hwtCallback);
  
  HWTelemetry.setMotorPoles(4);
  HWTelemetry.setGearRatio(5.7);
  HWTelemetry.setWheelSize(107);

  //UART1 for CRSF 
  CRSFSerial.begin(416666, SERIAL_8N1, 9, 10);
  if (!CRSFSerial) while (1) Serial.print("Invalid CRSFSerial configuration");
  crsf.begin(CRSFSerial);
  delay(100);
}

void sendChannels_ESPNOW()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    myData.ch[ChannelNum-1] = crsf.getChannel(ChannelNum);
    //Serial.print(crsf.getChannel(ChannelNum));
    //Serial.print(", ");

  }
  //Serial.println(" ");
}

void sendGps(float latitude, float longitude, float groundspeed, float heading, float altitude, float satellites)
{
  crsf_sensor_gps_t crsfGps = { 0 };

  // Values are MSB first (BigEndian)
  crsfGps.latitude = htobe32((int32_t)(latitude*10000000.0));
  crsfGps.longitude = htobe32((int32_t)(longitude*10000000.0));
  crsfGps.groundspeed = htobe16((uint16_t)(groundspeed*10.0));
  crsfGps.heading = htobe16((uint16_t)(heading*100.0)); //degrees * 100, so 0-360 degrees fits in 0-36000
  crsfGps.altitude = htobe16((uint16_t)(altitude + 1000.0));
  crsfGps.satellites = (uint8_t)(satellites);
  crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_GPS, &crsfGps, sizeof(crsfGps));
}

// Lets a connected handset set its clock from GPS time (requires ELRS 4.1+
// and EdgeTX 2.11+; older versions simply ignore the packet)
void sendGpsTime(int16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond)
{
  crsf_sensor_gps_time_t crsfGpsTime = { 0 };

  // Values are MSB first (BigEndian)
  crsfGpsTime.year = htobe16(year);
  crsfGpsTime.month = month;
  crsfGpsTime.day = day;
  crsfGpsTime.hour = hour;
  crsfGpsTime.minute = minute;
  crsfGpsTime.second = second;
  crsfGpsTime.millisecond = htobe16(millisecond);
  crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_GPS_TIME, &crsfGpsTime, sizeof(crsfGpsTime));
}

void sendBaroAltitude(float altitude, float verticalspd)
{
  crsf_sensor_baro_altitude_t crsfBaroAltitude = { 0 };

  // Values are MSB first (BigEndian)
  crsfBaroAltitude.altitude = htobe16((uint16_t)(altitude*10.0 + 10000.0));
  //crsfBaroAltitude.verticalspd = htobe16((int16_t)(verticalspd*100.0)); //TODO: fix verticalspd in BaroAlt packets
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BARO_ALTITUDE, &crsfBaroAltitude, sizeof(crsfBaroAltitude) - 2);
  
  //Supposedly vertical speed can be sent in a BaroAltitude packet, but I cant get this to work.
  //For now I have to send a second vario packet to get vertical speed telemetry to my TX.
  crsf_sensor_vario_t crsfVario = { 0 };

  // Values are MSB first (BigEndian)
  crsfVario.verticalspd = htobe16((int16_t)(verticalspd*100.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_VARIO, &crsfVario, sizeof(crsfVario));
}

// Temperatures are int16 in tenths of a degree Celsius, so 250 is 25.0C and
// -50 is -5.0C. sourceId 0 is conventionally the flight controller.
void sendTemperature(uint8_t sourceId, const int16_t *values, uint8_t count)
{
  if (count > CRSF_MAX_TEMP_VALUES)
    count = CRSF_MAX_TEMP_VALUES;

  uint8_t payload[1 + CRSF_MAX_TEMP_VALUES * 2];
  payload[0] = sourceId;

  for (uint8_t i = 0; i < count; i++)
  {
    // Values are MSB first (BigEndian)
    payload[1 + i*2]     = (values[i] >> 8) & 0xFF;
    payload[1 + i*2 + 1] = values[i] & 0xFF;
  }

  crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_TEMP, payload, 1 + count*2);
}


static void sendRxBattery(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe24((uint32_t)(capacity));        //mAh (24 bit field, max 16777215mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

// RPM values are signed 24 bit, so each one is packed as three bytes.
// Negative values mean the motor is spinning in reverse.
// sourceId identifies the group of motors, e.g. 0 for the first four ESCs.
void sendRpm(uint8_t sourceId, const int32_t *values, uint8_t count)
{
  if (count > CRSF_MAX_RPM_VALUES)
    count = CRSF_MAX_RPM_VALUES;

  uint8_t payload[1 + CRSF_MAX_RPM_VALUES * 3];
  payload[0] = sourceId;

  for (uint8_t i = 0; i < count; i++)
  {
    // Values are MSB first (BigEndian)
    payload[1 + i*3]     = (values[i] >> 16) & 0xFF;
    payload[1 + i*3 + 1] = (values[i] >> 8) & 0xFF;
    payload[1 + i*3 + 2] = values[i] & 0xFF;
  }

  crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RPM, payload, 1 + count*3);
}


// A single voltage in millivolts, shown on the radio as its own "Volt" sensor
// rather than as a cell of a pack. This is how an ELRS 4.0 receiver reports its
// measured voltage. It is a CELLS frame with the source ID at 128 or above,
// which the radio treats as a standalone voltage. voltageIndex 0, 1, 2... gives
// separate sensors, so you can report several batteries independently.
void sendVoltage(uint8_t voltageIndex, uint16_t millivolts)
{
  uint8_t payload[3];
  payload[0] = 128 + voltageIndex;
  payload[1] = (millivolts >> 8) & 0xFF; // MSB first (BigEndian)
  payload[2] = millivolts & 0xFF;

  crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_CELLS, payload, sizeof(payload));
}

void printMessages() {
        Serial.print(myData.RPM);
        Serial.print(" RPM / ");
        Serial.print(myData.Voltage);
        Serial.print(" V / ");
        Serial.print(myData.Current);
        Serial.print(" A / ");
        Serial.print(myData.ESCTemperature);
        Serial.print(" C / ");
        Serial.print(myData.MotorTemperature);
        Serial.print(" C / ");
        Serial.print(myData.BatteryCap);
        Serial.print(" mAh / ");
        Serial.print(duration);
        Serial.println(" (us)");
}

void loop() {
  unsigned long startTime = millis();
  unsigned long nowTime = micros();
  HWTelemetry.processInput();

  // Must call crsf.update() in loop() to process data
  //crsf.update();

  static uint32_t lastUpdate = 0;
      

  //Update @ 100Hz
  static unsigned long lastRPMUpdate = 0;
  if (startTime - lastRPMUpdate > 10)
  {
      lastRPMUpdate = startTime;
    // Four motors, the last one spinning in reverse
    int32_t motorRpm[1] = { myData.RPM };
    sendRpm(0, motorRpm, 1);      
  }

  //Update @ 10Hz
  static unsigned long lastTempUpdate = 0;
  if (startTime - lastTempUpdate > 100)
  {
    lastTempUpdate = startTime;
    // Flight controller and ambient temperature, in tenths of a degree C
    int16_t temperatures[2] = { myData.ESCTemperature, myData.MotorTemperature }; // 41.5C and 22.6C
    sendTemperature(0, temperatures, 2);
  } 

  //Update @ 8.3Hz
  static unsigned long lastGPSUpdate = 0;
  if (startTime - lastGPSUpdate > 120)
  {
    lastGPSUpdate = startTime;
    sendGps(42.12345, -82.12345, 20.5, 20.13, 690, 4);
    sendGpsTime(2026, 7, 14, 12, 34, 56, 789);
  }

  //sendBaroAltitude(verticalspd, verticalspd);

  //Update @ 100Hz
  static unsigned long lastRxBtUpdate = 0;
  if (startTime - lastRxBtUpdate > 10)
  {
    lastRxBtUpdate = startTime;
    myData.Voltage = millis()/1000; //seconds (for testing)
    sendRxBattery(myData.Voltage, myData.Current, myData.BatteryCap, 100);

    printMessages();
  }  


  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) 
  {
      lastPrint = millis();
      printMessages();
  }

  myData.BatteryCap +=((((myData.Current*1000.0f)*duration*1e-6f))/3600.0f); //mAh = mA x duration(s) / 1hr
  unsigned long endTime = micros();
  duration = endTime - nowTime;
}
