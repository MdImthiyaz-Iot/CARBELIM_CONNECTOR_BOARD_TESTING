// ================= UART ================= // vesrion 1
HardwareSerial SensorSerial (PA10, PA9);
HardwareSerial RS485Serial(PA3, PA2);

// ================= RS485 =================
#define RS485_DE     PA8
#define RS485_BAUD   9600
#define MODBUS_ID    1

// ================= ANALOG =================
#define PH_PIN   A0
#define TDS_PIN  A1
#define TURB_PIN A4

// ================= pH CAL =================
#define PH4_VOLTAGE  2.23f
#define PH7_VOLTAGE  1.65f
float pH_slope     = (7.0f - 4.0f) / (PH7_VOLTAGE - PH4_VOLTAGE);
float pH_intercept = 7.0f - pH_slope * PH7_VOLTAGE;

// ================= AIR SENSOR =================
#define ZPHS_RESP_LEN 26
byte resp[26];
bool zphs_cmd_sent = false;
unsigned long zphs_cmd_time = 0;
#define ZPHS_WAIT_MS 150

float waterTemp = 25.0f;

// ================= MODBUS =================
uint16_t modbusRegs[18] = {0};

// ================= TIMING =================
unsigned long lastSensorRead = 0;
#define SENSOR_INTERVAL 2000

// ================= RS485 CONTROL =================
void rs485Transmit() {
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(100);
}

void rs485Receive() {
  RS485Serial.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE, LOW);
}

// ================= CRC =================
uint16_t modbusRTU_CRC(byte* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
      else crc >>= 1;
    }
  }
  return crc;
}

// ================= SEND RESPONSE =================
void sendModbusResponse(byte funcCode, int startReg, int numRegs) {
  int byteCount = numRegs * 2;
  byte response[50];

  response[0] = MODBUS_ID;
  response[1] = funcCode;
  response[2] = byteCount;

  for (int i = 0; i < numRegs; i++) {
    response[3 + i * 2]     = (modbusRegs[startReg + i] >> 8);
    response[3 + i * 2 + 1] = (modbusRegs[startReg + i] & 0xFF);
  }

  uint16_t crc = modbusRTU_CRC(response, 3 + byteCount);
  response[3 + byteCount]     = crc & 0xFF;
  response[3 + byteCount + 1] = crc >> 8;

  rs485Transmit();
  RS485Serial.write(response, 3 + byteCount + 2);
  rs485Receive();
}

// ================= HANDLE MODBUS =================
void handleModbus() {
  if (RS485Serial.available() < 8) return;

  byte req[8];
  RS485Serial.readBytes(req, 8);

  if (req[0] != MODBUS_ID || req[1] != 0x03) return;

  uint16_t crcR = (req[7] << 8) | req[6];
  if (crcR != modbusRTU_CRC(req, 6)) return;

  int start = (req[2] << 8) | req[3];
  int count = (req[4] << 8) | req[5];

  if (start > 17 || start + count > 18) return;

  sendModbusResponse(0x03, start, count);
}

// ================= SENSOR CALC =================
float readVoltage(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) sum += analogRead(pin);
  return (sum / 10.0f) * (3.3f / 4095.0f);
}

float readPH(float v) {
  return constrain(pH_slope * v + pH_intercept, 0.0f, 14.0f);
}

float readTDS(float v) {
  float comp = 1.0f + 0.02f * (waterTemp - 25.0f);
  float cv   = v / comp;
  return (133.42f * cv * cv * cv - 255.86f * cv * cv + 857.39f * cv) * 0.5f;
}

float readTurbidity(float v) {
  float realV = v * 1.5f;
  if (realV < 2.5f) return 3000.0f;
  float ntu = -1120.4f * realV * realV + 5742.3f * realV - 4352.9f;
  return (ntu < 0.0f) ? 0.0f : ntu;
}

// ================= AIR SENSOR =================
void zphs_sendCommand() {
  byte cmd[9] = {0xFF,0x01,0x86,0,0,0,0,0,0x79};
  while (SensorSerial.available()) SensorSerial.read();
  SensorSerial.write(cmd, 9);
  zphs_cmd_sent = true;
  zphs_cmd_time = millis();
}

bool zphs_readResponse() {
  if (!zphs_cmd_sent) return false;
  if (millis() - zphs_cmd_time < ZPHS_WAIT_MS) return false;

  zphs_cmd_sent = false;

  if (SensorSerial.available() < ZPHS_RESP_LEN) return false;

  for (int i = 0; i < ZPHS_RESP_LEN; i++)
    resp[i] = SensorSerial.read();

  if (resp[0] != 0xFF || resp[1] != 0x86) return false;

  byte sum = 0;
  for (int j = 1; j <= 24; j++) sum += resp[j];
  if (((~sum) + 1) != resp[25]) return false;

  return true;
}

// ================= UPDATE MODBUS =================
void updateModbus(float turb, float ph, float tds) {
  modbusRegs[0] = turb * 10;
  modbusRegs[1] = ph * 100;
  modbusRegs[2] = tds * 10;

  if (resp[0] == 0xFF && resp[1] == 0x86) {
    modbusRegs[3]  = (resp[2]<<8)|resp[3];
    modbusRegs[4]  = (resp[4]<<8)|resp[5];
    modbusRegs[5]  = (resp[6]<<8)|resp[7];
    modbusRegs[6]  = (resp[8]<<8)|resp[9];
    modbusRegs[7]  = resp[10];
    modbusRegs[8]  = (((resp[11]<<8)|resp[12])-500);
    modbusRegs[9]  = (resp[13]<<8)|resp[14];
    modbusRegs[10] = (resp[15]<<8)|resp[16];
    modbusRegs[11] = (resp[17]<<8)|resp[18];
    modbusRegs[12] = (resp[19]<<8)|resp[20];
    modbusRegs[13] = (resp[21]<<8)|resp[22];
  }
}

// ================= SETUP =================
void setup() {
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  SensorSerial.begin(9600);
  RS485Serial.begin(RS485_BAUD);
  analogReadResolution(12);
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  handleModbus();

  if (zphs_readResponse()) {}

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    zphs_sendCommand();

    float phV   = readVoltage(PH_PIN);
    float tdsV  = readVoltage(TDS_PIN);
    float turbV = readVoltage(TURB_PIN);

    float ph   = readPH(phV);
    float tds  = readTDS(tdsV);
    float turb = readTurbidity(turbV);

    updateModbus(turb, ph, tds);
  }
}