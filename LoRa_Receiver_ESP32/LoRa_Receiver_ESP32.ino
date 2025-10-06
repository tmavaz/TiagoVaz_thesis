#include <SPI.h>

#define RFM95_CS    5
#define RFM95_RST   4
#define RFM95_DIO0  26

#define REG_FIFO             0x00
#define REG_OP_MODE          0x01
#define REG_FRF_MSB          0x06
#define REG_FRF_MID          0x07
#define REG_FRF_LSB          0x08
#define REG_FIFO_ADDR_PTR    0x0D
#define REG_FIFO_RX_BASE     0x0F
#define REG_IRQ_FLAGS        0x12
#define REG_RX_NB_BYTES      0x13
#define REG_PKT_RSSI_VALUE   0x1A
#define REG_RSSI_VALUE       0x1B
#define REG_MODEM_CONFIG1    0x1D
#define REG_MODEM_CONFIG2    0x1E
#define REG_PAYLOAD_LENGTH   0x22
#define REG_FIFO_RX_CURRENT  0x10
#define REG_VERSION          0x42

void writeReg(uint8_t addr, uint8_t value) {
  digitalWrite(RFM95_CS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(RFM95_CS, HIGH);
}

uint8_t readReg(uint8_t addr) {
  digitalWrite(RFM95_CS, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(RFM95_CS, HIGH);
  return value;
}

void resetLoRa() {
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
}

void setupLoRa() {
  resetLoRa();

  uint8_t version = readReg(REG_VERSION);
  Serial.print("Versão RFM95 (esperado 0x12): 0x");
  Serial.println(version, HEX);
  if (version != 0x12) {
    Serial.println("❌ SX1276 não encontrado.");
    while (1);
  }

  writeReg(REG_OP_MODE, 0x80); // Sleep + LoRa
  delay(10);

  // Frequência 868 MHz
  writeReg(REG_FRF_MSB, 0xD9);
  writeReg(REG_FRF_MID, 0x00);
  writeReg(REG_FRF_LSB, 0x00);

  // SF7, BW125kHz, CR 4/5
  writeReg(REG_MODEM_CONFIG1, 0x72);
  writeReg(REG_MODEM_CONFIG2, 0x74);

  // Payload length e FIFO
  writeReg(REG_PAYLOAD_LENGTH, 0x40);
  writeReg(REG_FIFO_RX_BASE, 0x00);
  writeReg(REG_FIFO_ADDR_PTR, 0x00);

  writeReg(REG_IRQ_FLAGS, 0xFF); // limpa flags

  writeReg(REG_OP_MODE, 0x85); // LoRa RX contínuo
  Serial.println("✅ RFM95W em modo recepção.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting...");

  pinMode(RFM95_CS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(RFM95_DIO0, INPUT);
  digitalWrite(RFM95_CS, HIGH);
  digitalWrite(RFM95_RST, HIGH);

  SPI.begin(18, 19, 23); // SCK, MISO, MOSI

  setupLoRa();
}

unsigned long lastRssiCheck = 0;

void loop() {
  uint8_t irqFlags = readReg(REG_IRQ_FLAGS);

  // Pacote recebido
  if (irqFlags & 0x40) {
    uint8_t fifoAddr = readReg(REG_FIFO_RX_CURRENT);
    writeReg(REG_FIFO_ADDR_PTR, fifoAddr);

    uint8_t length = readReg(REG_RX_NB_BYTES);
    char msg[65] = {0};
    for (uint8_t i = 0; i < length && i < 64; i++) {
      msg[i] = readReg(REG_FIFO);
    }

    int rssi = readReg(REG_PKT_RSSI_VALUE);
    rssi -= 157;

    Serial.print("Packet: ");
    Serial.println(msg);
    Serial.print("Packet's RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");

    writeReg(REG_IRQ_FLAGS, 0xFF);
  }

  // RSSI ambiente a cada 500 ms
  if (millis() - lastRssiCheck > 500) {
    lastRssiCheck = millis();
    int ambientRssi = readReg(REG_RSSI_VALUE);
    ambientRssi -= 157;
    Serial.print("Environment's RSSI: ");
    Serial.print(ambientRssi);
    Serial.println(" dBm");
  }
}
