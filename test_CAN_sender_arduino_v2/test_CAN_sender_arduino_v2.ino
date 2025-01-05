#include <SPI.h>
#include <mcp_can.h>

// Define o pino CS do módulo CAN
#define CAN_CS_PIN 10

#define data_lenght 8

// Inicializa o objeto CAN
MCP_CAN CAN(CAN_CS_PIN);

// Variáveis globais para ID e dados
uint16_t txId = 0x100;  // ID inicial da mensagem CAN
uint8_t txData[8];       // Dados da mensagem CAN

void setup() {
  Serial.begin(115200);  // Comunicação Serial para debug
  while (!Serial);

  // Inicializa o módulo CAN na velocidade de 500 kbps
  if (CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN module initialized successfully!");
  } else {
    Serial.println("Error initializing CAN module.");
    while (1);
  }

  // Define o modo de operação como normal
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN module in Normal Mode.");

  // Inicializa os dados para a mensagem CAN
  for (uint8_t i = 0; i < data_lenght; i++) {
    txData[i] = i;  // Preenche o array de dados com valores iniciais
  }
  /*txData[0] = 0xAA;
  txData[1] = 0x99;
  txData[2] = 0x55;*/
}

void loop() {
  // Enviar a mensagem CAN
  if (CAN.sendMsgBuf(txId, 0, data_lenght, txData) == CAN_OK) {
    Serial.print("Message sent! ID: 0x");
    Serial.print(txId, HEX);
    Serial.print(" Data: ");
    for (uint8_t i = 0; i < data_lenght; i++) {
      Serial.print(txData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Error sending message.");
  }

  // Incrementa o ID (volta para 0x100 após 0x7FF, limite de 11 bits)
  txId++;
  if (txId > 0x7FF) {
    txId = 0x100;
  }

  // Modifica os dados para a próxima mensagem
  for (uint8_t i = 0; i < data_lenght; i++) {
    txData[i] = (txData[i] + 1) & 0xFF;  // Incrementa cada byte (com overflow para 0x00 após 0xFF)
  }

  delay(10);  // Aguarde 1 segundo antes de enviar outra mensagem
}