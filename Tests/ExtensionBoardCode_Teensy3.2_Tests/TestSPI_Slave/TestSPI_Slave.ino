#include <DMAChannel.h>
#include <SPI.h>
#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x, BIN) //Serial.println(x,BIN)
#define SAMPLES 1
#define ChipSelectSlave 10

DMAMEM static uint16_t rx_buffer[SAMPLES];
DMAChannel rx(false);

void rxISR() {
  rx.clearInterrupt();
  asm volatile ("dsb");
  Serial.print("RX Interrupt --> Val = "); Serial.println(rx_buffer[0]);
}

bool initSPISlaveDMA() {
  rx.begin(true);
  rx.source((uint16_t &) LPSPI4_RDR);
  rx.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_RX);
  rx.attachInterrupt(rxISR);
  rx.interruptAtCompletion(); //TCD->CSR |= DMA_TCD_CSR_INTMAJOR;
  rx.destinationBuffer(rx_buffer, SAMPLES + 1);
  rx.enable();
  return 1;
}

bool initSPISlave() {
  LPSPI4_CR &= ~LPSPI_CR_MEN; //Modul ausschalten
  LPSPI4_CR = LPSPI_CR_RST; //Master Logic reset! (Control Register => Software Reset)
  LPSPI4_CR &=  ~LPSPI_CR_RST; //Master Logic reset! (Control Register => Software Reset)
  LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7); //16Bit Mode
  LPSPI4_DER = LPSPI_DER_RDDE; //RX DMA Request Enable
  LPSPI4_CR |= LPSPI_CR_MEN; //Enable SPI Module!
  return 1;
}

void setup() {
  Serial.begin(115200);
  SPI.begin();
  SPI.setCS(ChipSelectSlave);
  while (!Serial);

  Serial.println("Init SPI!");
  if (initSPISlave()) {
    Serial.println("SPI SLAVE init!");
  }
  if (initSPISlaveDMA()) {
    Serial.println("DMA Channel init!");
  }
}

void loop() {
  arm_dcache_delete(rx_buffer, SAMPLES); //delete Cache!
  Serial.println(LPSPI4_SR, BIN);
  delay(10);
 }
