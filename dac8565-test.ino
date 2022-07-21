#include <SPI.h>

// DAC pin default states
// 1: VoutA
// 2: VoutB
// 3: VrefOut: 150nF to GND
// 4: AVdd: 3v3 + 100nF decoupling cap to GND
// 5: VrefL: GND (NB! Does not look like this in the manual but it has to be GND even for internal ref)
// 6: GND
// 7: VoutC
// 8: VoutD
// 9: !SYNC: 0
// 10: SCLK
// 11: Din
// 12 IOVdd: 3v3
// 13: !RST: 1
// 14: RSTSEL: 0 (binary)
// 15: !ENABLE: 0
// 16: LDAC: 0

#define UPDATE_DACS 0b00100000
#define PIN_LDAC 0
#define PIN_MUX_EN0 1
#define PIN_MUX_EN1 2
#define PIN_MUX_A0 3
#define PIN_MUX_A1 4
#define PIN_MUX_A2 5
#define PIN_SYNC 10
#define ENABLED LOW
#define DISABLED HIGH

uint32_t         txBuffer[4];
// At 3v3, the DAC is specced to not be able to run above 25MHz
// Testing on a breadboard showed that 45MHz works, but 50MHz does not.
SPISettings     spiSettingsDac(45000000, MSBFIRST, SPI_MODE0);

void setup() {
  pinMode (PIN_LDAC, OUTPUT);
  pinMode (PIN_MUX_EN0, OUTPUT);
  pinMode (PIN_MUX_EN1, OUTPUT);
  pinMode (PIN_MUX_A0, OUTPUT);
  pinMode (PIN_MUX_A1, OUTPUT);
  pinMode (PIN_MUX_A2, OUTPUT);
  
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial.println("SPI Starting");
  SPI.enableInterrupt(&spi_isr4);
  
  digitalWriteFast(PIN_LDAC,LOW);
  digitalWriteFast(PIN_MUX_EN0,LOW);
  digitalWriteFast(PIN_MUX_EN1,LOW);
  digitalWriteFast(PIN_MUX_A0,LOW);
  digitalWriteFast(PIN_MUX_A1,LOW);
  digitalWriteFast(PIN_MUX_A2,LOW);
  SPI.begin();  
}

uint16_t out = 0;
uint8_t currentCV = 0;

void spi_isr4(void) {
  // Check transfer complete interrupt
  if(SPI.hasInterruptFlagSet(LPSPI_SR_TCF)) {
    
    // clear interrupt
    SPI.clearInterruptFlag(LPSPI_SR_TCF);

    digitalWriteFast(PIN_LDAC, HIGH);  
    digitalWriteFast(PIN_LDAC, LOW);  
    /*
    // TODO: Do not do this in ISR, use timer.
    // Move to next output and load from DAC.
    if(currentCV < 8) {
      digitalWriteFast(PIN_MUX_EN0, DISABLED);  
    } else {
      digitalWriteFast(PIN_MUX_EN1, DISABLED);  
    }
    
    // Delay to make sure MUX output is turned off 
    delayNanoseconds(1000);    
    bitRead(currentCV, 0) == 0 ? digitalWriteFast(PIN_MUX_A0, LOW) : digitalWriteFast(PIN_MUX_A0, HIGH);
    bitRead(currentCV, 1) == 0 ? digitalWriteFast(PIN_MUX_A1, LOW) : digitalWriteFast(PIN_MUX_A1, HIGH);
    bitRead(currentCV, 2) == 0 ? digitalWriteFast(PIN_MUX_A2, LOW) : digitalWriteFast(PIN_MUX_A2, HIGH);
    bitRead(currentCV, 0) == 0 ? digitalWriteFast(PIN_MUX_A0, LOW) : digitalWriteFast(PIN_MUX_A0, HIGH);
        
    // - load DACs
    digitalWriteFast(PIN_LDAC,HIGH);
    digitalWriteFast(PIN_LDAC,LOW);

    // Delay to make sure DAC output has settled
    delayNanoseconds(1000); 
    if(currentCV < 8) {
      digitalWriteFast(PIN_MUX_EN0, ENABLED);      
    } else {
      digitalWriteFast(PIN_MUX_EN1, ENABLED);
    } 
    */
  }  
}

void loop() {
  out +=256;
  for(uint8_t i = 0; i < 4; i++) {
    txBuffer[i] = out | (i << 17);
    if(i == 3) {
      // TODO: Uncomment for SW LDAC after last dac has been updated
      
      // Update all dacs after all four have been written.
      // Should probably do this in the interrupt routine 
      // AFTER turning off MUX. This can be done with 
      // the hardware LDAC pin by bringing it high.
      //txBuffer[i] = txBuffer[i] | (UPDATE_DACS << 16);
    }
  }
  updateDacChannels();
  delayMicroseconds(10);
}

void updateDacChannels() {  
  // CS must be normally LOW and connected to !SYNC-pin of DAC
  //SPI.setCS(PIN_SYNC); 
  SPI.beginTransaction(spiSettingsDac);
  SPI.send24((void *)txBuffer, 4, PIN_SYNC);  
}
