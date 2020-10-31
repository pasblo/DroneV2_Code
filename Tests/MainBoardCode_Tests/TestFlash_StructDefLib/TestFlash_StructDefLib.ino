#include<SPIMemory.h>

SPIFlash flash(10);

void setup() {
  Serial.begin(115200);
  delay(5000); //Time to terminal get connected
  flash.begin();

  struct Test {
    uint16_t s1;
    /*float s2;
    int32_t s3;
    bool s4;
    uint8_t s5;
    struct structOfStruct {
      uint8_t b1;
      float f2;
    } structofstruct;*/
  };
  Test _d;
  //Test _data;

  _d.s1 = 31325;
  /*_d.s2 = 4.84;
  _d.s3 = 880932;
  _d.s4 = true;
  _d.s5 = 5;
  _d.structofstruct.b1 = 234;
  _d.structofstruct.f2 = 6.28;*/

  uint32_t wTime = 0;
  uint32_t addr;

  addr = 0;

  if (flash.writeAnything(addr, _d)) {
    wTime = flash.functionRunTime();
  }
  Serial.println(wTime);
}

void loop() {
  // put your main code here, to run repeatedly:

}
