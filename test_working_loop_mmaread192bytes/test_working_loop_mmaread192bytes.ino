#include <Wire.h>
#include <Adafruit_MMA8451.h>
Adafruit_MMA8451 mma = Adafruit_MMA8451();
void setup() {
  Serial.begin(9600);
  Wire.setClock(400000L);
  mma.begin();
  if (! mma.begin()) {
  Serial.println("Couldnt start");
  while (1);
  }
  Serial.println("MMA8451 found!");
  
  Serial.print("MMA8451_REG_CTRL_REG1");Serial.println(MMA8451_REG_CTRL_REG1);
  //mma.setRange(MMA8451_RANGE_2_G);
  mma.getRange();
 uint8_t oa = mma.getFifostatus();
    Serial.print("FIFO Status :  "); Serial.print(oa);
 uint8_t ob = mma.getFifosetup();
 Serial.print("  FIFO Setup :  ");Serial.print(ob);
  mma.writeRegister8(0x2a, 0x14); // CRTL_REG1: 100 Hz mode DR = 010,LNOISE =1 F_Read = 0, Active = 0,
  Serial.print(" post write 0x14 Ctrlreg1 data :  "); Serial.println(mma.getCtrlreg1(),HEX); 
  mma.writeRegister8(0x09, 0x80); // FIFO Set to Fill Mode
  mma.writeRegister8(0x2D, 0x40); // Enable the interrupt Pin for the FIFO
  mma.writeRegister8(0x2E, 0x40); // Set the interrupt to route to INT1
  mma.writeRegister8(0x0E, 0x00); //NO HPF_OUT set, 2g mode
  uint8_t oc = mma.getCtrlreg1();
  Serial.print(" pre oc Ctrlreg1 data :  "); Serial.println(mma.getCtrlreg1(),BIN);
  oc= oc| 0x01;
  mma.writeRegister8(0x2a, oc);
  delay(3000);
  Serial.print("  Ctrlreg1 data :  "); Serial.println(mma.getCtrlreg1(),BIN);

/**************************************************************************/
}
void loop() {
int8_t p =0;
int16_t x, y, z,q,r,s, fifoData[192],t1;
unsigned long t2;
//t1=millis();
Serial.print("FIFO Statusafter delay :  "); Serial.println(mma.getFifostatus());
t2=micros();
for (uint8_t i=0; i<32; i++) {
  //t2=micros();
mma.read();
//t2=micros() - t2;
  fifoData[p]=mma.x;
  p=p+1;
  fifoData[p]=mma.y;
  p=p+1;
  fifoData[p]=mma.z;
  p=p+1;
}
t2=micros() - t2;
//Serial.print("FIFO Status :  "); Serial.println(mma.getFifostatus());
Serial.print("192 byte time:");Serial.print(t2);Serial.println("us");
for (uint8_t p=0; p<96; p++) {
if (p %12 == 0){
//Serial.println("");Serial.print("FIFO Status :  "); Serial.print(mma.getFifostatus());Serial.println("");
}
//Serial.print(fifoData[p]);Serial.print(" ");
}
delay(125);

}
