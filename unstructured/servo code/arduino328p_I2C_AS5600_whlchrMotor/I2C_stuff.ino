/*
AS5600 I2C magentic encoder
todo:
disable WDT (if angle is (approx.) constant for 1 minute, it goes into a low power state
disable hysteresis?
set OUT bits to 00 (0-100% analog, but with hysteresis!), but dont use it
(set filter modes, if using ADC)
make sure not to burn any settings in
set power mode to 00 (normal)
check STATUS register (to see if magnet is present/weak/overpowering)
address = 0x36 == 54 = 0b00110110; (to be bitshifted<<1 when put into SLA_RW)
read out RAW ANGLE register

after some testing, i can confirm that, although the memory address pointer thingy advances every read cycle, 
 the RAW_ANGLE (and ANGLE?) registers make an exception to this, by jumping back from the Lower byte to the Higher byte (instead of moving to the next register)
 to be clear, the pointer will go RAW_ANGLE_H->RAW_ANGLE_L->RAW_ANGLE_H, repeat indefinetly
                       instead of RAW_ANGLE_H->RAW_ANGLE_L->ANGLE_H->ANGLE_L->invalid_sector
the pointer back-jump is done immedietly, even in the middle of a read cycle, 
 so if you read 4 bytes starting from the RAW_ANGLE_H register, you will receive RAW_ANGLE twice.
in theory, one could have an infinitely long read cycle, where you just keep reading more and more bytes.
the sensor value actualy does update in the middle of a read cycle (even at 800kHz), shown in this single 10 value (20 byte) read: 59 59 59 59 60 60 60 60 60 61
from some (very limited) testing, it seems to update the sensor value every 5 values at 800kHz I2C, which means ~65us between measurements
so i'm guessing a sampling rate of 16Khz  (at a slow-filter rate of 00==16x)

nov update: i took another look at it, and the datasheet claims a 150us sampling rate, which the data seems to reflect.
 I think i measured the sampling rate with my oscilloscope last time, not sure. All i know is, when i look at 15 measurements retrieved in ~440us,
 the same values are repeated 4~5 times, which makes ~5*30 = 150us.
 It sucks that the sampling rate is so limited, but this project was never meant to turn the motors that quickly (more about accurately).
 also, polling a single sensor value (at 800kHz I2C) takes like 72us, which leaves a lovely 70us to do some calulations in (or perhaps even leaves room for sensor interpolation???)
 i guess we'll see what the ESP32 makes of it (remember, that's what this shit will eventually need to run on)

*/

//// AS5600 constants:
//configuration registers:
#define AS5600_ZMCO 0x00
#define AS5600_ZPOS_H 0x01
#define AS5600_ZPOS_L 0x02
#define AS5600_MPOS_H 0x03
#define AS5600_MPOS_L 0x04
#define AS5600_MANG_H 0x05
#define AS5600_MANG_L 0x06
#define AS5600_CONF_A 0x07
#define AS5600_CONF_B 0x08
//output registers:
#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D
#define AS5600_ANGLE_H 0x0E
#define AS5600_ANGLE_L 0x0F
//status registers:
#define AS5600_STATUS 0x0B
#define AS5600_AGC 0x1A
#define AS5600_MAGNITUDE_H 0x1B
#define AS5600_MAGNITUDE_L 0x1C
//burn command:
#define AS5600_BURN 0xFF


const uint8_t slaveAddress = 54; //7-bit address

//// I2C constants:
#define TW_WRITE 0 //https://en.wikipedia.org/wiki/I%C2%B2C  under "Addressing structure"
#define TW_READ  1
const uint8_t twi_basic = (1<<TWINT) | (1<<TWEN); //any action will feature these 2 things (note TWEA is 0)
const uint8_t twi_START = twi_basic | (1<<TWSTA);
const uint8_t twi_STOP  = twi_basic | (1<<TWSTO);
const uint8_t twi_basic_ACK = twi_basic | (1<<TWEA); //(for master receiver mode) basic action, repond with ACK (if appropriate)
const uint8_t SLA_W = ( slaveAddress <<1) | TW_WRITE;
const uint8_t SLA_R = ( slaveAddress <<1) | TW_READ;

const uint8_t twi_SR_noPres = 0b11111000; //TWSR (status register) without prescaler bits
// status register contents (master mode)
const uint8_t twi_SR_M_START = 0x08;      //start condition has been transmitted
const uint8_t twi_SR_M_RESTART = 0x10;    //repeated start condition has been transmitted
const uint8_t twi_SR_M_SLA_W_ACK = 0x18;  //SLA+W has been transmitted, ACK received
const uint8_t twi_SR_M_SLA_W_NACK = 0x20; //SLA+W has been transmitted, NOT ACK received
const uint8_t twi_SR_M_DAT_T_ACK = 0x28;  //data has been transmitted, ACK received
const uint8_t twi_SR_M_DAT_T_NACK = 0x30; //data has been transmitted, NOT ACK received
const uint8_t twi_SR_M_arbit = 0x38;      //arbitration
const uint8_t twi_SR_M_SLA_R_ACK = 0x40;  //SLA+R has been transmitted, ACK received
const uint8_t twi_SR_M_SLA_R_NACK = 0x48; //SLA+R has been transmitted, NOT ACK received
const uint8_t twi_SR_M_DAT_R_ACK = 0x50;  //data has been received, ACK returned
const uint8_t twi_SR_M_DAT_R_NACK = 0x58; //data has been received, NOT ACK returned
// status register contents (slave mode)
const uint8_t twi_SR_S_SLA_W_ACK = 0x60;  //own address + W has been received, ACK returned
const uint8_t twi_SR_S_arbit_SLA_W = 0x68;//arbitration
const uint8_t twi_SR_S_GEN_ACK = 0x70;    //general call + W has been received, ACK returned
const uint8_t twi_SR_S_arbit_GEN = 0x78;  //arbitration
const uint8_t twi_SR_S_DAT_SR_ACK = 0x80; //data has been received after SLA+W, ACK returned
const uint8_t twi_SR_S_DAT_SR_NACK = 0x88;//data has been received after SLA+W, NOT ACK returned
const uint8_t twi_SR_S_DAT_GR_ACK = 0x90; //data has been received after GEN+W, ACK returned
const uint8_t twi_SR_S_DAT_GR_NACK = 0x98;//data has been received after GEN+W, NOT ACK returned
const uint8_t twi_SR_S_prem_STOP_RE =0xA0;//a STOP or repeated_START condition has been received prematurely (page 193)
const uint8_t twi_SR_S_SLA_R_ACK = 0xA8;  //own address + R has been received, ACK returned
const uint8_t twi_SR_S_arbit_SLA_R = 0xB0;//arbitration
const uint8_t twi_SR_S_DAT_ST_ACK = 0xB8; //data has been transmitted, ACK received     (master receiver wants more data)
const uint8_t twi_SR_S_DAT_ST_NACK = 0xC0;//data has been transmitted, NOT ACK received (master receiver doesnt want any more)
const uint8_t twi_SR_S_DAT_STL_ACK = 0xC8;//last (TWEA==0) data has been transmitted, ACK received (data length misconception)
// status register contents (miscellaneous states)
const uint8_t twi_SR_nothing = twi_SR_noPres; //(0xF8) no relevant state info, TWINT=0
const uint8_t twi_SR_bus_err = 0; //bus error due to an illigal start/stop condition (if this happens, set TWCR to STOP condition)

/*  what the ACK bit does (and what the status registers read if ACK is used wrong/unexpectedly)
after a START, in response to an address byte, the slave uses ACK if (TWEA=1) it accepts the communication in general
during data transferrence (either direction) the ACK/NOT-ACK is used to let the other side know whether or not they want more data
if the recipient sends an ACK, it expects more data
if the master transmitter ran out of data to send to the slave receiver, the slave status register will read 0xA0 (twi_SR_S_STOP_RESTART)
if the slave transmitter ran out of data to send to the master receiver, the slave status register will read 0xC8 (twi_SR_S_DAT_STL_ACK)
  in that case, the slave transmitter will send all 1's untill STOP (or RESTART)
in cases where there is too much data (from either side), the NOT-ACK will just be received earlier than expected
  if the slave sends NOT-ACK early, the master should STOP/RESTART the transmission (or the slave should ignore the overflowing data)
  if the master sends NOT-ACK early, the slave doesnt have to do anything (except maybe raise an error internally)

in general, the TWEA (Enable Ack) bit should be synchronized in both devices (except for master transmitter, which doesnt use it).
in master receiver, TWEA signals to the slave that the last byte is received, and the transmission will end
in both slave modes, if TWEA==0, the slave expects for there to be a STOP/RESTART next 'tick', if not, the status register will read 0 (twi_SR_bus_err)
*/

#define twoWireTransferWait   while(!(TWCR & (1<<TWINT)));
#define twoWireStatusReg      (TWSR & twi_SR_noPres)

void twoWireClockSetup(uint32_t frequency) {
  // set frequency (SCL freq = F_CPU / (16 + 2*TWBR*prescaler) , where prescaler is 1,8,16 or 64x, see page 200)
  TWSR &= 0b11111000; //set prescaler to 1x
  //TWBR  = 12; //set clock reducer to 400kHz (i recommend external pullups at this point)
  #define prescaler 1
  TWBR = ((F_CPU / frequency) - 16) / (2*prescaler);
  uint32_t reconstFreq = F_CPU / (16 + (2*TWBR*prescaler));
  //Serial.print("freq: "); Serial.print(frequency); Serial.print(" TWBR:"); Serial.print(TWBR); Serial.print(" freq: "); Serial.println(reconstFreq);
  // the fastest i could get I2C to work is 800kHz (with another arduino as slave at least), which is TWBR=2 (with some 1K pullups)
  // any faster and i get SLA_ACK errors.
}

uint8_t twoWireReadOneByte(uint8_t registerToRead) {
  TWCR = twi_START; //send start
  twoWireTransferWait;
  TWDR = SLA_W;
  TWCR = twi_basic; //send SLA_W
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { Serial.println("SLA_W ack error"); TWCR = twi_STOP; return(0); }
  TWDR = registerToRead;
  TWCR = twi_basic; //send data
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(0); } //should be ACK(?)
  TWCR = twi_START; //repeated start
  twoWireTransferWait;
  TWDR = SLA_R;
  TWCR = twi_basic; //send SLA_R
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_R_ACK) { Serial.println("SLA_R ack error"); TWCR = twi_STOP; return(0); }
  TWCR = twi_basic; //request 1 byte
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { Serial.println("DAT_R Nack error"); return(0); }
  uint8_t returnData = TWDR;
  TWCR = twi_STOP;
  return(returnData);
}

uint16_t twoWireReadTwoBytes(uint8_t registerToStartRead) {
  TWCR = twi_START; //send start
  twoWireTransferWait;
  TWDR = SLA_W;
  TWCR = twi_basic; //send SLA_W
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { Serial.println("SLA_W ack error"); TWCR = twi_STOP; return(0); }
  TWDR = registerToStartRead;
  TWCR = twi_basic; //send data
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(0); } //should be ACK(?)
  TWCR = twi_START; //repeated start
  twoWireTransferWait;
  TWDR = SLA_R;
  TWCR = twi_basic; //send SLA_R
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_R_ACK) { Serial.println("SLA_R ack error"); TWCR = twi_STOP; return(0); }
  TWCR = twi_basic_ACK; //request several bytes
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { Serial.println("DAT_R Nack error"); return(0); }
  uint16_t returnData = TWDR << 8;
  TWCR = twi_basic; //request 1 more byte
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { Serial.println("DAT_R Nack error"); return(0); }
  returnData |= TWDR;
  TWCR = twi_STOP;
  return(returnData);
}

void twoWireReadBytes(uint8_t registerToStartRead, uint8_t readBuff[], uint16_t bytesToRead) {
  TWCR = twi_START; //send start
  twoWireTransferWait;
  TWDR = SLA_W;
  TWCR = twi_basic; //send SLA_W
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { Serial.println("SLA_W ack error"); TWCR = twi_STOP; return; }
  TWDR = registerToStartRead;
  TWCR = twi_basic; //send data
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(0); } //should be ACK(?)
  TWCR = twi_START; //repeated start
  twoWireTransferWait;
  TWDR = SLA_R;
  TWCR = twi_basic; //send SLA_R
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_R_ACK) { Serial.println("SLA_R ack error"); TWCR = twi_STOP; return; }
  for(uint16_t i=0; i<(bytesToRead-1); i++) {
    TWCR = twi_basic_ACK; //request several bytes
    twoWireTransferWait;
    //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { Serial.println("DAT_R Nack error"); return; }
    readBuff[i] = TWDR;
  }
  TWCR = twi_basic; //request 1 more byte
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { Serial.println("DAT_R Nack error"); return; }
  readBuff[bytesToRead-1] = TWDR;
  TWCR = twi_STOP;
}

//uint16_t twoWireReadTwoBytes(uint8_t registerToStartRead) {
//  uint8_t readBuff[2];
//  twoWireReadBytes(registerToStartRead, readBuff, 2);
//  uint16_t returnVal = readBuff[0] << 8;
//  returnVal |= readBuff[1];
//  return(returnVal); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
//}

void twoWireOnlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
  TWCR = twi_START; //send start
  twoWireTransferWait;
  TWDR = SLA_R;
  TWCR = twi_basic; //send SLA_R
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_R_ACK) { Serial.println("SLA_R ack error"); TWCR = twi_STOP; return; }
  for(uint16_t i=0; i<(bytesToRead-1); i++) {
    TWCR = twi_basic_ACK; //request several bytes
    twoWireTransferWait;
    //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { Serial.println("DAT_R Nack error"); return; }
    readBuff[i] = TWDR;
  }
  TWCR = twi_basic; //request 1 more byte
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { Serial.println("DAT_R Nack error"); return; }
  readBuff[bytesToRead-1] = TWDR;
  TWCR = twi_STOP;
}

uint16_t twoWireOnlyReadTwoBytes() {
  uint8_t readBuff[2];
  twoWireOnlyReadBytes(readBuff, 2);
  uint16_t returnVal = readBuff[0] << 8;
  returnVal |= readBuff[1];
  return(returnVal); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
}


void twoWireWriteBytes(uint8_t registerToStartWrite, uint8_t dataToWrite[], uint8_t dataByteCount) {
  TWCR = twi_START; //send start
  twoWireTransferWait;
  TWDR = SLA_W;
  TWCR = twi_basic; //send SLA_W
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { Serial.println("SLA_W ack error"); TWCR = twi_STOP; return; }
  TWDR = registerToStartWrite;
  TWCR = twi_basic; //send data
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return; } //should be ACK(?)
  for(uint8_t i=0; i<dataByteCount; i++) {
    TWDR = dataToWrite[i];
    TWCR = twi_basic; //send data
    twoWireTransferWait;
    //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return; } //should be ACK(?)
  }
  TWCR = twi_STOP;
}

template<class T> void twoWireWriteData(uint8_t registerToStartWrite, T dataToWrite) { //arbitrary type T (so i dont have to write out all options)
  twoWireWriteBytes(registerToStartWrite, (uint8_t*) &dataToWrite, sizeof(dataToWrite));
}

void twoWireWriteSingleValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t dataByteCount) {
  TWCR = twi_START; //send start
  twoWireTransferWait;
  TWDR = SLA_W;
  TWCR = twi_basic; //send SLA_W
  twoWireTransferWait;
  if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { Serial.println("SLA_W ack error"); TWCR = twi_STOP; return; }
  TWDR = registerToStartWrite;
  TWCR = twi_basic; //send data
  twoWireTransferWait;
  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(0); } //should be ACK(?)
  for(uint8_t i=0; i<dataByteCount; i++) {
    TWDR = valueToWrite;
    TWCR = twi_basic; //send data
    twoWireTransferWait;
    //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(0); } //should be ACK(?)
  }
  TWCR = twi_STOP;
}

void twoWirePrintConfig() {
  uint8_t readBuff[9];
  twoWireReadBytes(0x00, readBuff, 9);
  //Serial.println("printing config:");
  Serial.print("burn count (ZMCO):"); Serial.println(readBuff[0] & 0b00000011);
  uint16_t bigVal = ((readBuff[1] & 0b00001111) << 8) | readBuff[2];
  Serial.print("start pos (ZPOS):"); Serial.println(bigVal);
  bigVal = ((readBuff[3] & 0b00001111) << 8) | readBuff[4];
  Serial.print("stop pos (MPOS):"); Serial.println(bigVal);
  bigVal = ((readBuff[5] & 0b00001111) << 8) | readBuff[6];
  Serial.print("max angle (MANG):"); Serial.println(bigVal);
  Serial.print("watchdog (WD):"); Serial.println((readBuff[7] & 0b00100000)>>5);
  Serial.print("fast filter threshold (FTH):"); Serial.println((readBuff[7] & 0b00011100)>>2);
  Serial.print("slow filter (SF):"); Serial.println(readBuff[7] & 0b00000011);
  Serial.print("PWM frequency (PWMF):"); Serial.println((readBuff[8] & 0b11000000)>>6);
  Serial.print("output stage (OUTS):"); Serial.println((readBuff[8] & 0b00110000)>>4);
  Serial.print("hysteresis (HYST):"); Serial.println((readBuff[8] & 0b00001100)>>2);
  Serial.print("power mode (PM):"); Serial.println(readBuff[8] & 0b00000011);
}

void twoWireResetConfig() {
  twoWireWriteSingleValue(0x01, 0, 8); //write zeroes to all configuration registers (reset to default)
}
