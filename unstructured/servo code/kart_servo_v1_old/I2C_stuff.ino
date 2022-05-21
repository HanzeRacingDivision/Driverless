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
const uint8_t SLA_W = ( slaveAddress <<1) | TW_WRITE;
const uint8_t SLA_R = ( slaveAddress <<1) | TW_READ;
#ifdef ARDUINO_ARCH_ESP32
  /*
  The ESP32 is both fantastically fast, and absolutely bogged down at the same time.
  The core itself runs wicked fast, so actual math is no problem.
  But interfacing with peripherals is often slow (because of the Real Time Operating System ?)
  Unfortunately, optimizing code by using low-level instructions is rarely easy, as there are always at least 3 'low' levels.
  In the case of I2C there is the arduino compatibility layer (top???): wire.begin(), etc.
  which is based on the HAL layer: i2cInit(), see esp32-hal-i2c.h in the core in Arduino15 in %appData%
  and below that there are some functions as well: i2c_driver_install() and i2c_master_cmd_begin(), see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
  I imagine there may be layer(s) even lower than that, but i can't find documentation.
  The code does not appear to be very fast (an atmega328p can do twoWireReadTwoBytes() in ~80us and the ESP needs ~128us)
   using twoWireOnlyReadTwoBytes is ~34us faster, so if you're willing to risk (your code requiring some thought), you may get away with that
  The recommended method is to use a i2c_cmd_handle_t, but that slows things down SO DAMN MUCH.
  There are some options for timing customizations, and i spotted a ~10us solid LOW gap in the SCL signal, so there may be some time to be saved there.
  
  note: the frequency is significantly lower than you set it. Add stronger pullups to SDA and SCL to increase speed closer to desired (but never quite there).
          adding 4.7K pullups makes it so 1MHz requested == 750kHz real
          the signal curves looked like they could do with even lower resistance pullups, in the AS5600 datasheet they vaguely mention 20mA "Logic low output current"
  
  TBD:
    success checking and error handling
    const command links instead of making new ones?
    DMA?
  */
  
  #include "driver/i2c.h"
  
  //// I2C constants:
  const i2c_port_t I2CportToUse = 0;
  const TickType_t I2CtimeoutTicks = 100 / portTICK_RATE_MS; //timeout (divide by portTICK_RATE_MS to convert millis to the right format)
  uint8_t constWriteBuff[1]; //i2c_master_write_read_device() requires a const uint8_t* writeBuffer. You can make this array bigger if you want, shouldnt really matter
  #define ACK_CHECK_EN 1
  #define ACK_CHECK_DIS 0
  
  esp_err_t twoWireSetup(uint32_t frequency, int SDApin, int SCLpin)
  {
    static i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDApin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = SCLpin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = frequency;
    //conf.clk_flags = 0;          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    esp_err_t err = i2c_param_config(I2CportToUse, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2CportToUse, conf.mode, 0, 0, 0);
  }
  
  uint8_t twoWireReadOneByte(uint8_t registerToRead) {
    uint8_t returnData;
  //  i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create a CMD sequence
  //  i2c_master_start(cmd);
  //  i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
  //  i2c_master_write_byte(cmd, registerToRead, ACK_CHECK_DIS);
  //  i2c_master_start(cmd);
  //  i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
  //  i2c_master_read_byte(cmd, &returnData, I2C_MASTER_NACK); //I2C_MASTER_LAST_NACK
  //  i2c_master_stop(cmd);
  //  esp_err_t err = i2c_master_cmd_begin(I2CportToUse, cmd, I2CtimeoutTicks);
  //  i2c_cmd_link_delete(cmd);
  
    constWriteBuff[0] = registerToRead;
    i2c_master_write_read_device(I2CportToUse, slaveAddress, constWriteBuff, 1, &returnData, 1, I2CtimeoutTicks); //faster (seems to work fine)
    return(returnData);
  }
  
  //uint16_t twoWireReadTwoBytes(uint8_t registerToStartRead) {  //see alternate function based on twoWireReadBytes()
  //  uint16_t returnData;
  //  byte* returnDataPointer = (byte*) &returnData;
  //  i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create a CMD sequence  (all of this manual command stuff takes a lot of time, like 50us i think)
  //  i2c_master_start(cmd);
  //  i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
  //  i2c_master_write_byte(cmd, registerToStartRead, ACK_CHECK_DIS);
  //  i2c_master_start(cmd);
  //  i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
  //  i2c_master_read_byte(cmd, returnDataPointer+1, I2C_MASTER_ACK); //HIGH side of 16bit int first
  //  i2c_master_read_byte(cmd, returnDataPointer, I2C_MASTER_NACK);  //LOW side second
  //  i2c_master_stop(cmd);
  //  esp_err_t err = i2c_master_cmd_begin(I2CportToUse, cmd, I2CtimeoutTicks); //this takes like 120us (depends heavily on frequency, which depends on pullup resistors)
  //  i2c_cmd_link_delete(cmd); //this takes like 30us
  //}
  
  void twoWireReadBytes(uint8_t registerToStartRead, uint8_t readBuff[], uint16_t bytesToRead) {
  //  i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create a CMD sequence
  //  i2c_master_start(cmd);
  //  i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
  //  i2c_master_write_byte(cmd, registerToStartRead, ACK_CHECK_DIS);
  //  i2c_master_start(cmd);
  //  i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
  //  i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
  //  i2c_master_stop(cmd);
  //  int success = i2c_master_cmd_begin(I2CportToUse, cmd, I2CtimeoutTicks);
  //  i2c_cmd_link_delete(cmd);
  
    constWriteBuff[0] = registerToStartRead;
    esp_err_t err = i2c_master_write_read_device(I2CportToUse, slaveAddress, constWriteBuff, 1, readBuff, bytesToRead, I2CtimeoutTicks); //faster (seems to work fine)
  }
  
  uint16_t twoWireReadTwoBytes(uint8_t registerToStartRead) {
    uint8_t readBuff[2];
    twoWireReadBytes(registerToStartRead, readBuff, 2);
    uint16_t returnData = readBuff[0] << 8;
    returnData |= readBuff[1];
    return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
  }
  
  void twoWireOnlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
  //  i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create a CMD sequence
  //  i2c_master_start(cmd);
  //  i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
  //  i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
  //  i2c_master_stop(cmd);
  //  esp_err_t err = i2c_master_cmd_begin(I2CportToUse, cmd, I2CtimeoutTicks);
  //  i2c_cmd_link_delete(cmd);
  
    esp_err_t err = i2c_master_read_from_device(I2CportToUse, slaveAddress, readBuff, bytesToRead, I2CtimeoutTicks);  //faster?
  }
  
  
  void twoWireWriteBytes(uint8_t registerToStartWrite, uint8_t dataToWrite[], uint8_t dataByteCount) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create a CMD sequence
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, registerToStartWrite, ACK_CHECK_DIS);
    i2c_master_write(cmd, dataToWrite, dataByteCount, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2CportToUse, cmd, I2CtimeoutTicks);
    i2c_cmd_link_delete(cmd);
  }
  
  void twoWireWriteSingleValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t dataByteCount) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create a CMD sequence
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, registerToStartWrite, ACK_CHECK_DIS);
    // one way
    for(uint8_t i=0; i<dataByteCount; i++) {
      i2c_master_write_byte(cmd, valueToWrite, ACK_CHECK_DIS);
    }
    // an alternate way:
  //  uint8_t singleValueArray[dataByteCount];  for(uint8_t i=0;i<dataByteCount;i++) { singleValueArray[i]=valueToWrite; }
  //  i2c_master_write(cmd, singleValueArray, dataByteCount, ACK_CHECK_DIS);
    // either way should (probably) work, the difference is in command size i guess
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2CportToUse, cmd, I2CtimeoutTicks);
    i2c_cmd_link_delete(cmd);
  }

#else //328p arduino I2C
  //// I2C constants:
  const uint8_t twi_basic = (1<<TWINT) | (1<<TWEN); //any action will feature these 2 things (note TWEA is 0)
  const uint8_t twi_START = twi_basic | (1<<TWSTA);
  const uint8_t twi_STOP  = twi_basic | (1<<TWSTO);
  const uint8_t twi_basic_ACK = twi_basic | (1<<TWEA); //(for master receiver mode) basic action, repond with ACK (if appropriate)
  
  const uint8_t twi_SR_noPres = 0b11111000; //TWSR (stas register) without prescalebits
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
  //  uint16_t returnData = readBuff[0] << 8;
  //  returnData |= readBuff[1];
  //  return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
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
#endif //328p arduino I2C

uint16_t twoWireOnlyReadTwoBytes() {
  uint8_t readBuff[2];
  twoWireOnlyReadBytes(readBuff, 2);
  uint16_t returnData = readBuff[0] << 8;
  returnData |= readBuff[1];
  return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
}

template<class T> void twoWireWriteData(uint8_t registerToStartWrite, T dataToWrite) { //arbitrary type T (so i dont have to write out all options)
  twoWireWriteBytes(registerToStartWrite, (uint8_t*) &dataToWrite, sizeof(dataToWrite));
}

void twoWirePrintConfig() {
  uint8_t readBuff[9];
  twoWireReadBytes(AS5600_ZMCO, readBuff, 9);
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
  twoWireWriteSingleValue(AS5600_ZPOS_H, 0, 8); //write zeroes to all configuration registers (reset to default)
}
