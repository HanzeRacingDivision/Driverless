

//https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf

#ifndef AS5600
#define AS5600

#include "Arduino.h"
#ifdef ARDUINO_ARCH_ESP32
  #include "driver/i2c.h"
#endif

//#define AS5600_unlock_burning   //enable the chip-burning features of the AS5600
//#define AS5600_return_esp_err   //return functions as esp_err_t types instead of booleans for slightly more user-controlled debugging.

//#define AS5600debugPrint(x)  Serial.println(x)
//#define AS5600debugPrint(x)  log_d(x)   //using the ESP32 debug printing

#ifndef AS5600debugPrint
  #define AS5600debugPrint(x)  ;
#endif

#ifndef ARDUINO_ARCH_ESP32
   #undef AS5600_return_esp_err
#endif
#ifdef AS5600_return_esp_err
  #define AS5600_ERR_RETURN_TYPE  esp_err_t
#else
  #define AS5600_ERR_RETURN_TYPE  bool
#endif


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

#define LSB_NIBBLE 0x0F    //(nibble==4bits) the AS5600 never uses values larger than 12bits, but sometimes inserts bits in the MSB of other registers

#define AS5600_ZMCO_bits 0b00000011
#define AS5600_WD_bits   0b00100000
#define AS5600_FTH_bits  0b00011100
#define AS5600_SF_bits   0b00000011
#define AS5600_PWMF_bits 0b11000000
#define AS5600_OUTS_bits 0b00110000
#define AS5600_HYST_bits 0b00001100
#define AS5600_PM_bits   0b00000011
#define AS5600_MD_bits   0b00100000
#define AS5600_ML_bits   0b00010000
#define AS5600_MH_bits   0b00001000
#define AS5600_BURN_ANGLE_bits  0x80
#define AS5600_BURN_SETTING_bits  0x40


#define TW_WRITE 0 //https://en.wikipedia.org/wiki/I%C2%B2C  under "Addressing structure"
#define TW_READ  1

#define ACK_CHECK_EN 1  //only for ESP32
#define ACK_CHECK_DIS 0 //ack check disable does not seem to work??? (it always checks for an ack and spits out )

#define SIZEOF_I2C_CMD_DESC_T  20  //a terrible fix for a silly problem. The actual struct/typedef code is too far down the ESP32 dependancy rabbithole.
#define SIZEOF_I2C_CMD_LINK_T  20  //if the ESP32 didn't have such a fragmented filestructure this wouldn't be a problem, maybe
/* the reason why i need those 2 defines:   //this code was derived from https://github.com/espressif/esp-idf/blob/master/components/driver/i2c.c
uint8_t buffer[sizeof(i2c_cmd_desc_t) + sizeof(i2c_cmd_link_t) * numberOfOperations] = { 0 };
i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
*/

class AS5600
{
  public:
    volatile bool keepLooping = true; //used in loopReadWithCallback
  //private:
    //// I2C constants:
    const uint8_t slaveAddress = 54; //7-bit address
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
    there is a layer below that: https://github.com/espressif/esp-idf/blob/master/components/driver/i2c.c
    but it's badly documented, hard to read and very RTOS-optimized. I don't think i'm quite ready to unravel that thread (for a few microseconds of extra speed)
    
    The code does not appear to be very fast (an atmega328p can do twoWireReadTwoBytes() in ~80us and the ESP needs ~128us)
    using twoWireOnlyReadTwoBytes is ~34us faster, so if you're willing to risk (your code requiring some thought), you may get away with that
    The recommended method is to use a i2c_cmd_handle_t, but that slows things down SO DAMN MUCH.
    There are some options for timing customizations, and i spotted a ~10us solid LOW gap in the SCL signal, so there may be some time to be saved there.
    
    note: the frequency is significantly lower than you set it. Add stronger pullups to SDA and SCL to increase speed closer to desired (but never quite there).
            adding 4.7K pullups makes it so 1MHz requested == 750kHz real
            the signal curves looked like they could do with even lower resistance pullups, in the AS5600 datasheet they vaguely mention 20mA "Logic low output current"
    
    TBD:
        returning esp_err_t instead of boolean for most functions (while maintaining compatiblity with arduino.?.)
        const command links instead of making new ones?
        the ESP32 documentation states you "have to" delete them, but i tested it and you can reuse a command just fine...
          (note: after switching to static CMDs, it seems that constant/predefined commands would (at most!) save about 6us, while making the library a whole lot stupider)
    */
    
    public:
    //// I2C constants:
    i2c_port_t I2Cport = 0;
    uint32_t I2Ctimeout = 10; //in millis
    //const TickType_t I2CtimeoutTicks = 100 / portTICK_RATE_MS; //timeout (divide by portTICK_RATE_MS to convert millis to the right format)
    //uint8_t constWriteBuff[1]; //i2c_master_write_read_device() requires a const uint8_t* writeBuffer. You can make this array bigger if you want, shouldnt really matter
    
    esp_err_t init(uint32_t frequency, int SDApin, int SCLpin, i2c_port_t I2CportToUse = 0) {
      if(I2CportToUse < I2C_NUM_MAX) { I2Cport = I2CportToUse; } else { AS5600debugPrint("can't init(), invalid I2Cport!"); return(ESP_ERR_INVALID_ARG); }
      static i2c_config_t conf;
      conf.mode = I2C_MODE_MASTER;
      conf.sda_io_num = SDApin;
      conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
      conf.scl_io_num = SCLpin;
      conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
      conf.master.clk_speed = frequency;
      //conf.clk_flags = 0;          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
      esp_err_t err = i2c_param_config(I2Cport, &conf);
      if (err != ESP_OK) { AS5600debugPrint("can't init(), i2c_param_config error!"); AS5600debugPrint(esp_err_to_name(err)); return(err); }
      return(i2c_driver_install(I2Cport, conf.mode, 0, 0, 0));
    }
    
    uint8_t requestReadByte(uint8_t registerToRead) {
      uint8_t returnData;
//      const uint8_t numberOfCommands = 7; //start, write, write, start, write, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToRead, ACK_CHECK_DIS);
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read_byte(cmd, &returnData, I2C_MASTER_NACK); //I2C_MASTER_LAST_NACK
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      //constWriteBuff[0] = registerToRead;
      esp_err_t err = i2c_master_write_read_device(I2Cport, slaveAddress, &registerToRead, 1, &returnData, 1, I2Ctimeout / portTICK_RATE_MS); //faster (seems to work fine)
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      return(returnData);
    }
    
//    uint16_t requestReadInt(uint8_t registerToStartRead) {  //see alternate function based on twoWireReadBytes()
//      uint16_t returnData;
//      byte* returnDataPointer = (byte*) &returnData;
//      const uint8_t numberOfCommands = 8; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToStartRead, ACK_CHECK_DIS);
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read_byte(cmd, returnDataPointer+1, I2C_MASTER_ACK); //HIGH side of 16bit int first
//      i2c_master_read_byte(cmd, returnDataPointer, I2C_MASTER_NACK);  //LOW side second
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);
//      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
//      return(returnData);
//    }
    
    AS5600_ERR_RETURN_TYPE requestReadBytes(uint8_t registerToStartRead, uint8_t readBuff[], uint16_t bytesToRead) {
//      const uint8_t numberOfCommands = 8; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToStartRead, ACK_CHECK_DIS);
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      //constWriteBuff[0] = registerToStartRead;
      esp_err_t err = i2c_master_write_read_device(I2Cport, slaveAddress, &registerToStartRead, 1, readBuff, bytesToRead, I2Ctimeout / portTICK_RATE_MS); //faster (seems to work fine)
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    AS5600_ERR_RETURN_TYPE onlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
//      const uint8_t numberOfCommands = 5; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      esp_err_t err = i2c_master_read_from_device(I2Cport, slaveAddress, readBuff, bytesToRead, I2Ctimeout / portTICK_RATE_MS);  //faster?
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    
    AS5600_ERR_RETURN_TYPE writeBytes(uint8_t registerToStartWrite, uint8_t dataToWrite[], uint8_t dataByteCount) {
      const uint8_t numberOfCommands = 5; //start, write, write, write, stop
      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
      i2c_master_write_byte(cmd, registerToStartWrite, ACK_CHECK_DIS);
      i2c_master_write(cmd, dataToWrite, dataByteCount, ACK_CHECK_DIS);
      i2c_master_stop(cmd);
      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
      i2c_cmd_link_delete_static(cmd);
      // probably slightly slower:
//      uint8_t copiedArray[dataByteCount+1]; copiedArray[0]=registerToStartWrite; for(uint8_t i=0;i<dataByteCount;i++) { copiedArray[i+1]=dataToWrite[i]; }
//      esp_err_t err = i2c_master_write_to_device(I2Cport, slaveAddress, copiedArray, dataByteCount+1, I2Ctimeout / portTICK_RATE_MS);
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    AS5600_ERR_RETURN_TYPE writeSameValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t dataByteCount) {
//      const uint8_t numberOfCommands = 5; //start, write, write, write, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToStartWrite, ACK_CHECK_DIS);
//      uint8_t singleValueArray[dataByteCount];  for(uint8_t i=0;i<dataByteCount;i++) { singleValueArray[i]=valueToWrite; }
//      i2c_master_write(cmd, singleValueArray, dataByteCount, ACK_CHECK_DIS);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);
      // another way:
//      uint8_t singleValueArray[dataByteCount];  for(uint8_t i=0;i<dataByteCount;i++) { singleValueArray[i]=valueToWrite; }
//      return(writeBytes(registerToStartWrite, singleValueArray, dataByteCount));
      // yet another way:
      uint8_t singleValueArray[dataByteCount+1]; singleValueArray[0]=registerToStartWrite; for(uint8_t i=0;i<dataByteCount;i++) { singleValueArray[i+1]=valueToWrite; }
      esp_err_t err = i2c_master_write_to_device(I2Cport, slaveAddress, singleValueArray, dataByteCount+1, I2Ctimeout / portTICK_RATE_MS);
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }

  #else //328p arduino I2C

    private:
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

    /*  what the ACK bit does (and what the status registers read if ACK is used wrong/unexpectedly):
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
    
    inline void twoWireTransferWait() { while(!(TWCR & (1<<TWINT))); }
    #define twoWireStatusReg      (TWSR & twi_SR_noPres)

    inline void twiWrite(uint8_t byteToWrite) {
      TWDR = byteToWrite;
      TWCR = twi_basic; //initiate transfer
      twoWireTransferWait();
    }

//    inline uint8_t twiReadAck(bool ack) {
//      TWCR = twi_basic_ACK; //request several bytes
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(0); }
//      return(TWDR);
//    }
//
//    inline uint8_t twiRead(bool ack) {
//      TWCR = twi_basic; //request several bytes
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(0); }
//      return(TWDR);
//    }
    
    inline bool startWrite() {
      TWCR = twi_START; //send start
      twoWireTransferWait();
      twiWrite(SLA_W);
      if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { AS5600debugPrint("SLA_W ack error"); TWCR = twi_STOP; return(false); }
      return(true);
    }

    inline bool startRead() {
      TWCR = twi_START; //repeated start
      twoWireTransferWait();
      twiWrite(SLA_R);
      if(twoWireStatusReg != twi_SR_M_SLA_R_ACK) { AS5600debugPrint("SLA_R ack error"); TWCR = twi_STOP; return(false); }
      return(true);
    }

    public:

    uint32_t init(uint32_t frequency) {
      // set frequency (SCL freq = F_CPU / (16 + 2*TWBR*prescaler) , where prescaler is 1,8,16 or 64x, see page 200)
      TWSR &= 0b11111000; //set prescaler to 1x
      //TWBR  = 12; //set clock reducer to 400kHz (i recommend external pullups at this point)
      #define prescaler 1
      TWBR = ((F_CPU / frequency) - 16) / (2*prescaler);
      uint32_t reconstFreq = F_CPU / (16 + (2*TWBR*prescaler));
      //Serial.print("freq: "); Serial.print(frequency); Serial.print(" TWBR:"); Serial.print(TWBR); Serial.print(" freq: "); Serial.println(reconstFreq);
      // the fastest i could get I2C to work is 800kHz (with another arduino as slave at least), which is TWBR=2 (with some 1K pullups)
      // any faster and i get SLA_ACK errors.
      return(reconstFreq);
    }

    
    uint8_t requestReadByte(uint8_t registerToRead) { //can also be abstracted with requestReadBytes(), but whatever
      if(!startWrite()) { return; }
      twiWrite(registerToRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      if(!startRead()) { return; }
      TWCR = twi_basic; //request 1 byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(0); }
      uint8_t returnData = TWDR;
      TWCR = twi_STOP;
      return(returnData);
    }
    
//    uint16_t requestReadInt(uint8_t registerToStartRead) { //note: by int i mean 12bit unsigned
//      if(!startWrite()) { return; }
//      twiWrite(registerToStartRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
//      if(!startRead()) { return; }
//      TWCR = twi_basic_ACK; //request several bytes
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(0); }
//      uint16_t returnData = (TWDR & LSB_NIBBLE) << 8;
//      TWCR = twi_basic; //request 1 more byte
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(0); }
//      returnData |= TWDR;
//      TWCR = twi_STOP;
//      return(returnData);
//    }
    
    bool requestReadBytes(uint8_t registerToStartRead, uint8_t readBuff[], uint16_t bytesToRead) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      if(!startRead()) { return(false); }
      for(uint16_t i=0; i<(bytesToRead-1); i++) {
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(false); }
        readBuff[i] = TWDR;
      }
      TWCR = twi_basic; //request 1 more byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(false); }
      readBuff[bytesToRead-1] = TWDR;
      TWCR = twi_STOP;
      return(true);
    }
    
    bool onlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
      if(!startRead()) { return(false); }
      for(uint16_t i=0; i<(bytesToRead-1); i++) {
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(false); }
        readBuff[i] = TWDR;
      }
      TWCR = twi_basic; //request 1 more byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(false); }
      readBuff[bytesToRead-1] = TWDR;
      TWCR = twi_STOP;
      return(true);
    }
    
    
    bool writeBytes(uint8_t registerToStartWrite, uint8_t dataToWrite[], uint8_t dataByteCount) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartWrite);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      for(uint8_t i=0; i<dataByteCount; i++) {
        twiWrite(dataToWrite[i]);
        //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      }
      TWCR = twi_STOP;
      return(true);
    }
    
    bool writeSameValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t dataByteCount) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartWrite);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      for(uint8_t i=0; i<dataByteCount; i++) {
        twiWrite(valueToWrite);
        //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      }
      TWCR = twi_STOP;
      return(true);
    }

    bool loopReadWithCallback(/*uint8_t registerToStartRead,*/ void (*callbackFunc)(uint16_t angle), int16_t readsBeforeRestart) {
      keepLooping = true;
      const uint8_t registerToStartRead = AS5600_RAW_ANGLE_H;
      uint8_t readBuff[2];
      uint16_t newAngle = 0;
      bool everyTwoBytes = true;
      
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      if(!startRead()) { return(false); }
      //for(uint16_t readItt=0; ((readItt<((readsBeforeRestart*2)-1)) || (readsBeforeRestart<0)) && keepLooping; readItt++) { //this works as an infinite loop if readsBeforeRestart < 0
      for(uint16_t readItt=0; ((readsBeforeRestart>0) ? (readItt<((readsBeforeRestart*2)-1)) : true) && keepLooping; readItt++) { //this works as an infinite loop if readsBeforeRestart < 0
        everyTwoBytes = !everyTwoBytes; //flip-flop boolean to ensure 2 bytes are 
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(false); }
        readBuff[everyTwoBytes] = TWDR;
        if(everyTwoBytes) {
          newAngle = (readBuff[0] & LSB_NIBBLE) << 8;
          newAngle |= readBuff[1];
          callbackFunc(newAngle);
        }
      }
      //according to the datasheet, any read interaction MUST end with a Nack read
      TWCR = twi_basic; //request 1 last byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(false); }
      readBuff[1] = TWDR;
      TWCR = twi_STOP;
      if(keepLooping) { //if the forloop was stopped manually, then you shouldn't run the callback another time
        newAngle = (readBuff[0] & LSB_NIBBLE) << 8;
        newAngle |= readBuff[1];
        callbackFunc(newAngle);
      }
      //keepLooping = false; //redundant?
      return(true);
    }
    
  #endif //328p arduino I2C

  //// the following functions are abstract enough that they'll work for either architecture
  
  uint16_t requestReadInt(uint8_t registerToStartRead) { //note: by int i mean 12bit unsigned
    uint8_t readBuff[2]; //im secretly hoping the compiler will understand what i'm trying to do here and make it as efficient as possible, but probably...
    requestReadBytes(registerToStartRead, readBuff, 2);
    uint16_t returnData = (readBuff[0] & LSB_NIBBLE)  << 8;
    returnData |= readBuff[1];
    return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
  }

  uint16_t onlyReadInt() { //note: by int i mean 12bit unsigned
    uint8_t readBuff[2];
    onlyReadBytes(readBuff, 2);
    uint16_t returnData = (readBuff[0] & LSB_NIBBLE) << 8;
    returnData |= readBuff[1];
    return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
  }

  uint16_t getAngle() { return(requestReadInt(AS5600_RAW_ANGLE_H)); } //basically macros
  uint16_t getAltAngle() { return(requestReadInt(AS5600_ANGLE_H)); } //this angle has hysteresis (i think)

  AS5600_ERR_RETURN_TYPE writeInt(uint8_t registerToStartWrite, uint16_t intToWrite) {
    uint8_t writeBuff[sizeof(intToWrite)];
    //writeBuff[0] = (intToWrite >> 8) & LSB_NIBBLE;
    writeBuff[0] = intToWrite >> 8;
    writeBuff[1] = intToWrite & 0xFF;
    return(writeBytes(registerToStartWrite, writeBuff, sizeof(intToWrite)));
  }

  AS5600_ERR_RETURN_TYPE setZPOS(uint16_t newValue) { return(writeInt(AS5600_ZPOS_H, newValue)); }
  AS5600_ERR_RETURN_TYPE setMPOS(uint16_t newValue) { return(writeInt(AS5600_MPOS_H, newValue)); }
  AS5600_ERR_RETURN_TYPE setMANG(uint16_t newValue) { return(writeInt(AS5600_MANG_H, newValue)); }

  AS5600_ERR_RETURN_TYPE setWD(bool newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_A);
    currentReg &= ~AS5600_WD_bits; //erase old value
    currentReg |= (newValue * AS5600_WD_bits); //insert new value
    return(writeBytes(AS5600_CONF_A, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setFTH(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_A);
    currentReg &= ~AS5600_FTH_bits; //erase old value
    currentReg |= ((newValue<<2) & AS5600_FTH_bits); //insert new value
    return(writeBytes(AS5600_CONF_A, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setSF(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_A);
    currentReg &= ~AS5600_SF_bits; //erase old value
    currentReg |= (newValue & AS5600_SF_bits); //insert new value
    return(writeBytes(AS5600_CONF_A, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setPWMF(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_PWMF_bits; //erase old value
    currentReg |= ((newValue<<6) & AS5600_PWMF_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setOUTS(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_OUTS_bits; //erase old value
    currentReg |= ((newValue<<4) & AS5600_OUTS_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setHYST(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_HYST_bits; //erase old value
    currentReg |= ((newValue<<2) & AS5600_HYST_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setPM(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_PM_bits; //erase old value
    currentReg |= (newValue & AS5600_PM_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  #ifdef AS5600_unlock_burning
    AS5600_ERR_RETURN_TYPE burnAngle() { return(writeSameValue(AS5600_BURN, AS5600_BURN_ANGLE_bits, 1)); }
    AS5600_ERR_RETURN_TYPE burnSetting() { return(writeSameValue(AS5600_BURN, AS5600_BURN_SETTING_bits, 1)); }
  #endif

  uint8_t getZMCO() { return(requestReadByte(AS5600_ZMCO) & AS5600_ZMCO_bits); } //burn count
  uint16_t getZPOS() { return(requestReadInt(AS5600_ZPOS_H)); }            //start pos
  uint16_t getMPOS() { return(requestReadInt(AS5600_MPOS_H)); }            //stop pos
  uint16_t getMANG() { return(requestReadInt(AS5600_MANG_H)); }            //max angle
  bool getWD() { return(requestReadByte(AS5600_CONF_A) & AS5600_WD_bits); }   //watchdog config
  uint8_t getFTH() { return((requestReadByte(AS5600_CONF_A) & AS5600_FTH_bits) >> 2); }  //fast filter threshold
  uint8_t getSF() { return(requestReadByte(AS5600_CONF_A) & AS5600_SF_bits); }          //slow filter
  uint8_t getPWMF() { return((requestReadByte(AS5600_CONF_B) & AS5600_PWMF_bits) >> 6); } //PWM frequency
  uint8_t getOUTS() { return((requestReadByte(AS5600_CONF_B) & AS5600_OUTS_bits) >> 4); } //output stage
  uint8_t getHYST() { return((requestReadByte(AS5600_CONF_B) & AS5600_HYST_bits) >> 2); } //hysteresis
  uint8_t getPM() { return(requestReadByte(AS5600_CONF_B) & AS5600_PM_bits); }          //power mode

  uint8_t getSTATUS() { return(requestReadByte(AS5600_STATUS) & 0b00111000); } //status: magnet detected, magnet too weak, magnet too strong
  uint8_t getAGC() { return(requestReadByte(AS5600_AGC)); } //should return 128 if magnet is positioned optimally (affected by temp, airgap and magnet degredation)
  uint16_t getMAGNITUDE() { return(requestReadInt(AS5600_MAGNITUDE_H)); } //magnitude of the 

  bool magnetDetected()  { return(getSTATUS() & AS5600_MD_bits); }
  bool magnetTooWeak()   { return(getSTATUS() & AS5600_ML_bits); }
  bool magnetTooStrong() { return(getSTATUS() & AS5600_MH_bits); }

  void printConfig() {
    uint8_t readBuff[9];
    if(requestReadBytes(AS5600_ZMCO, readBuff, 9)) {
      //Serial.println("printing config:");
      Serial.print("burn count (ZMCO):"); Serial.println(readBuff[0] & AS5600_ZMCO_bits);
      uint16_t bigVal = ((readBuff[1] & LSB_NIBBLE) << 8) | readBuff[2];
      Serial.print("start pos (ZPOS):"); Serial.println(bigVal);
      bigVal = ((readBuff[3] & LSB_NIBBLE) << 8) | readBuff[4];
      Serial.print("stop pos (MPOS):"); Serial.println(bigVal);
      bigVal = ((readBuff[5] & LSB_NIBBLE) << 8) | readBuff[6];
      Serial.print("max angle (MANG):"); Serial.println(bigVal);
      Serial.print("watchdog (WD):"); Serial.println((readBuff[7] & AS5600_WD_bits)>>5);
      Serial.print("fast filter threshold (FTH):"); Serial.println((readBuff[7] & AS5600_FTH_bits)>>2);
      Serial.print("slow filter (SF):"); Serial.println(readBuff[7] & AS5600_SF_bits);
      Serial.print("PWM frequency (PWMF):"); Serial.println((readBuff[8] & AS5600_PWMF_bits)>>6);
      Serial.print("output stage (OUTS):"); Serial.println((readBuff[8] & AS5600_OUTS_bits)>>4);
      Serial.print("hysteresis (HYST):"); Serial.println((readBuff[8] & AS5600_HYST_bits)>>2);
      Serial.print("power mode (PM):"); Serial.println(readBuff[8] & AS5600_PM_bits);
    } else {
      Serial.println("failed to read config!");
    }
  }

  void printStatus() {
    Serial.print("magnetDetected:"); Serial.println(magnetDetected());
    Serial.print("magnetTooWeak:"); Serial.println(magnetTooWeak());
    Serial.print("magnetTooStrong:"); Serial.println(magnetTooStrong());
    Serial.print("Automatic Gain Control (ideal: 128@5v, 64@3.3v):"); Serial.println(getAGC());
    Serial.print("magnitude (CORDIC):"); Serial.println(getMAGNITUDE());
  }

  AS5600_ERR_RETURN_TYPE resetConfig() { return(writeSameValue(AS5600_ZPOS_H, 0, 8)); } //write zeroes to all configuration registers (reset to default)
};

#endif  // AS5600

/*
AS5600 I2C magentic encoder

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


//note when writing lib: right now, i'm reading the angle as Hreg, Lreg, 
// but technically i could request the Lreg first, and the pointer return should make it so the value after that is the Hreg
// that my be able to save a single clock cylce when shifting the byte to make an int, maybe not.
