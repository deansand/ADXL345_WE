#ifndef ADXL345_WE_COMBINED_H_
#define ADXL345_WE_COMBINED_H_

#include <Arduino.h>
#include "ADXL345_WE.h"
#include <Wire.h>

#define USE_I2C
// #define USE_SPI

#ifdef USE_I2C
#define ADXL345_I2CADDR_DEFAULT 0x53
#define ADXL345_I2CADDR_HIGH 0x1D // if SDO = HIGH
#endif

#ifdef USE_SPI
#include <SPI.h>
#endif

#ifdef
struct xyzFloat
{
    float x;
    float y;
    float z;
} axis_t;

// typedef struct xyzFloat xyzFloat_t;
#endif


// todo test
typedef struct
{
    volatile float *x;
    volatile float *y;
    volatile float *z;
} axis_t;


// todo test
typedef struct
{
    xyzValues *values;
    volatile float *g;
} xyzVectorG;


// #define ADXL345_OK    1 // no error
// #define ADXL345_ERROR 0 // indicates error is predent

#ifdef ADXL345_WE_COMBINED
#define ADXL345_WE_COMBINED 
class ADXL345_WE_COMBINED : public ADXL345_WE {
public:
    using ADXL345_WE::ADXL345_WE;

    bool selfTest() {
        
        //set selfTest bit = 1
        setSelfTestBit(true);

        //get value with selfTest
        xyzValues selfTestData = readAccel();

        //set selfTest bit = 0
        setSelfTestBit(false);

        //get value without selfTest
        xyzValues normalData = readAccel();

        //calculate difference betwen data
        float diffX = std::abs(*selfTestData.x - *normalData.x);
        float diffY = std::abs(*selfTestData.y - *normalData.y);
        float diffZ = std::abs(*selfTestData.z - *normalData.z);
    
        //checking if data is in our limits
        return(diffX >= EXPECTED_MIN_X && diffX <= EXPECTED_MAX_X
            && diffY >= EXPECTED_MIN_Y && diffY <= EXPECTED_MAX_Y
            && diffZ >= EXPECTED_MIN_Z && diffZ <= EXPECTED_MAX_Z)

    ADXL345_WE() : useSPI{false} {}
    #ifdef USE_I2C
        ADXL345_WE(uint8_t addr) : _wire{&Wire}, i2cAddress{addr}, useSPI{false} {}
    
        ADXL345_WE(TwoWire *w, uint8_t addr = ADXL345_I2CADDR_DEFAULT) : _wire{w}, i2cAddress{addr}, useSPI{false} {}
    #endif
    #ifdef USE_SPI
        ADXL345_WE(int cs, bool spi, int mosi = 999, int miso = 999, int sck = 999)
            : _spi{&SPI}, csPin{cs}, useSPI{spi}, mosiPin{mosi}, misoPin{miso}, sckPin{sck} {}
    
        ADXL345_WE(SPIClass *s, int cs, bool spi, int mosi = 999, int miso = 999, int sck = 999)
            : _spi{s}, csPin{cs}, useSPI{spi}, mosiPin{mosi}, misoPin{miso}, sckPin{sck} {}
    #endif

    #ifdef USE_I2C
    void setWire(TwoWire *w);
    void setAddr(uint8_t addr);
    #endif

    #ifdef USE_SPI
    void setSPIClockSpeed(unsigned long clock);
    #endif
    
protected:
    #ifdef USE_I2C
        TwoWire *_wire = nullptr;
        uint8_t i2cAddress = ADXL345_I2CADDR_DEFAULT;
    #endif
    
    #ifdef USE_SPI
        SPIClass *_spi = nullptr;
        SPISettings mySPISettings;
    #endif
        uint8_t regVal; // intermediate storage of register values
        xyzFloat offsetVal;
        xyzFloat angleOffsetVal;
        xyzFloat corrFact;

        bool useSPI;
    
        #ifdef USE_SPI
        uint8_t csPin;
        uint8_t mosiPin;
        uint8_t misoPin;
        uint8_t sckPin;
    #endif
        int16_t sensorID;
        float rangeFactor;
        bool adxl345_lowRes;
        uint8_t writeToRegister(uint8_t reg, uint8_t val);
        uint8_t readRegisterSingle(uint8_t reg_addr);
        void readFromRegisterMulti(uint8_t reg_addr, uint8_t count, uint8_t *buf);

        bool getRegisterBit(byte regAdress, int bitPos);
        void setRegisterBit(byte regAdress, int bitPos, bool state);

        void setAngleOffsets(xyzFloat aos);
    
    uint8_t regVal; // intermediate storage of register values
    xyzFloat offsetVal;
    xyzFloat angleOffsetVal;
    xyzFloat corrFact;
    
    /* Angles and Orientation */
    void measureAngleOffsets(xyzFloat *aos = nullptr);


    /** The parameters of the setFreeFallThresholds function are:
         - g threshold - do not choose a parameter which is too low. 0.3 - 0.6 g is fine.
         - time threshold in ms, maximum is 1275. Recommended is 100 - 350;
        If time threshold is too low, vibrations can be detected as free fall.
    */
    void setFreeFallThresholds(float threshold, float fftime);
    uint8_t getActTapStatusAsValue();

    /** FIFO */

    /** The following two FIFO parameters need to be set:
     *    1. Trigger Bit: the trigger is an interrupt at INT1 or INT2
     *       ADXL345_TRIGGER_INT_1 - Trigger is an interrupt at INT1
     *       ADXL345_TRIGGER_INT_2 - Trigger is an interrupt at INT2
     *      @note not relevant for:
     *             ADXL345_STREAM
     *             ADXL345_FIFO
     *             ADXL345_TRIGGER
     *    2. FIFO samples (max 32). Defines the size of the FIFO. @note One sample is an x,y,z triple.
     */
    void setFifoParameters(adxl345_triggerInt intNumber, uint8_t samples = MAX_ADXL_BUFF_SIZE);
    
    //  uint8_t getFifoSize(void);
    byte getFifoSize(void);
    void burstReadXYZ(float *x, float *y, float *z, byte samples); // burst read function for getting all samples from fifo

    bool getSelfTestBit();
    void setSelfTestBit(bool selfTestBit);

    bool useSPI;
    #ifdef USE_SPI
    uint8_t csPin;
    uint8_t mosiPin;
    uint8_t misoPin;
    uint8_t sckPin;
    #endif

private:
    void printAllRegister();
    // DATA* registers
    // void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
    // int16_t getAccelerationX();
    // int16_t getAccelerationY();
    // int16_t getAccelerationZ();
    static constexpr float EXPECTED_MIN_X = 0.1;
    static constexpr float EXPECTED_MAX_X = 0.3;
    static constexpr float EXPECTED_MIN_Y = 0.1;
    static constexpr float EXPECTED_MAX_Y = 0.3;
    static constexpr float EXPECTED_MIN_Z = 0.1;
    static constexpr float EXPECTED_MAX_Z = 0.3;
}


#if 0
 
 /** Get 3-axis accleration measurements.
  * These six bytes (Register 0x32 to Register 0x37) are eight bits each and hold
  * the output data for each axis. Register 0x32 and Register 0x33 hold the
  * output data for the x-axis, Register 0x34 and Register 0x35 hold the output
  * data for the y-axis, and Register 0x36 and Register 0x37 hold the output data
  * for the z-axis. The output data is twos complement, with DATAx0 as the least
  * significant byte and DATAx1 as the most significant byte, where x represent
  * X, Y, or Z. The DATA_FORMAT register (Address 0x31) controls the format of
  * the data. It is recommended that a multiple-byte read of all registers be
  * performed to prevent a change in data between reads of sequential registers.
  * 
  * The DATA_FORMAT register controls the presentation of data to Register 0x32
  * through Register 0x37. All data, except that for the +/-16 g range, must be
  * clipped to avoid rollover.
  *
  * @param x 16-bit signed integer container for X-axis acceleration
  * @param y 16-bit signed integer container for Y-axis acceleration
  * @param z 16-bit signed integer container for Z-axis acceleration
  * @see ADXL345_RA_DATAX0
  */
 void ADXL345::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
     I2Cdev::readBytes(devAddr, ADXL345_RA_DATAX0, 6, buffer);
     *x = (((int16_t)buffer[1]) << 8) | buffer[0];
     *y = (((int16_t)buffer[3]) << 8) | buffer[2];
     *z = (((int16_t)buffer[5]) << 8) | buffer[4];
 }
 /** Get X-axis accleration measurement.
  * @return 16-bit signed X-axis acceleration value
  * @see ADXL345_RA_DATAX0
  */
 int16_t ADXL345::getAccelerationX() {
     I2Cdev::readBytes(devAddr, ADXL345_RA_DATAX0, 2, buffer);
     return (((int16_t)buffer[1]) << 8) | buffer[0];
 }
 /** Get Y-axis accleration measurement.
  * @return 16-bit signed Y-axis acceleration value
  * @see ADXL345_RA_DATAY0
  */
 int16_t ADXL345::getAccelerationY() {
     I2Cdev::readBytes(devAddr, ADXL345_RA_DATAY0, 2, buffer);
     return (((int16_t)buffer[1]) << 8) | buffer[0];
 }
 /** Get Z-axis accleration measurement.
  * @return 16-bit signed Z-axis acceleration value
  * @see ADXL345_RA_DATAZ0
  */
 int16_t ADXL345::getAccelerationZ() {
     I2Cdev::readBytes(devAddr, ADXL345_RA_DATAZ0, 2, buffer);
     return (((int16_t)buffer[1]) << 8) | buffer[0];
 }
 
 
 
 
 /** Read multiple bytes from an 8-bit device register.
  * @param devAddr I2C slave device address
  * @param regAddr First register regAddr to read from
  * @param length Number of bytes to read
  * @param data Buffer to store read data in
  * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
  * @return Number of bytes read (-1 indicates failure)
  */
 int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) {
#ifdef I2CDEV_SERIAL_DEBUG
         Serial.print("I2C (0x");
         Serial.print(devAddr, HEX);
         Serial.print(") reading ");
         Serial.print(length, DEC);
         Serial.print(" bytes from 0x");
         Serial.print(regAddr, HEX);
         Serial.print("...");
#endif
 
     int8_t count = 0;
     uint32_t t1 = millis();

#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
         TwoWire *useWire = &Wire;
         if (wireObj) useWire = (TwoWire *)wireObj;

#if (ARDUINO < 100)
             // Arduino v00xx (before v1.0), Wire library
 
             // I2C/TWI subsystem uses internal buffer that breaks with large data requests
             // so if user requests more than I2CDEVLIB_WIRE_BUFFER_LENGTH bytes, we have to do it in
             // smaller chunks instead of all at once
             for (int k = 0; k < length; k += min((int)length, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                 useWire->beginTransmission(devAddr);
                 useWire->send(regAddr);
                 useWire->endTransmission();
                 useWire->beginTransmission(devAddr);
                 useWire->requestFrom((uint8_t)devAddr, (uint8_t)min((int)length - k, I2CDEVLIB_WIRE_BUFFER_LENGTH));
 
                 for (; useWire->available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                     data[count] = useWire->receive();
#ifdef I2CDEV_SERIAL_DEBUG
                         Serial.print(data[count], HEX);
                         if (count + 1 < length) Serial.print(" ");
#endif
                 }
 
                 useWire->endTransmission();
             }
#elif (ARDUINO == 100)
             // Arduino v1.0.0, Wire library
             // Adds standardized write() and read() stream methods instead of send() and receive()
 
             // I2C/TWI subsystem uses internal buffer that breaks with large data requests
             // so if user requests more than I2CDEVLIB_WIRE_BUFFER_LENGTH bytes, we have to do it in
             // smaller chunks instead of all at once
             for (int k = 0; k < length; k += min((int)length, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                 useWire->beginTransmission(devAddr);
                 useWire->write(regAddr);
                 useWire->endTransmission();
                 useWire->beginTransmission(devAddr);
                 useWire->requestFrom((uint8_t)devAddr, (uint8_t)min((int)length - k, I2CDEVLIB_WIRE_BUFFER_LENGTH));
         
                 for (; useWire->available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                     data[count] = useWire->read();
#ifdef I2CDEV_SERIAL_DEBUG
                         Serial.print(data[count], HEX);
                         if (count + 1 < length) Serial.print(" ");
#endif
                 }
         
                 useWire->endTransmission();
             }
#elif (ARDUINO > 100)
             // Arduino v1.0.1+, Wire library
             // Adds official support for repeated start condition, yay!
 
             // I2C/TWI subsystem uses internal buffer that breaks with large data requests
             // so if user requests more than I2CDEVLIB_WIRE_BUFFER_LENGTH bytes, we have to do it in
             // smaller chunks instead of all at once
             for (int k = 0; k < length; k += min((int)length, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                 useWire->beginTransmission(devAddr);
                 useWire->write(regAddr);
                 useWire->endTransmission();
                 useWire->beginTransmission(devAddr);
                 useWire->requestFrom((uint8_t)devAddr, (uint8_t)min((int)length - k, I2CDEVLIB_WIRE_BUFFER_LENGTH));
         
                 for (; useWire->available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                     data[count] = useWire->read();
#ifdef I2CDEV_SERIAL_DEBUG
                         Serial.print(data[count], HEX);
                         if (count + 1 < length) Serial.print(" ");
#endif
                 }
             }
#endif

#elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
 
         // Fastwire library
         // no loop required for fastwire
         uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, data, length);
         if (status == 0) {
             count = length; // success
         } else {
             count = -1; // error
         }

#endif
 
     // check for timeout
     if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

#ifdef I2CDEV_SERIAL_DEBUG
         Serial.print(". Done (");
         Serial.print(count, DEC);
         Serial.println(" read).");
#endif
 
     return count;
 }


#ifdef ADXL345_DEBUG
#define ADXL_PRINT(x) Serial.print(x)
#define ADXL_PRINTLN(x) Serial.println(x)
#define ADXL_PRINT_BIN(x) Serial.print(x, BIN)
#define ADXL_PRINT_HEX(x) Serial.print(x, HEX)
#else
#define ADXL_PRINT(x)
#define ADXL_PRINTLN(x)
#define ADXL_PRINT_BIN(x)
#define ADXL_PRINT_HEX(x)
#endif

#define ADXL345_TO_READ (6) // num of bytes we are going to read each time (two bytes for each axis)


/* .cpp file */
/***** Basic sattings *****/
void print_byte(byte val)
{
    int i;
    ADXL_PRINT("B");
    for (i = 7; i >= 0; i--)
    {
        ADXL_PRINT_BIN(val >> i & 1);
    }
}

#ifdef USE_SPI
    if (useSPI)
    {
        if (mosiPin == 999)
        {
            _spi->begin();
        }
#ifdef ESP32
        else
        {
            _spi->begin(sckPin, misoPin, mosiPin, csPin);
        }
#endif
        mySPISettings = SPISettings(5000000, MSBFIRST, SPI_MODE3);
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
    }
#endif
#if (defined USE_SPI) && (defined USE_I2C)
    else
    {
        if (_wire == nullptr)
        {
            _wire = &Wire;
        }
#ifndef I2C_NO_SPEED_CONFIG
        _wire->setClock(400000L);
#endif
        _wire->begin();
    }
#elif defined USE_I2C
    if (!useSPI)
    {
        if (_wire == nullptr)
        {
            _wire = &Wire;
        }
#ifndef I2C_NO_SPEED_CONFIG
        _wire->setClock(400000L);
#endif
        _wire->begin();
    }
#endif

/**
 * @brief Power ON
 */
// void FaBo3Axis::powerOn()
// {
//   uint8_t power = ADXL345_AUTO_SLEEP_OFF;
//   power |= ADXL345_MEASURE_ON;
//   power |= ADXL345_SLEEP_OFF;
//   power |= ADXL345_WAKEUP_8HZ;
//   writeI2c(ADXL345_POWER_CTL_REG, power);
// }

/**************************************************************************/
/*!
    @brief  Reads the device ID (can be used to check connection)
    @return The Device ID of the connected sensor
*/
/**************************************************************************/
uint8_t ADXL345_WE::getDeviceID(void)
{
    // Check device ID register
    return readRegisterSingle(ADXL345_DEVID);
}

bool ADXL345_WE::checkConnection()
{
    uint8_t deviceid = getDeviceID();
    if (deviceid != ADXL345_DEVICE)
    {
        /* No ADXL345 detected ... return false */
        return false;
    }
    return true;
}

#ifdef USE_I2C
void ADXL345_WE::setWire(TwoWire *w)
{
    if (w == nullptr)
    {
        return;
    }
    this->_wire = w;
}

void ADXL345_WE::setAddr(uint8_t addr)
{
    this->i2cAddress = addr;
}
#endif

#ifdef USE_SPI
void ADXL345_WE::setSPIClockSpeed(unsigned long clock)
{
    mySPISettings = SPISettings(clock, MSBFIRST, SPI_MODE3);
}
#endif

// gets the state of the SELF_TEST bit
bool ADXL345_WE::getSelfTestBit()
{
    return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to 0 it disables the self-test force
void ADXL345_WE::setSelfTestBit(bool selfTestBit)
{
    setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

void ADXL345_WE::setGeneralTapParameters(adxl345_actTapSet axes, float threshold, float duration, float latent)
{
    this->regVal = readRegisterSingle(ADXL345_TAP_AXES);
    this->regVal &= 0b11111000;
    this->regVal |= static_cast<uint8_t>(axes);
    writeToRegister(ADXL345_TAP_AXES, this->regVal);

    float regValRaw = round(threshold / 0.0625);
    this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
    if (this->regVal < 1)
    {
        this->regVal = 1;
    }
    writeToRegister(ADXL345_THRESH_TAP, this->regVal);

    if (duration < 10)
    {
        duration = 10;
    }
    if (duration > 159) // todo make a define or variable
    {
        duration = 159;
    }

    regValRaw = round(duration / 0.625);
    this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
    if (this->regVal < 1)
    {
        this->regVal = 1;
    }
    writeToRegister(ADXL345_DUR, this->regVal);

    if (latent < 20)
    {
        latent = 20;
    }

    if (latent > 318)
    {
        latent = 318;
    }

    regValRaw = round(latent / 1.25);
    this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
    if (this->regVal < 1)
    {
        this->regVal = 1;
    }
    writeToRegister(ADXL345_LATENT, this->regVal);
}

uint8_t ADXL345_WE::getActTapStatusAsValue()
{
    uint8_t mask = (readRegisterSingle(ADXL345_ACT_INACT_CTL)) & 0b01110000;
    mask |= ((readRegisterSingle(ADXL345_TAP_AXES)) & 0b00000111);

    this->regVal = getActTapStatus();
    this->regVal &= mask;
    return this->regVal;
}

/************ FIFO ************/

void ADXL345_WE::setFifoParameters(adxl345_triggerInt intNumber, uint8_t samples)
{
    if (samples > MAX_ADXL_BUFF_SIZE)
    {
        samples = MAX_ADXL_BUFF_SIZE;
    }

    this->regVal = readRegisterSingle(ADXL345_FIFO_CTL);
    this->regVal &= 0b11000000;
    this->regVal |= (samples - 1);
    if (intNumber == ADXL345_TRIGGER_INT_2)
    {
        this->regVal |= 0x20;
    }
    writeToRegister(ADXL345_FIFO_CTL, this->regVal);
}

void ADXL345_WE::setFifoMode(adxl345_fifoMode mode)
{
    this->regVal = readRegisterSingle(ADXL345_FIFO_CTL);
    this->regVal &= 0b00111111;
    this->regVal |= (mode << 6);
    writeToRegister(ADXL345_FIFO_CTL, this->regVal);
}

uint8_t ADXL345_WE::getFifoStatus()
{
    return readRegisterSingle(ADXL345_FIFO_STATUS);
}

void ADXL345_WE::resetTrigger()
{
    setFifoMode(ADXL345_BYPASS);
    setFifoMode(ADXL345_TRIGGER);
}

// getFifoEntries OR getFifoSize
byte ADXL345_WE::getFifoSize(void)
{
    byte _b;
    readFromRegisterMulti(ADXL345_FIFO_STATUS, 1, &_b);
    _b &= 0b00111111; // MASK FOR CURRENT SIZE OF FIFO BUFFER
    return _b;
}

void ADXL345_WE::burstReadXYZ(float *x, float *y, float *z, byte samples)
{
    for (int i = 0; i < samples; i++)
    {
        xyzFloat rawData = getRawValues();
        x[i] = rawData.x;
        y[i] = rawData.y;
        z[i] = rawData.z;
        ADXL_PRINT("Sample ");
        ADXL_PRINT(i);
        ADXL_PRINTLN(": X=");
        ADXL_PRINT(x[i]);
        ADXL_PRINT(" Y=");
        ADXL_PRINT(y[i]);
        ADXL_PRINT(" Z=");
        ADXL_PRINT(z[i]);
    }
}

/************************************************
    private functions
*************************************************/

// writeTo
uint8_t ADXL345_WE::writeToRegister(uint8_t reg_addr, uint8_t val)
{
    if (!useSPI)
    {
#ifdef USE_I2C
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg_addr);
        _wire->write(val);
        return _wire->endTransmission();
#endif
    }
    else
    {
#ifdef USE_SPI
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg_addr);
        _spi->transfer(val);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
#endif
        return false; // to be amended
    }
}

uint8_t ADXL345_WE::readRegisterSingle(uint8_t reg_addr)
{
    uint8_t regValue = 0;
    if (!useSPI)
    {
#ifdef USE_I2C
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg_addr);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress, static_cast<uint8_t>(1));
        if (_wire->available())
        {
            regValue = _wire->read();
        }
        // Wire.endTransmission();
#endif
    }
    else
    {
#ifdef USE_SPI
        reg_addr |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg_addr);
        regValue = _spi->transfer(0x00);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
#endif
    }
    return regValue;
}

// readFrom
void ADXL345_WE::readFromRegisterMulti(uint8_t reg_addr, uint8_t count, uint8_t *buf)
{
    if (!useSPI)
    {
#ifdef USE_I2C
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg_addr);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress, count);
        for (int i = 0; i < count; i++)
        {
            buf[i] = _wire->read();
        }
        // Wire.endTransmission();
        /* todo OR
        Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device
        Wire.requestFrom(ADXL345_DEVICE, num);    // request 6 bytes from device
        int i = 0;
        while (Wire.available()) {      // device may send less than requested (abnormal)
            _buff[i] = Wire.read();    // receive a byte
            i++;
            if (i > num) {
                break;
            }
        }
        if (i != num) {
            status = ADXL345_ERROR;         // #define ADXL345_OK    0 // no error
            error_code = ADXL345_READ_ERROR; // #define ADXL345_ERROR 1 // indicates error is predent
        }
        Wire.endTransmission();

        ? #define ADXL345_NO_ERROR   0 // initial state
        ? #define ADXL345_READ_ERROR 1 // problem reading accel
        ? #define ADXL345_BAD_ARG    2 // bad method argument

        */
#endif
    }
    else
    {
#ifdef USE_SPI
        reg_addr = reg_addr | 0x80;
        reg_addr = reg_addr | 0x40;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg_addr);
        for (int i = 0; i < count; i++)
        {
            buf[i] = _spi->transfer(0x00);
        }
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
#endif
    }
}

// print all register value to the serial ouptut, which requires it to be setup
// this can be used to manually to check the current configuration of the device

void ADXL345_WE::printAllRegister()
{
    byte _b;
    Serial.print("0x00: ");
    readFromRegisterMulti(ADXL345_DEVID, 1, &_b);
    //	print_byte(_b);
    //	Serial.println("");
    int i;
    for (i = 29; i <= 57; i++)
    {
        // todo add adxl debug macro
        //  Serial.print("0x");
        //  Serial.print(i, HEX);
        //  Serial.print(": ");
        readFromRegisterMulti(i, 1, &_b);
        //	print_byte(_b);
        //	Serial.println("");
    }
}

bool ADXL345_WE::getRegisterBit(byte regAdress, int bitPos)
{
    byte _b;
    readFromRegisterMulti(regAdress, 1, &_b);
    return ((_b >> bitPos) & 1);
}

void ADXL345_WE::setRegisterBit(byte regAdress, int bitPos, bool state)
{
    byte _b;
    readFromRegisterMulti(regAdress, 1, &_b);
    if (state)
    {
        _b |= (1 << bitPos); // forces nth bit of _b to be 1.  all other bits left alone.
    }
    else
    {
        _b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
    }
    writeToRegister(regAdress, _b);
}

/*

// read how many samples in Fifi
https://github.com/Seeed-Studio/Accelerometer_ADXL345/blob/master/ADXL345.cpp#L59

byte ADXL345::getFifoEntries(void) {
    byte _b;
    readFromRegisterMulti(ADXL345_FIFO_STATUS, 1, &_b);
    _b &=  0b00111111;

    return _b;
}
*/

void ADXL345_WE::readAccel(int *xyz)
{
    readXYZ(xyz, xyz + 1, xyz + 2);
}

void ADXL345_WE::readXYZ(int *x, int *y, int *z)
{
    uint8_t rawData[ADXL345_TO_READ];
    readFromRegisterMulti(ADXL345_DATAX0, ADXL345_TO_READ, rawData); // read the acceleration data from the ADXL345
    *x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
    *y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
    *z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);
}

void ADXL345_WE::getAcceleration(double *xyz)
{
    int i;
    int xyz_int[3];
    readAccel(xyz_int);
    double gains[3]; // counts to Gs
    gains[0] = 0.00376390;
    gains[1] = 0.00376009;
    gains[2] = 0.00349265;
    for (i = 0; i < 3; i++)
    {
        xyz[i] = xyz_int[i] * gains[i];
    }
}



#endif