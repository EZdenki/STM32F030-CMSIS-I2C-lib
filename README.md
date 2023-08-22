# STM32F030-CMSIS-I2C-lib
Basic I2C library for the STM32F030 based on CMSIS (no HAL)

# The following routines are supported:
+ **```void I2C_init( I2C_TypeDef *thisI2C, uint32_t I2CSpeed )```**<br>
Initialize the specified I2C interface to operate at the specified speed. Note that
currently *only I2C1* is supported! I2C1 uses the following pins:<br>
  - SCL: GPIO pin A9,  pin 17
  - SDA: GPIO pin A10, pin 18<br>
Possible I2C speeds are from 10 kHz up to 400 kHz. Speeds below 10 kHz will default to
10 kHz and speeds above 400 kHz will default to 400 kHz.
+ **```void I2C_start( I2C_TypeDef *thisI2C )```***<br>
Set the start bit and wait for acknowledge that it was set.
+ **```void I2C_setAddress( I2C_TypeDef *thisI2C, uint8_t address )```**<br>
Write the address to the SADD bits of the CR2 register.
+ **```void I2C_stop( I2C_TypeDef *thisI2C )```**<br>
Set and then clear the stop bit.
+ **```void I2C_setNBytes( I2C_TypeDef *thisI2C, uint8_t nBytes )```**<br>
Set the number of bytes to be written.
+ **```void I2C_write( I2C_TypeDef *thisI2C, uint8_t data )```**<br>
Write a byte of data to the I2C interface.
+ **```void//  I2C_stop( I2C_TypeDef *thisI2C )```**<br>
Send the I2C stop bit and wait 20 us to allow enough time for next I2C command to occur
properly.
+ **```void I2C_setReadMode( I2C_TypeDef *thisI2C )```**<br>
Set the I2C interface into the read mode.
+ **```void I2C_setWriteMode( I2C_TypeDef *thisI2C )```**<br>
Set the I2C interface into the write mode.
+ **```uint8_t I2C_read( I2C_TypeDef *thisI2C )```**<br>
Read a byte from the I2C interface.

## Normal Write Command Flow:
1. ```I2C_setAddress( I2C1, deviceI2CAddress )```    (Set once per device)
2. ```I2C_setNBytes( I2C1, numberOfBytesToTransmit )```
3. ```I2C_start( I2C1 )```
4. ```I2C_write( I2C1, data )```<br>
(repeat write as required)
5. ```I2C_stop( I2C1 )```

## Normal Read Command Flow:
1. ```I2C_setAddress( I2C1, deviceI2CAddress )```  (Set once per device)
2. ```I2C_setNBytes( I2C1, numberOfBytesToReceive )```
3. ```I2C_setReadMode( I2C1 )```
4. ```I2C_start( I2C1 )```
5. ```data[0] = I2C_read( I2C1 )```<br>
(repeat read as required)
6. ```I2C_stop( I2C1 )```
7. ```I2C_setWriteMode( I2C1 )```
