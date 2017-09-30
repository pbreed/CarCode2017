/*NB_REVISION*/

/*NB_COPYRIGHT*/

/* This is the header file for the NetBurner I2C Master-only I2C Driver.
   Master I2C will allow the NetBurner to operate as a master device on
   the I2C bus.

   Note: If you want to use multi-master I2C to allow you to act as a slave or
         a master device then please look at the i2cmulti driver.
-------------------------------------------------------------------------------*/

#ifndef _I2CMASTER_H
#define _I2CMASTER_H // if this include guard is changed, you must update the check in i2cmulti.h

#ifdef _I2CMULTI_H
#error i2cmulti.h included previously. Only one of i2cmulti.h and i2cmaster.h may be included.
#else
#include <basictypes.h>


#define I2C_MAX_BUF_SIZE  (64)  // Size allocated to input and output buffers in slave mode I2C

/* defined I2C Timeout values will be used if user does not include the 'ticks_to_wait'
   parameter when calling I2C functions                                                  */
#define I2C_RX_TX_TIMEOUT (5)   // Ticks allowed before timeout of a single byte transmission
#define I2C_START_TIMEOUT (20)  // Ticks allowed before timeout when attempting start on I2C bus

// These are the values the NetBurner I2C functions will return
#define I2C_OK             ( 0 )  // Last instruction terminated correctly
#define I2C_NEXT_WRITE_OK  ( 1 )  // I2C bus is OK for a write
#define I2C_NEXT_READ_OK   ( 2 )  // I2C bus is OK for a read
#define I2C_MASTER_OK      ( 3 )  // I2C finished transmission but still owns but (need to stop or restart)
#define I2C_TIMEOUT        ( 4 )  // A timeout occured while trying communicate on I2C bus
#define I2C_BUS_NOT_AVAIL  ( 5 )  // A timeout occured while trying gain I2C bus control
#define I2C_NOT_READY      ( 6 )  // A read or write was attempted before I2C ready or during a slave transmission
#define I2C_LOST_ARB       ( 7 )  // Lost arbitration during start
#define I2C_LOST_ARB_ADD   ( 8 )  // Lost arbitration and then winner addressed our slave address
#define I2C_NO_LINK_RX_ACK ( 9 )  // We are in Master TX mode and recieved no ACK from slave device, possibly during start

// The following defines were created to allow the multi and master-only drivers to
// have the same function calls.  i.e. Multi_I2CInit(); is the same as calling I2CInit();
#define I2CInit      Master_I2CInit
#define I2CSendBuf   Master_I2CSendBuf
#define I2CReadBuf   Master_I2CReadBuf
#define I2CRestart   Master_I2CRestart
#define I2CStart     Master_I2CStart
#define I2CStop      Master_I2CStop
#define I2CSend      Master_I2CSend
#define I2CRead      Master_I2CRead
#define I2CResetPeripheral  Master_ResetPeripheral

// Set up different platform sim references to i2c struct
#if defined (MCF5441X)
   #define I2C_SR  sim2.i2c0.i2sr
   #define I2C_CR  sim2.i2c0.i2cr
   #define I2C_DR  sim2.i2c0.i2dr
   #define I2C_FDR sim2.i2c0.i2fdr
   #define I2C_ADR sim2.i2c0.i2adr
#elif defined (MCF5213)
   #define I2C_SR  sim.i2c.i2sr
   #define I2C_CR  sim.i2c.i2cr
   #define I2C_DR  sim.i2c.i2dr
   #define I2C_FDR sim.i2c.i2fdr
   #define I2C_ADR sim.i2c.i2ar
#elif defined (MCF5208)
   #define I2C_SR  sim.i2c.i2sr
   #define I2C_CR  sim.i2c.i2cr
   #define I2C_DR  sim.i2c.i2dr
   #define I2C_FDR sim.i2c.i2fdr
   #define I2C_ADR sim.i2c.i2adr
#else
   #define I2C_SR  sim.i2c.sr
   #define I2C_CR  sim.i2c.cr
   #define I2C_DR  sim.i2c.dr
   #define I2C_FDR sim.i2c.fdr
   #define I2C_ADR sim.i2c.adr
#endif


/*------------------------------------------------------------------------------------------
      Simple NetBurner I2C Functions

The following functions are a used to easily communicate on the I2C bus without
the need of configuring timeouts or terminations.  These functions will send or recieve
buffers instead of uint8_t like the advaced functions.
-------------------------------------------------------------------------------------------
===========================================================================================*/

/*----------------------------------------------------------------------------
void I2CInit( uint8_t freqdiv = 0x16 );  MCF5270, MCF5234 and MCF5208
void I2CInit( uint8_t freqdiv = 0x15 );  MCF5213, MCF5282 and MCF52234

I2C initialization routine

FOR ALL MCF5270, MCF5234 and MCF5208 products:
System Bus Clock which = 150MHZ/2 divided by 'freqdiv' will give you the max baud rate
of the master mode I2C bus. Values of freqdiv are found in a the I2FDR table found in
the I2C section of the MCF5270 user manual provided by freescale.  0x16=768 which
gives a baud rate close to 100Kbits/s is the value set if parameter excluded

FOR ALL MCF5213, MCF5282 and MCF52234 products:
System Bus Clock which = 66MHZ divided by 'freqdiv' will give you the max baud rate
of the master mode I2C bus. Values of freqdiv are found in a the I2FDR table found in
the I2C section of the MCF5282 user manual provided by freescale.  0x15=640 which
gives a baud rate close to 100Kbits/s is the value set if parameter excluded

FOR ALL MCF5441x products:
System Bus Clock which = 250MHZ/2 divided by 'freqdiv' will give you the max baud rate
of the master mode I2C bus. Values of freqdiv are found in a the I2FDR table found in
the I2C section of the MCF54415 reference manual provided by freescale.  0x3C=1280 which
gives a baud rate close to 100Kbits/s is the value set if parameter excluded

returns nothing
*///////////////////////////////////////////////////////////////////////////////////
#if ( defined MCF5270 || defined MCF5234 || defined MCF5208 )
void Master_I2CInit( uint8_t freqdiv = 0x16 );
#elif ( defined MCF5213 || defined MCF5282 )
void Master_I2CInit( uint8_t freqdiv = 0x15 );
#elif ( defined MCF52234 )
void Master_I2CInit( uint8_t freqdiv = 0x37 );
#elif ( defined MCF5441X )
void Master_I2CInit( uint8_t freqdiv = 0x3C );
#else
#error Master_I2CInit declaration missing for defined platform
#endif

/*----------------------------------------------------------------------------
int I2CSendBuf(uint8_t addr, puint8_t buf, int count, bool stop = true);

Sends 'num' uint8_tS of buffer 'buf' on the I2C bus to address 'addr'
The transmission is then terminated with a stop signal if user either calls
the function with no 'stop' parameter or a stop = true.  If user wishes to terminate
differently such as a restart then 'stop' can be set false and a NetBurner advanced
I2Cfunction can be used for termination.

returns the result state of the I2C bus
*///////////////////////////////////////////////////////////////////////////////////
uint8_t Master_I2CSendBuf(uint8_t addr, puint8_t buf, int num, bool stop = true);

/*----------------------------------------------------------------------------
int I2CReadBuf(uint8_t addr, puint8_t buf, int count, bool stop = true);

Reads 'num' uint8_tS of buffer 'buf' on the I2C bus to address 'addr'
The transmission is then terminated with a stop signal if user either calls
the function with no 'stop' parameter or a stop = true.  If user wishes to terminate
differently such as a restart then 'stop' can be set false and a NetBurner advanced
I2Cfunction can be used for termination.

returns the result state of the I2C bus
*///////////////////////////////////////////////////////////////////////////////////
uint8_t Master_I2CReadBuf(uint8_t addr, puint8_t buf, int num, bool stop = true);


/*------------------------------------------------------------------------------
int I2CRestart( uint8_t addr, BOOL Read_Not_Write, uint32_t ticks_to_wait = I2C_RX_TX_TIMEOUT);

Restart of comunication with a slave device after finished communincation on the bus
This is used instead of a 'stop' and will allow user to communicate on bus again without
giving up control of the bus first.
'addr' is the address of the slave device we wish to comunicate with on the bus
'ticks_to_wait' is the amount of time we will wait for a start transmision to complete
this time includes the time it takes to recieve an an ack from the addressed slave device
The timeout value defined in the header file will be used if function called without
ticks_to_wait parameter
'R/W' will select whether we will be read or write the next uint8_t from slave

returns the state of the communication on the bus
*///////////////////////////////////////////////////////////////////////////////////
#define I2C_START_READ  ( 1 ) //defines to be used for bRead_Not_Write
#define I2C_START_WRITE ( 0 )
uint8_t Master_I2CRestart( uint8_t addr, bool Read_Not_Write, uint32_t ticks_to_wait = I2C_RX_TX_TIMEOUT);



/*------------------------------------------------------------------------------------------
      Advanced NetBurner I2C Functions

The following functions are used for advanced communications on using the I2C bus.  These
functions are useful if user wishes to talk to a device that does not follow the Phillips
I2C standard.  One example is an eeprom that first must be addressed and then read from
with no restart in between.
Data must be sent or recieved a byte at a time.  Also user must check function returns
to verify the status of the bus before performing the next option.
-------------------------------------------------------------------------------------------
===========================================================================================*/

/*------------------------------------------------------------------------------
int I2CStart( uint8_t addr, BOOL Read_Not_Write, uint32_t ticks_to_wait = I2C_START_TIMEOUT);

Start of comunication with a slave device
'addr' is the address of the slave device we wish to comunicate with on the bus
'ticks_to_wait' is the amount of time we will wait for a start transmision to complete
this time includes waiting for a busy bus to become active plus the time it takes
to recieve an an ack from the slave device
The timeout value defined in the header file will be used if function called without
ticks_to_wait parameter
'R/W' will select whether we will be read or write the next uint8_t from slave.

returns the state of the communication on the bus
*///////////////////////////////////////////////////////////////////////////////////
//#define I2C_START_READ  ( 1 ) //defines to be used for bRead_Not_Write
//#define I2C_START_WRITE ( 0 )
uint8_t Master_I2CStart( uint8_t addr, bool Read_Not_Write, uint32_t ticks_to_wait = I2C_START_TIMEOUT);

/*---------------------------------------------------------------------------------
void I2CStop()

Will issue a stop signal if the module has master control of the I2C bus
And will release the bus if we are a slave device

returns nothing
*///////////////////////////////////////////////////////////////////////////////////
uint8_t Master_I2CStop(uint32_t ticks_to_wait = I2C_RX_TX_TIMEOUT);

/*----------------------------------------------------------------------------
int I2CSend( uint8_t val, uint32_t ticks_to_wait = 5 );

Sends uint8_t 'val' on the I2C bus
With a timeout of 'ticks_to_wait'
Timeout will be value defined in header file if not included when function called

Returns the current state of the I2C bus
*///////////////////////////////////////////////////////////////////////////////////
uint8_t Master_I2CSend( uint8_t val, uint32_t ticks_to_wait = I2C_RX_TX_TIMEOUT );

/*----------------------------------------------------------------------------
int I2CRead( puint8_t val, uint32_t ticks_to_wait = I2C_RX_TX_TIMEOUT );

Reads uint8_t to address 'val' from the I2C bus
With a timeout of 'ticks_to_wait'
Timeout will be value defined in header file if not included when function called
Does not handle no Ack of last byte which
needs to have a I2C_SET_NO_ACK called before second to last read
The parameter IsLastRead should be set to true

Returns the current state of the I2C bus
*///////////////////////////////////////////////////////////////////////////////////
uint8_t Master_I2CRead( puint8_t val, uint32_t ticks_to_wait = I2C_RX_TX_TIMEOUT );



/* reset the bus if it hangs */
void Master_ResetPeripheral();


/*---------------------------------------------------------------------------
         Useful I2C macros
----------------------------------------------------------------------------*/
//Is I2C bus busy (bit 5 of I2SR)
#define I2C_SR_BUSY           ( ((0x20 & I2C_SR) == 0x20) )
//Is I2C bus set as slave (bit 5 of I2CR)
#define I2C_CR_SLAVE          ( ((0x20 & I2C_CR) == 0x00) )
//Was I2C bus Arbitration Lost (bit 4 of I2SR)
#define I2C_SR_ARB_LOST       ( ((0x10 & I2C_SR) == 0x10) )
//Was Device addressed as a slave(bit 6 of I2SR)
#define I2C_SR_ADRES_AS_SLAVE ( ((0x40 & I2C_SR) == 0x40) )
//Was Device addressed as a slave for TX (bit 2 of I2SR)
#define I2C_SR_SLAVE_TX       ( ((0x04 & I2C_SR) == 0x04) )
//Are we configured for TX (bit 4 of I2CR)
#define I2C_CR_TX             ( ((0x10 & I2C_CR) == 0x10) )
//Did we recieve an RX ACK after last transmit (bit 0 of I2SR)
#define I2C_SR_RX_ACK         ( ((0x01 & I2C_SR) == 0x00) )
//Are we set to RX Ack
#define I2C_CR_RX_ACK         ( ((0x08 & I2C_CR) == 0x00) )
//Configure I2C not to send a RX ACK (bit 3 of I2CR)
#define I2C_SET_NO_ACK        ( (I2C_CR |= 0x08) )
//Configure I2C to send a RX ACK (bit 3 of I2CR)
#define I2C_SET_ACK           ( (I2C_CR &= 0xF7) )
//Configure I2C to be in TX mode (bit 4 of I2CR)
#define I2C_SET_TX            ( (I2C_CR |= 0x10) )
//Configure I2C to be in RX mode (bit 4 of I2CR)
#define I2C_SET_RX            ( (I2C_CR &= 0xEF) )
//Configure I2C to send a repeated start signal
#define I2C_SET_REPEAT_START  ( (I2C_CR |= 0x04) )
//Clear Arbitration lost error condition
#define I2C_CLR_ARB_LOST      ( (I2C_SR &= 0xEF) )
//----------------------------------------------------------------------------


#endif // #ifdef _I2CMULTI_H
#endif // #ifndef _I2CMASTER_H
