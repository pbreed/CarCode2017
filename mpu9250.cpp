#include <predef.h>
#include <init.h>
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include "i2cmaster.h"
#include <pins.h>
#include <math.h>
#include <pin_irq.h>
#include <math.h>
#include <config_obj.h>
#include "introspec.h"
#include "MPU9250.h"
#include "SinLookUp.h"

LOGFILEINFO;


START_INTRO_OBJ(IMUReportObj,"IMUReport")
int16_element ax{"ax"};
int16_element ay{"ay"};
int16_element az{"az"};
int16_element gz{"gz"};
float_element head{"head"};
float_element  rhead{"rhead"};
uint32_element axsum{"axsum"};
uint32_element lidar{"lidar"};
uint32_element lidmin{"lmin"};
uint32_element odo{"Odo"};
uint32_element dtodo{"DtOdo"};
int32_element  corner{"corner"};
END_INTRO_OBJ;


START_INTRO_OBJ(CompassReportObj,"MagReport")
int16_element mx{"mx"};
int16_element my{"my"};
int16_element mz{"mz"};
//int32_element li{"li"};
//float_element lf{"lf"};
float_element mh{"mhead"};
END_INTRO_OBJ;


START_INTRO_OBJ(MagLimitReportObj,"MagLimit")
int16_element maxx{"maxx"};
int16_element maxy{"maxy"};
int16_element maxz{"maxz"};
int16_element minx{"minx"};
int16_element miny{"miny"};
int16_element minz{"minz"};
END_INTRO_OBJ;


class CompassCal:public config_obj
{
public:
CompassCal():config_obj(appdata,"CompassCal") {};
    config_int MaxC0{-32000,"MaxC0"};
    config_int MaxC1{-32000,"MaxC1"};
    config_int MaxC2{-32000,"MaxC2"};
	config_int MinC0{32000,"MinC0"};
    config_int MinC1{32000,"MinC1"};
    config_int MinC2{32000,"MinC2"};
    config_int SlewLimit{(16384/250),"SlewLimit"};
    config_int SlewSec{30,"SlewSec"};
	config_bool bUpdateRec{true,"Update"};
	ConfigEndMarker;
};


class LidarCornerParams:public config_obj
{
public:
LidarCornerParams():config_obj(appdata,"LidarCorner") {};
    config_int det_diff{400000,"diff"};
	ConfigEndMarker;
};


CompassCal compass_cal;
LidarCornerParams lidar_corner;


volatile bool bCompassCalDirty;

static IMUReportObj	    ImuR;
static CompassReportObj	CompR;
static MagLimitReportObj MagLim;


/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
*/


// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02


#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

void initAK8963(float * destination);
void initMPU9250();











uint8_t I2CWriteReadBuffer(uint8_t address, uint8_t writeVal, uint8_t *buffer, int readLength)
{
    int status;

    if(readLength <= 0)
    {
        // Non valid length. Return state 10, "Read Length Not Supported"
        return 10;
    }

    status = I2CSendBuf(address,&writeVal, 1, false);

    if(status > I2C_MASTER_OK)
    {
        // I2C Bus is in an undesired state. Return the state
        return status;
    }
    status = I2CRestart(address, I2C_START_READ);
    if ((status > I2C_MASTER_OK) || (status == I2C_NEXT_WRITE_OK))
    {
        // I2C Bus is in an undesired state. Return the state
        return status;
    }

   status = I2CReadBuf(address,buffer, readLength, true);

    if((status >= I2C_NEXT_WRITE_OK) && (status <= I2C_MASTER_OK))
    {
        I2CResetPeripheral();

    }

    return status;
}


void WriteReg(uint8_t addr,uint8_t reg, uint8_t val)
{
uint8_t buf[10];
buf[0]=reg;
buf[1]=val;
I2CSendBuf(addr,buf, 2, true);
}

uint8_t ReadReg(uint8_t addr, uint8_t reg)
{

    uint8_t buf[10];
    I2CWriteReadBuffer(addr,reg,buf,1);
    return buf[0];
}




void ScanI2C()
{
    I2CResetPeripheral();

 iprintf("Scanning I2C \r\n");

 for (int x = 1; x < 0x80; x++)
 {
     int result = I2CStart(x, I2C_START_WRITE, 1);
     if (result < I2C_TIMEOUT)
     {
         iprintf("Found device at  0x%X, Result: %d\r\n", x, result);
         I2CStop();
     }
     else
     {
         I2CStop();
         I2CResetPeripheral();
     }
 }

 iprintf("Scan complete\r\n");
}


void ReadManyReg(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    I2CWriteReadBuffer(address,subAddress,dest,count);
}



OS_SEM PirqSem;

volatile uint32_t PirqCount;

void Pirq(void)
{
PirqSem.Post();
PirqCount++;
}

void IMUSampleTask(void *p);

void Mpu9250setup(int TaskPrio)
{


    I2CInit();



  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = ReadReg(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  iprintf("MPU9250 Whoami returns %02X\r\n",c);

  if (c != 0x71) // WHO_AM_I should always be 0x71
  {
      iprintf("Attempting to unclod I2C bus\r\n");
      Pins[27].function(PIN_27_GPIO);//I2C for IMU
      Pins[29].function(PIN_29_GPIO);//I2C For IMU
      Pins[29].hiz();

      for(int i=0; i<256; i++)
      {
         Pins[27]=1;
         for (volatile int d=0; d<100; d++) asm(" nop");
         Pins[27]=0;
         for (volatile int d=0; d<100; d++) asm(" nop");
      }



   Pins[27].function(PIN_27_I2C0_SCL    );//I2C for IMU
   Pins[29].function(PIN_29_I2C0_SDA    );//I2C For IMU
   I2CResetPeripheral();
   OSTimeDly(2);
   c = ReadReg(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
   iprintf("Retry ID=%02X\r\n",(int)c);
   if(c!=0x71) ScanI2C();
  }



  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
	  bIMU_Id=true;
    iprintf("MPU9250 is online...");
			
    initMPU9250();
	PirqSem.Init();

    OSSimpleTaskCreatewName(IMUSampleTask,TaskPrio,"IMU");
	SetPinIrq(50, 1,Pirq);



    iprintf("MPU9250 initialized for active data mode....\r\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    uint8_t d = ReadReg(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    iprintf("Whoami for magnatometer is %02X\r\n",d);
   PirqSem.Post();
  }

  else
  {
    iprintf("Could not connect to MPU9250: %02x\r\n",c);
  }
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================




void ReadGAData(int16_t * dest)
{
ReadManyReg(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, (uint8_t *)dest);  // Read the seven raw data registers into data array

}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  ReadManyReg(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  ReadManyReg(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(ReadReg(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  ReadManyReg(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
   }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  ReadManyReg(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  WriteReg(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  OSTimeDly(1);;
  WriteReg(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  OSTimeDly(1);;
  ReadManyReg(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  WriteReg(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  OSTimeDly(1);;
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  WriteReg(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  OSTimeDly(1);;
}


void initMPU9250()
{
 // wake up device
  WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  OSTimeDly(5);; // Wait for all registers to reset

 // get stable time source
  WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  OSTimeDly(10);;

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  WriteReg(MPU9250_ADDRESS, CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  WriteReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = ReadReg(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  WriteReg(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = ReadReg(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  WriteReg(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = ReadReg(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  WriteReg(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   WriteReg(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
   WriteReg(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

   WriteReg(AK8963_ADDRESS ,AK8963_CNTL ,0x16);

   OSTimeDly(5);;
}



float DegScale(int32_t v);
float CalcMagHeading();


volatile uint32_t LastIerror;
volatile uint32_t nCor;
volatile float lferror;


volatile uint32_t IMUSample;
volatile uint32_t CompassCount;
volatile int16_t IMUResults[8];
volatile int16_t CompassResult[3];
volatile int16_t RotVel[3];
volatile IMUModeEnum ImuMode;
volatile int32_t GAxisSum[3];
volatile int32_t AAxisSum[3];
volatile int32_t RawHeadSum;

volatile float MagHeading;
volatile float IntegratedHeading;
volatile float RawHeading;
volatile bool  bIMU_Id;


int32_t GZero[3];
int32_t AZero[3];

int32_t GZeroCalc[3];
int32_t AZeroCalc[3];



int32_t ZeroCount;



void ProcessIMUResults()
{
switch(ImuMode)
    {case eCalibrating:

        {
            GZero[0]+=IMUResults[4];
            GZero[1]+=IMUResults[5];
            GZero[2]+=IMUResults[6];
			AZero[0]+=IMUResults[0];
			AZero[1]+=IMUResults[1];
			AZero[2]+=IMUResults[2];
            ZeroCount++;
			GZeroCalc[0]=GZero[0]/ZeroCount;
			GZeroCalc[1]=GZero[1]/ZeroCount;
			GZeroCalc[2]=GZero[2]/ZeroCount;
		    AZeroCalc[0]=AZero[0]/ZeroCount;
			AZeroCalc[1]=AZero[1]/ZeroCount;
			AZeroCalc[2]=AZero[2]/ZeroCount;




        }
        break;
case eRunning :
     {
        for(int i=0; i<3; i++)
        {  int32_t a=IMUResults[0+i]-AZeroCalc[i];
           int32_t v=IMUResults[4+i]-GZeroCalc[i];
           AAxisSum[i]+=a;
		   RotVel[i]=v;
           GAxisSum[i]+=v;
           if (GAxisSum[i]>4718592)GAxisSum[i]-=2*4718592;
           if (GAxisSum[i]<-4718592)GAxisSum[i]+=2*4718592;
		   if(i==2)
		   {
			   RawHeadSum+=v;
			   if (RawHeadSum> 4718592)RawHeadSum-=2*4718592;
			   if (RawHeadSum<-4718592)RawHeadSum+=2*4718592;
		   }
        }
		IntegratedHeading=DegScale(GAxisSum[2]);
		RawHeading=DegScale(RawHeadSum);

     }
     break;
case eIdle:
	 break;
    }


}


/*static float SinSum;
static float CosSum;
static int MagCalCount;
*/
void DoCalMagHeading(float f)
{
/* SinSum+=LookUpSinDeg(f);
 CosSum+=LookUpCosDeg(f);
 MagCalCount++;
*/
}

void SetInitalCompass()
{
/*float dx=SinSum;
float dy=CosSum;
uint32_t n=MagCalCount;
dx/=n;
dy/=n;
dx=Fast_atan2(dy,dx);
dx*=180.0/3.141592654;
dy=dx;
dx*=200.0*32768.0/250.0;
GAxisSum[2]=(int32_t)dx;
*/
double a=MagHeading;
a*=-(200*32768.0)/250.0;
int32_t iv=(int)a;
GAxisSum[2]=iv;
//printf("Set inital heading to %g and %g for %d\r\n",dy,DegScale(GAxisSum[2]),n);
printf("MagHeading=%g %g\r\n",MagHeading, DegScale(GAxisSum[2]));
}






void SetGyroCompass(int Slew_Seec);


void ProcessCompassResults()
{
bool bWasDirty=bCompassCalDirty;


if(compass_cal.bUpdateRec)
{

if(CompassResult[0] >compass_cal.MaxC0) {compass_cal.MaxC0=CompassResult[0]; bCompassCalDirty=true; }
if(CompassResult[1] >compass_cal.MaxC1) {compass_cal.MaxC1=CompassResult[1]; bCompassCalDirty=true; }
if(CompassResult[2] >compass_cal.MaxC2) {compass_cal.MaxC2=CompassResult[2]; bCompassCalDirty=true; }

if(CompassResult[0] <compass_cal.MinC0) {compass_cal.MinC0=CompassResult[0]; bCompassCalDirty=true; }
if(CompassResult[1] <compass_cal.MinC1) {compass_cal.MinC1=CompassResult[1]; bCompassCalDirty=true; }
if(CompassResult[2] <compass_cal.MinC2) {compass_cal.MinC2=CompassResult[2]; bCompassCalDirty=true; }

}
MagHeading=CalcMagHeading();

if(ImuMode==eCalibrating)
{
	DoCalMagHeading(MagHeading);
}




if((RotVel[2]< (compass_cal.SlewLimit)) && (RotVel[2]>-(compass_cal.SlewLimit)))
{
       SetGyroCompass(compass_cal.SlewSec);
}

if((bWasDirty!=bCompassCalDirty) && (!bWasDirty))
{
MagLim.maxx=compass_cal.MaxC0;
MagLim.maxy=compass_cal.MaxC1;
MagLim.maxz=compass_cal.MaxC2;
MagLim.minx=compass_cal.MinC0;
MagLim.miny=compass_cal.MinC1;
MagLim.minz=compass_cal.MinC2;
MagLim.Log();
}


}


extern volatile uint32_t LIDAR_VALUE;
extern volatile uint32_t LIDAR_MIN;
extern volatile uint32_t OdoCount;
extern volatile uint32_t DtOdoCount;

void ProcessNewImuData();

void IMUSampleTask(void *p)
{
int16_t dest[8];
static uint32_t PrevOdo;
static uint32_t PLidarArray[5];


while(1)
 {
 PirqSem.Pend();
 if (ReadReg(MPU9250_ADDRESS, INT_STATUS) & 0x01)
 {  // On interrupt, check if data ready interrupt
	 ReadGAData(&dest[0]);
	 USER_ENTER_CRITICAL();
	 for(int i=0; i<8; i++)
		 IMUResults[i]=dest[i];
	 IMUSample++;
	 uint32_t lm=LIDAR_MIN;
	 LIDAR_MIN=0x7FFFFFFF;
     USER_EXIT_CRITICAL();
     ProcessIMUResults();
	 ImuR.ax=IMUResults[0];
     ImuR.ay=IMUResults[1];
	 ImuR.az=IMUResults[2];
	 ImuR.gz=IMUResults[6];

	 ImuR.lidar=LIDAR_VALUE;
	 ImuR.lidmin=lm;
	 ImuR.odo=OdoCount;
	 ImuR.dtodo=DtOdoCount;
     ImuR.head=IntegratedHeading;
	 ImuR.corner=0;
	 ImuR.rhead=RawHeading;
	 ImuR.axsum=AAxisSum[0];

	 if(PrevOdo!=OdoCount)
	 {
		 PLidarArray[4]=PLidarArray[3] ;
		 PLidarArray[3]=PLidarArray[2] ;
		 PLidarArray[2]=PLidarArray[1] ;
		 PLidarArray[1]=PLidarArray[0] ;
         PLidarArray[0]=LIDAR_VALUE;

		 uint32_t a1=(PLidarArray[3]+PLidarArray[4])/2;
		 uint32_t a0=(PLidarArray[1]+PLidarArray[0])/2;

		 if (a1>(a0+lidar_corner.det_diff))
		 {
		  if((PLidarArray[4]>a0) && (PLidarArray[3]>a0))
		       ImuR.corner=1;//From open to corner toward me
		 }
		 else
	    if(a0>(a1+lidar_corner.det_diff))
	    {
			if((PLidarArray[4]<a0) && (PLidarArray[3]<a0))
			ImuR.corner=-1; //from wall to open corner
	    }
	 }

	 ImuR.Log();



	
     uint8_t cstatus=ReadReg(0xC,2);
	 if(cstatus& 1)
	 {
	 uint8_t buf[10];
	 I2CWriteReadBuffer(AK8963_ADDRESS ,AK8963_XOUT_L, buf,7); //Reading 7 to get ST2 register
	 uint8_t * p8=(uint8_t *)&CompassResult[0];
	 USER_ENTER_CRITICAL();
	 p8[0]=buf[1];
	 p8[1]=buf[0];
	 p8[2]=buf[3];
	 p8[3]=buf[2];
	 p8[4]=buf[5];
	 p8[5]=buf[4];
	 CompassCount++;
	 USER_EXIT_CRITICAL();
	 ProcessCompassResults();
	 CompR.mx=CompassResult[0];
     CompR.my=CompassResult[1];
	 CompR.mz=CompassResult[2];
	 CompR.mh=MagHeading;
//	 CompR.li=LastIerror;
//	 CompR.lf=lferror;
	 CompR.Log();
	 }
 }
 ProcessNewImuData();
}
}




float MScale(int16_t iv,int16_t maxv, int16_t minv)
{
  float v=iv;
  float md=maxv;
  float nd=minv;
  return ((v-(nd+md)/2.0)*2.0)/(md-nd);
}

float CalcMagHeading()
{
float y=MScale(CompassResult[0],compass_cal.MaxC0,compass_cal.MinC0);
float x=MScale(CompassResult[1],compass_cal.MaxC1,compass_cal.MinC1);
float a=Fast_atan2(y,x);
return a*180.0/3.141592654;
}




float DegScale(int32_t v)
{
float dv=v;
dv/=200;//Samples per sec
dv*=(250.0/32768.0);//rate
return -dv;
}


void SetGyroCompass(int Slew_Sec)
{
float error=IntegratedHeading-MagHeading;   //-115
static float error_rem;

while (error<-180.0) error+=360;
while (error>180.0) error-=360;

lferror=error;

if (Slew_Sec) error/=(float)(100.0*Slew_Sec); //100 sps compass * Seconds to slew

if(Secs<20) error*=4000; //Slew it all early;
else
if(Secs<60) error*=400; //Slew it all early;

//Error now = error time number of seconds to slew full amount


error/=(250.0/32768.0);
//Error now scaled in counts/SPS since 250 deg/sec is 32768 so we are in counts...

error+=error_rem;


int32_t ierror=(int32_t)error;

error_rem=(error-ierror);

    LastIerror=ierror;
//error now in counts...
GAxisSum[2]+=ierror; //GAxisSum and inteegrated Heading are opposite sign.

nCor++;

}




