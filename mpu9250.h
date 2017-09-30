extern volatile uint32_t IMUSample;
extern volatile uint32_t CompassCount;
extern volatile int16_t IMUResults[8];

extern volatile int16_t RotVel[3];

extern volatile int16_t CompassResult[3];


void Mpu9250setup(int prio);


enum IMUModeEnum {eIdle,eCalibrating,eRunning};


extern volatile IMUModeEnum ImuMode;


extern volatile int32_t AxisSum[3];

extern volatile float MagHeading;
extern volatile float IntegratedHeading;
extern volatile bool bIMU_Id;

extern volatile bool bCompassCalDirty;

void SetInitalCompass();



