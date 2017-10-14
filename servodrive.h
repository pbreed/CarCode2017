void ServoDriveInit();
/* Value is -1.0 ... 1.0 */
void SetServoPos(int servo, double v);
void SetServoRaw(int servo, int v);
//void SetServoHighRaw(int servo, int v);
int GetServoCount(int servo);

inline float HiCon(int v) {return (float)((float)(v-1024)/500.0);};
extern OS_SEM * pNotifyNextFrameSem;
extern volatile uint32_t ServoFrameCnt;

#define NUM_SERVO_DRIVE (3)

