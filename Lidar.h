
extern volatile uint32_t UResult[360];
extern volatile uint32_t ssUResult[360];
extern volatile uint32_t LidarScanCount;
extern volatile uint32_t LidarRxc;

void InitLidar(int port,int TaskPrio, int CalcTaskPrio);

enum LidarWallMode {eOff, eRight,eLeft, eCalculating, eDoneLeft, eDoneRight};

const char * GetLidarStateName(LidarWallMode lm);


bool LidarBusy();
//Returns slope number 0->256
//Tc=total count
//b=offset l/r

int GetLidarResult(int32_t & b,uint32_t &tc, uint32_t &pc);
void LidarSampleStart(LidarWallMode mode);

float SlopeNumToDeg(int SlopeNum); 
int GetLidarHeading(int DesiredHead);

bool DoPedStop();


struct IntPoint
{
int x;
int y;
};

#define MAX_LIDAR_POINTS (48)

extern volatile IntPoint LidarPointSet[MAX_LIDAR_POINTS];
extern volatile uint32_t LidarPointCount;


//Bit mapped 11111111 means -90..+90 clear
//           00000000 means nothing open.
//           00001111 means left side blocked...
//           11110000 means right side blocked.
extern volatile uint8_t warning_byte;


