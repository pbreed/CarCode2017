
extern volatile uint32_t UResult[360];
extern volatile uint32_t ssUResult[360];
extern volatile uint32_t LidarScanCount;
extern volatile uint32_t LidarRxc;

void InitLidar(int port,int TaskPrio, int CalcTaskPrio);

enum LidarWallMode {eOff, eRight,eLeft, eCalculating, eDoneLeft, eDoneRight};


bool LidarBusy();
int GetLidarResult(int32_t & b,uint32_t &tc, uint32_t &pc);
void LidarSampleStart(LidarWallMode mode);



struct IntPoint
{
int x;
int y;
};

#define MAX_LIDAR_POINTS (48)

extern volatile IntPoint LidarPointSet[MAX_LIDAR_POINTS];
extern volatile uint32_t LidarPointCount;
extern volatile bool bLidarRight;
extern volatile bool bLidarLeft;


