
extern volatile uint32_t UResult[360];
extern volatile uint32_t ssUResult[360];
extern volatile uint32_t LidarScanCount;
extern volatile uint32_t LidarRxc;

void InitLidar(int port,int TaskPrio);


struct IntPoint
{
int x;
int y;
};

#define MAX_LIDAR_POINTS (256)

extern volatile IntPoint LidarPointSet[MAX_LIDAR_POINTS];
extern volatile uint32_t LidarPointCount;
extern volatile bool bLidarRight;
extern volatile bool bLidarLeft;


