#define RC_LOG       0x3A31 //Full RC frames
#define LIDAR_LOG    0x3A32 //Indiviidual LASER REadings
#define IMU_LOG      0x3A33 //FUll IMU record
#define COMPASS_LOG  0x3A34 //Full Compass Record
#define ODO_LOG    	 0x3A35 //ODO count
#define MESSAGE_LOG  0x3A36 //Text Messages


					  
extern volatile bool bLog;					  

void Log(uint16_t log_type, void * pData, int datalen);
void LogMessage(const char * msg);
int GetLogUsedCount();

void  LogGetData(unsigned char * &cp,int &len);
void  LogClear();
	  
