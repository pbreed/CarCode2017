
/* RC reciever channels 
0 Throttle
1 Aileron
2 Elevator
3 Rudder
4 Gear
5 Aux 1
6 Aux 2
7 Aux 3
8 Aux 4


Scale is 1024=center range 0 to 2048
*/






extern volatile uint32_t  rc_ch[16];
extern volatile uint32_t RCFrameCnt;
extern void ( * RCCallBack)(uint32_t ch, uint32_t v);
void InitDSM2Rx(int serial_port);




