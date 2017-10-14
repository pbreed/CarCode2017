
float LookUpSinRad(float rads);
float LookUpCosRad(float rads);
float LookUpSinDeg(float deg);
float LookUpCosDeg(float deg);
float Fast_atan2( float y, float x );
void  Normalize(float & x, float & y);
void SetUpTables(float dist);

void LookUpSinCosDistIndex(unsigned long index, float & s ,float & c);
void LookUpSinCosIndex(unsigned long index, float & s ,float & c);
unsigned long ConvertDegToIndex(float deg);
float ConvertIndexToFloat(unsigned long index);
int32_t AvgHIndex(int32_t h1,int32_t h2);





extern  const float AX128ToSinCosInIn[46080][2];



