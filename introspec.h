#ifndef INTROSPECTION_H
#define INTROSPECTION_H




class IntrospecObject;
struct IntroSpecDescription;

class IntrospecObject
{
    IntroSpecDescription * pDesc;
const char * m_name;
public:
static void Element(void * p, size_t siz,const char * label, char c);
static bool bShowMe;
IntrospecObject(const char * name);
virtual void GetExtent(uint32_t * & sm, uint32_t * &em)=0;
virtual void BuildOne()=0;
void LogStructure();
void Log(bool ForceLog=false);
};


class START_MARKER_OBJ
{
public:
 uint32_t pItem;
    START_MARKER_OBJ() {if(IntrospecObject::bShowMe) IntrospecObject::Element(this,sizeof(*this),NULL,0); };
};


#define START_INTRO_OBJ(name,label) \
class name : public IntrospecObject \
{ \
public:     \
     name():IntrospecObject(#name){}; \
private: \
virtual void BuildOne() {name ni;}; \
public: \
START_MARKER_OBJ start_marker;



#define END_INTRO_OBJ \
uint32_t end_marker; \
virtual void GetExtent(uint32_t * & sm, uint32_t * & em) {sm=&start_marker.pItem; em=&end_marker;}; \
}



template <typename T> class INTROIEL {
public:
 T val;
 INTROIEL(const char * label) {val=0; if(IntrospecObject::bShowMe) IntrospecObject::Element(this,sizeof(*this),label,'s'); };
 inline T operator=(T rhs) { val = rhs; return rhs;}
 inline operator T() const { return val; }
 inline T operator^=(T rhs) { val ^= rhs; return val; }
 inline T operator|=(T rhs) { val |= rhs; return val; }
 inline T operator&=(T rhs) { val &= rhs; return val; }
};

template <typename T> class INTROUEL {
public:
 T val;
 INTROUEL(const char * label) {val=0; if(IntrospecObject::bShowMe) IntrospecObject::Element(this,sizeof(*this),label,'u'); };
 inline T operator=(T rhs) { val = rhs; return rhs;}
 inline operator T() const { return val; }
 inline T operator^=(T rhs) { val ^= rhs; return val; }
 inline T operator|=(T rhs) { val |= rhs; return val; }
 inline T operator&=(T rhs) { val &= rhs; return val; }
};

template <typename T> class INTROFEL {
public:
 T val;
 INTROFEL(const char * label) {val=0; if(IntrospecObject::bShowMe) IntrospecObject::Element(this,sizeof(*this),label,'f'); };
 inline T operator=(T rhs) { val = rhs; return rhs;}
 inline operator T() const { return val; }
 inline T operator^=(T rhs) { val ^= rhs; return val; }
 inline T operator|=(T rhs) { val |= rhs; return val; }
 inline T operator&=(T rhs) { val &= rhs; return val; }
};



typedef INTROIEL<int8_t>    int8_element;
typedef INTROIEL<int16_t>   int16_element;
typedef INTROIEL<int32_t>   int32_element;
typedef INTROIEL<char>      char_element;
typedef INTROIEL<int>       int_element;
typedef INTROIEL<short>     short_element;


typedef INTROUEL<uint8_t>   uint8_element;
typedef INTROUEL<uint16_t>  uint16_element;
typedef INTROUEL<uint32_t>  uint32_element;

typedef INTROFEL<float>     float_element;
typedef INTROFEL<double>    double_element;



void LogMessage(const char * cp, bool force=false);
void LogEvent(bool force=false);
void InitLogFtp(int prio);
int GetLogPercent();
int GetLogSize();

extern volatile bool bLog;					  

void LogFileVersions();
void LogAppRecords();

class FileLog
{
public:
const char * LogMsg;
FileLog * pNext;
	FileLog(const char * LogMsg);
};


#define LOGFILEINFO static FileLog fl("Source " __FILE__ " build on" __DATE__ "at" __TIME__ ".");

#endif
