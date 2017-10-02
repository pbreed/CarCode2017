#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <errno.h>
#include <iostream>
#include <string>

using namespace std;


#define KEY_START (250)
#define KEY_ESCAPE (249)
#define ELEMENT_DESCRIBE (99)
#define KEY_DESCRIBE     (98)
#define KEY_EVENTN       (97)
#define KEY_MESSAGE      (96)


enum Action_t {eListNames, eCountObj, eDisplayData, eVerbose};


Action_t gAction;
bool bShowMessages;
bool bShowEvents;
bool bShowLastVal;
bool bDataOn;
bool bEmitAll;


unsigned int  event_count;
unsigned int  msg_count;

const int id_idle =0;
const int id_after_start =-1;

class DisplayItem; //forward

class FieldDescription
{
public:

 FieldDescription * pNext;
 unsigned int offset;
 unsigned int len;
 char vtype;
 string name;
 string display_value;
 bool bDoDisplay;
 bool bEmitField;
 FieldDescription() {bEmitField=false; bDoDisplay=false; offset=0; len=0; name=""; vtype='?'; pNext=0; display_value="";};
 void Show();
 void DataShow(unsigned char* p );
 void NumShow(unsigned char * pD, int len,bool issigned);
 void FloatShow(unsigned char * pD);
 void DoubleShow(unsigned char * pD);


};


extern string Eval(const string & s, const string & rpn);
class ObjectDescription
{
public:
static ObjectDescription * pGlobalHead;
ObjectDescription * pNext;
 unsigned int key;
 unsigned int len;
 unsigned int num_rxed;
 string name;
 FieldDescription * pHead;
    ObjectDescription()
    {
     key=0;
     len=0;
     name="";
     pHead=0;
     pNext=pGlobalHead;
     num_rxed=0;
     pGlobalHead=this;
     };

    ~ObjectDescription()
    {
        printf("Should never call destructor\n");
    };
static ObjectDescription *  FindDescription(int id);
void ShowData(unsigned char * data, int len);
void Show();
};




class DisplayItem
{

public:
static bool bEmitDataLine;
string Rpn;
DisplayItem * pNext;
string ObjName;
string FieldName;
string dname;
FieldDescription *Field;
bool EmitLineOnRx;
string DisplayElement;
DisplayItem(const char * compound_name);
static void PossiblyAttach(ObjectDescription * po, FieldDescription * fd);
static void ObjectEnd();
static void EmitHeaders();
};


bool DisplayItem::bEmitDataLine;


DisplayItem * pDisplayHead;
DisplayItem * pMsgDisplay;
DisplayItem * pEventDisplay;


DisplayItem::DisplayItem(const char * compound_name)
{
ObjName="";
FieldName="";
Rpn="";
DisplayElement="";
Field=0;

//printf("Parsing compound name %s\r\n",compound_name);
const char * cp=compound_name;
while ((*cp) && (*cp!='.') && (*cp>' '))
    {
     ObjName+=*cp++;
    }


if(*cp=='.')
{
  cp++;
}
while ((*cp) && (*cp>' ')) FieldName+=*cp++;

DisplayItem * pd=pDisplayHead;
 pNext=0;

if(pd)
{
while(pd->pNext) pd=pd->pNext;
pd->pNext=this;
}
else
{
 pDisplayHead=this;
}

if(strstr(cp,"name:"))
{
const char * np=strstr(cp,"name:");
if(np) np+=5;
while((*np) && (!isspace(*np))) dname+=(*np++);
}
else
{
dname=ObjName+'.'+FieldName;
}


if((strstr(cp,"emit")) || (bEmitAll))
{
//printf("Emit set to tru for %s\r\n",dname.c_str());
EmitLineOnRx=true;
}

if(strstr(cp,"rpn"))
{
const char * rp=strstr(cp,"rpn");
if(rp) rp+=3;
Rpn=rp;
}


if(FieldName.empty())
{
   if(ObjName=="messages")
   {
       pMsgDisplay=this;
	   dname="messages";
   }
   else
   if(ObjName=="events")
   {
      pEventDisplay=this;
	  dname="events";

   }
}

//printf("New Display item[%s].[%s] %s\r\n",ObjName.c_str(),FieldName.c_str(),compound_name);

}

void DisplayItem::EmitHeaders()
{
DisplayItem * pd=pDisplayHead;

while(pd)
{
   printf("\"%s\"",pd->dname.c_str());
   pd=pd->pNext;
   if(pd) printf(",");
}
printf("\n");
}


void DisplayItem::ObjectEnd()
{
   //printf(".");
   if(bEmitDataLine)
   {
       DisplayItem * pd=pDisplayHead;

       while(pd)
       {

		if(pd->Field)
		{

		string raw_val=pd->Field->display_value;
        if (!raw_val.empty())
		{
			if(pd->Rpn.empty())
				{		
				pd->DisplayElement=raw_val;
				}
				else
				{
				 pd->DisplayElement=Eval(raw_val,pd->Rpn);
				}

		}
		}

		printf("%s",pd->DisplayElement.c_str());

           pd=pd->pNext;

           if(pd) printf(",");
       }
       printf("\n");
       if(bShowLastVal)
       {
        if(pMsgDisplay)   pMsgDisplay->DisplayElement="";
        if(pEventDisplay) pEventDisplay->DisplayElement="";
       }

      bEmitDataLine=false;
		
	      pd=pDisplayHead;
		  while(pd)
		   {
			 if(!bShowLastVal) pd->DisplayElement="";
			 if(pd->Field) pd->Field->display_value="";
		     pd=pd->pNext;
		   }
	
   }
}


void DisplayItem::PossiblyAttach(ObjectDescription * po, FieldDescription * fd)
{
DisplayItem * pd=pDisplayHead;
while(pd)
{
   if((pd->ObjName==po->name) && (pd->FieldName==fd->name))
   {
    pd->Field=fd;
	fd->bDoDisplay=true;
    if(pd->EmitLineOnRx)
    {
     fd->bEmitField=true;
    }
//    printf("Attached %s.%s to fd\n",po->name.c_str(),fd->name.c_str());
   }
 pd=pd->pNext;
}
}










ObjectDescription * ObjectDescription::pGlobalHead;


ObjectDescription *  ObjectDescription::FindDescription(int id)
{
ObjectDescription * pTest=pGlobalHead;
while(pTest)
{
    if(pTest->key==id) break;
    pTest=pTest->pNext;
}

return pTest;
}


void FDRecurData(unsigned char * pData,FieldDescription * pF)
{
   if(pF)
   {
    FDRecurData(pData,pF->pNext);
    pF->DataShow(pData);
   }
}




void ObjectDescription::ShowData(unsigned char * data, int len)
{
num_rxed++;

if(gAction==eVerbose)  printf("%s:[\n",name.c_str());
if(data)
 {
   FDRecurData(data,pHead);
 }
if(gAction==eVerbose)printf("\n]\n");
}






void FDRecurShow(FieldDescription * pF)
{
   if(pF)
   {
    FDRecurShow(pF->pNext);
    pF->Show();
   }
}

void ObjectDescription::Show()
{
if(gAction==eVerbose) printf("Object ID: %d Name:%s\nElements:\n",key,name.c_str());
 FieldDescription * pF=pHead;
 FDRecurShow(pF);
}

void FieldDescription::Show()
{
if(gAction==eVerbose) printf("Element [%s]%c %d long at %X\n",name.c_str(),vtype,len,offset);
}




int GetInt8Val(unsigned char * pD)
{
 return (int)((char)pD[0]);
}

int GetInt16Val(unsigned char *pD)
{
volatile short s;
unsigned char * p=(unsigned char * )&s;
p[0]=pD[1];
p[1]=pD[0];
return (int)s;
}

int GetInt32Val(unsigned char *pD)
{
volatile int i;
unsigned char * p=(unsigned char * )&i;
p[3]=pD[0];
p[2]=pD[1];
p[1]=pD[2];
p[0]=pD[3];
return i;
}







void FieldDescription::NumShow(unsigned char * pD, int len,bool issigned)
{

	char buffer[80];

  if(issigned)
  {
	switch(len)
	{
	case 1: sprintf(buffer,"%+i",GetInt8Val(pD)); break;
	case 2: sprintf(buffer,"%+i",GetInt16Val(pD)); break;
	case 4: sprintf(buffer,"%+i",GetInt32Val(pD)); break;
	}
  }
  else
  {
  unsigned int v=0;
   for(int i=0; i<len; i++)
   {
       v=(v<<8)|pD[i];
   }
   sprintf (buffer,"%u",(unsigned)v);
  }

   if ((bDataOn) && (bDoDisplay))
   {
	display_value=buffer;
	DisplayItem::bEmitDataLine|=bEmitField;
   }
   if(gAction==eVerbose) printf(buffer);
}


void FieldDescription::FloatShow(unsigned char * pD)
{
 volatile float f;
 unsigned char * pF=(unsigned char *)&f;
 pF[0]=pD[3];
 pF[1]=pD[2];
 pF[2]=pD[1];
 pF[3]=pD[0];
 char buffer[80];
 sprintf(buffer,"%f",f);
 if(gAction==eVerbose)  printf(buffer);
 if ((bDataOn) && (bDoDisplay))
 {
  display_value=buffer;
  DisplayItem::bEmitDataLine|=bEmitField;
 }
}


void FieldDescription::DoubleShow(unsigned char * pD)
{
 volatile double d;
 unsigned char * pF=(unsigned char *)&d;
 pF[0]=pD[7];
 pF[1]=pD[6];
 pF[2]=pD[5];
 pF[3]=pD[4];
 pF[4]=pD[3];
 pF[5]=pD[2];
 pF[6]=pD[1];
 pF[7]=pD[0];
 char buffer[80];
 sprintf(buffer,"%f",d);
 if(gAction==eVerbose)  printf(buffer);
 if ((bDataOn) && (bDoDisplay))
{
 display_value=buffer;
 DisplayItem::bEmitDataLine|=bEmitField;
}
}



void FieldDescription::DataShow(unsigned char * pD)
{
if(gAction==eVerbose)  printf("%s:",name.c_str());
    switch(vtype)
    {
    case 's': NumShow(pD+offset,len,true); break;
    case 'u': NumShow(pD+offset,len,false);break;
    case 'f':
                if(len==4) FloatShow(pD+offset);
                if(len==8) DoubleShow(pD+offset);
                break;
    }
if(gAction==eVerbose)  printf("\n");
}





bool Parse32(int val, unsigned int & item)
{
   if(val & 0x80)
       {
         item=(item<<7)|(val & 0x7F);
         return false;
        }
   else
        {
        item=(item<<7)|(val & 0x7F);
        return true;
        }
}



bool ParseString(int val, string & s)
{
  if(val>0)
      {s+=(char)val;
       return false;
      }
  return true;
}

//Message is jsut zero terminated string
void DispatchMessage(int val)
{static string msg;

if(ParseString(val,msg))
 {
  if(msg.length())

   {if (bShowMessages)printf("Msg[%s]\n",msg.c_str());

   if((bDataOn) && (pMsgDisplay) )
   {
       pMsgDisplay->DisplayElement=msg;
       if(pMsgDisplay->bEmitDataLine) DisplayItem::ObjectEnd();
   }
  }
  msg="";
  msg_count++;
 }
}


/*
LogRawByte(ELEMENT_DESCRIBE);
LogString(Label)
LogRawByte(c);
LogRaw32(off);
LogRaw32(siz);
*/
bool ParseField(ObjectDescription * pObj,int val)
{
   static int state;

if(!pObj) return false;
 if(val==-1)
 {
  pObj->pHead=new FieldDescription;
  state=0;
 }
 else
 {
 if(!pObj->pHead) return false;
  switch(state)
  {
  case 0:
      if(val==ELEMENT_DESCRIBE) state++;
      break;

  case 1:
      if(ParseString(val,pObj->pHead->name))
      {
       state++;
       DisplayItem::PossiblyAttach(pObj, pObj->pHead);
       //printf("Element name %s\n",pObj->pHead->name.c_str());
      }
      break;
  case 2:
       pObj->pHead->vtype=(char)val;
       state++;
      break;
  case 3:
      //printf("offset %d\n",val);
      if(Parse32(val,pObj->pHead->offset))
          state++;
      break;
  case 4:
      if(Parse32(val,pObj->pHead->len ))
          {state++;
          return true;
          }
      break;
  case 5:
      if(val==ELEMENT_DESCRIBE)
      {
         state=1;
         FieldDescription * cObj=pObj->pHead;
         pObj->pHead=new FieldDescription;
         pObj->pHead->pNext=cObj;
      }
  }
 }
 return false;
}




//Should only recieve 0-1 next element as events are zero length
void DispatchEvent(int val)
{
   if ((val=-1) && (bShowEvents))printf("Event\n");
   if((bDataOn) && (pEventDisplay))
   {
	  pEventDisplay->DisplayElement="Event";
      if(pEventDisplay->bEmitDataLine) DisplayItem::ObjectEnd();
   }
   event_count++;
}





//
//  LogRaw32(pDesc->KeyValue); ///Key
//  LogRaw32(pend-pstart);//Len
//  const char * cp=m_name;
//    while(*cp)
//        LogRawByte(*cp++);
//    LogRawByte(0);
void DispatchDescribe(int val)
{
static ObjectDescription * pObj;

static int state;
static unsigned int key;
static unsigned int len;
static string  Name;

if(val==-1)
{

   if(pObj)
       {
       // printf("Done with Description:");
       // pObj->Show();
       }

   pObj=new ObjectDescription();
   state=0;
   return;
}
else
{
 if(!pObj)
 {
  pObj=new ObjectDescription();
 }
 if(pObj)
 {
 switch (state)
 {
 case 0:
 if (Parse32(val,pObj->key)) state=1;
 break;
 case 1:
 if (Parse32(val,pObj->len))
     {len*=4;
      state=2;
     }
 break;
 case 2:
     {
        if(ParseString(val,pObj->name))
            {
             state=3;
             ParseField(pObj,-1);
            }
     }
   break;
 case 3: //Parsing individual fields
        ParseField(pObj,val);
     break;

 }
 }
 else
 {
   //printf("Trying to dispatch null object state=%d obj=%p\n",state,pObj);
 }
}
}



void DispatchProcess(int id, int value)
{
static unsigned char ObjBuffer[5000];
static int blen;
static unsigned int tint;


switch(id)
    {
    case KEY_MESSAGE:
    DispatchMessage(value);
    break;
    case KEY_EVENTN:
    DispatchEvent(value);
    break;
    case KEY_DESCRIBE: DispatchDescribe(value);
    break;

default:
     if(value==-1)
     {
         if(id)
         {
         ObjectDescription * pO=ObjectDescription::FindDescription(id);
         if(pO)
           {
             pO->ShowData(ObjBuffer,blen);
             if(bDataOn) DisplayItem::ObjectEnd();
           }
          else
           {

              printf("Unknown object ID %d\n",id);
           }
         }
       blen=0;
       tint=0;
     }
     else
     {
         if(Parse32(value,tint))
         {
             ObjBuffer[blen++]=((tint >>24) &0xFF);
             ObjBuffer[blen++]=((tint >>16) &0xFF);
             ObjBuffer[blen++]=((tint >>8) &0xFF);
             ObjBuffer[blen++]=((tint) &0xFF);
          tint=0;
         }
     }
    }
}



void RawProcess(int v)
{
static int id_state;
if(v==-1) //Start
{
  DispatchProcess(id_state,-1);
  id_state=id_after_start;
  return;
}
else
if(id_state==id_after_start)
{
   id_state=v;
   return;
}
else
  DispatchProcess(id_state,v);
}




void ProcessChar(unsigned char b)
{
static bool escape;

if(b==KEY_START)
{
  RawProcess(-1);
  return;
}
else
if(b==KEY_ESCAPE)
{
    escape=true;
    return;
}

if(escape)
{
  if(b==0) RawProcess(KEY_START);
  if(b==1) RawProcess(KEY_ESCAPE);
  escape=false;
  return;
}

RawProcess(b);
}




void ProcessFile(const char * fn)
{
DWORD cnt=0;
BYTE rxbuffer[257];

FILE * fp=fopen(fn,"rb");
while (!feof(fp))
	{
    cnt=0;
	cnt=fread((char *)rxbuffer,1,256,fp);
	if(cnt>0) for(int i=0; i<cnt; i++) ProcessChar(rxbuffer[i]);
	}
fclose(fp);
}






void ListNames()
{
 ObjectDescription * po=ObjectDescription ::pGlobalHead;

 if(msg_count) printf("messages\n");
 if(event_count) printf("events\n");
 while(po)
 {
     FieldDescription * pfd=po->pHead;
     while(pfd)
         {
          printf("%s.%s\n",po->name.c_str(),pfd->name.c_str());
          pfd=pfd->pNext;
         }
     po=po->pNext;
 }

}




void ListCounts()
{
 ObjectDescription * po=ObjectDescription ::pGlobalHead;

 printf("messages:%d\n",msg_count);
 printf("events:%d\n",event_count);

 while(po)
 {
     if(po->name.length())
         printf("%s:%d\n",po->name.c_str(),po->num_rxed);
     po=po->pNext;
 }

}

void ParseOptionFile(const char * fn)
{
char rxbuffer[513];

FILE * fp=fopen(fn,"r");
while (!feof(fp))
	{
     if(fgets(rxbuffer,512,fp))
     {
        rxbuffer[512]=0; //Make sure ALWAYS terminated
        if(rxbuffer[0]>20)
        {
          new DisplayItem(rxbuffer);
        }
     }

	}
fclose(fp);
}

bool bNextIsOption;
bool bListCounts;
bool bListVars;




void ParseOptions(const char * arg)
{
//printf("Parse Suboption %s\n",arg);
    switch (arg[1])
    {
    case 'A': bEmitAll=true; break;
    case 'M': bShowMessages=true; break;
    case 'E': bShowEvents=true; break;
    case 'V': gAction=eVerbose; break;
    case 'D': bDataOn=true; break;
    case 'O': bNextIsOption=true; break;
    case 'H': bShowLastVal=true; break;
    case 'C': bListCounts=true; break;
    case 'L': bListVars=true; break;




    case '?':
         printf("Usage is: read <options> file\n");
         printf("Options:\n");
         printf("-A Emit All\n");
         printf("-L list names\n");
         printf("-C count elements\n");
         printf("-D make CSV file\n");
         printf("-V verbose diagnositc.\n");
         printf("-M show messages\n");
         printf("-E show events\n");
         printf("-H Hold last valid value\n");
         printf("-O optionsfile   List of data fields to displaty in same format as list \n");
         exit(-1);
        break;
    }
}



int main(int argc, char ** argv)
{
const char * fn="Log.bin";

gAction=eCountObj;
bShowMessages=false;
bShowEvents=false;


for(int i=1; i<argc; i++)
{
if(argv[i][0]=='-')
     ParseOptions(argv[i]);
else
if(bNextIsOption)
{
  ParseOptionFile(argv[i]);
//  printf("Parsing option file %s\n",argv[i]);
  bNextIsOption=false;
}
else
fn=argv[i];
}

fprintf(stderr,"Data Read from %s\n",fn);

if(bDataOn) DisplayItem::EmitHeaders();

ProcessFile(fn);

if(bListVars) ListNames();
if(bListCounts) ListCounts();


 return 0;
}




