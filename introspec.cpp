#include <predef.h>
#include <init.h>
#include <basictypes.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iosys.h>
#include <nettypes.h>
#include <ftpd.h>
#include <nbrtos.h>
#include <config_obj.h>
#include <iointernal.h>
#include "introspec.h"

LOGFILEINFO;

#define START_OF_KEYS (100)


#define KEY_START (250)
#define KEY_ESCAPE (249)


#define ELEMENT_DESCRIBE (99)
#define KEY_DESCRIBE     (98)
#define KEY_EVENT        (97)
#define KEY_MESSAGE      (96)
bool IntrospecObject::bShowMe;


volatile bool bLog;					  



#define LOG_SIZE (1024*1024*16)




uint8_t LogData[LOG_SIZE];


uint32_t LogGet;
uint32_t LogPut;
bool bOverFlow;


OS_CRIT LogCrit;

struct IntroSpecDescription
{
IntroSpecDescription * pNext;
const char * m_name;
int KeyValue;
bool bStructureLogged;
};



static IntroSpecDescription *pDescHead;
static int StaticKeyValue;




IntroSpecDescription * BuildOrFindDescription(const char * name)
{
IntroSpecDescription * pItem=pDescHead;

while(pItem)
{
 if(strcmp(pItem->m_name,name)==0) return pItem;
 pItem=pItem->pNext;
}

pItem=new IntroSpecDescription;
pItem->KeyValue=(++StaticKeyValue+START_OF_KEYS);
pItem->m_name=name;
pItem->bStructureLogged=false;

USER_ENTER_CRITICAL();
pItem->pNext=pDescHead;
pDescHead=pItem;
USER_EXIT_CRITICAL();

return pItem;

}





IntrospecObject::IntrospecObject(const char * name)
{
m_name=name;
pDesc=NULL;
}



void LogEscapedByte(uint8_t v)
{
LogData[LogPut++]=v;
if(LogPut>=LOG_SIZE) LogPut=0;

if(LogPut==LogGet)
{
   LogGet++;
   if(LogGet>=LOG_SIZE) LogGet=0;
   bOverFlow=true;
}
}


void LogRawByte(uint8_t v)
{
   if(v==KEY_START)
   {
       LogEscapedByte(KEY_ESCAPE);
       LogEscapedByte(0);

   }
   else
   if(v==KEY_ESCAPE)
   {
       LogEscapedByte(KEY_ESCAPE);
       LogEscapedByte(1);
   }
   else
   {
     LogEscapedByte(v);
   }


}



void LogRaw32(uint32_t dat)
{
     if(dat<128)
         {
          LogRawByte(dat);
         }
     else
     if(dat< 16384)
     {
         LogRawByte((dat>>7)|0x80);
         LogRawByte(dat & 0x7F);
     }
     else
     if(dat< 2097152)
      {
         LogRawByte((dat>>14)|0x80);
         LogRawByte(((dat>>7)|0x80)& 0xFF);
         LogRawByte(dat & 0x7F);
     }
     else
     if(dat< 268435456)
     {
        LogRawByte((dat>>21)|0x80);
        LogRawByte(((dat>>14)|0x80)& 0xFF);
        LogRawByte(((dat>>7)|0x80)& 0xFF);
        LogRawByte(dat & 0x7F);
    }
    else
    {
     LogRawByte((dat>>28)|0x80);
     LogRawByte(((dat>>21)|0x80)& 0xFF);
     LogRawByte(((dat>>14)|0x80)& 0xFF);
     LogRawByte(((dat>>7)|0x80)& 0xFF);
     LogRawByte(dat & 0x7F);
    }
}



void LogRawData(uint32_t * pStart, uint32_t * pEnd)
{
    while(pStart<pEnd) LogRaw32(*pStart++);
}





void IntrospecObject::Element(void * p, size_t siz,const char * label, char c)
{
static uint32_t * OffsetPos;
if((label==NULL) && (c==0))
{
   OffsetPos=(uint32_t *)p;
   OffsetPos++;
   return;
}

LogRawByte(ELEMENT_DESCRIBE);
uint8_t * pStart=(uint8_t *)OffsetPos;
uint8_t * pMe=(uint8_t *)p;
uint32_t off;
if(pMe>pStart)
    {
     off=(pMe-pStart);
    }
else
    {
     off=0;
    }

    const char * cp=label;
    while(*cp)
LogRawByte(*cp++);
LogRawByte(0);
LogRawByte(c);
LogRaw32(off);
LogRaw32(siz);
}



void IntrospecObject::LogStructure()
{
    uint32_t * pstart;
    uint32_t * pend;
    GetExtent(pstart,pend);
    pstart++;

    LogEscapedByte(KEY_START);
    LogRawByte(KEY_DESCRIBE);
    LogRaw32(pDesc->KeyValue); ///Key
    LogRaw32(pend-pstart);//Len
    const char * cp=m_name;
    while(*cp)
        LogRawByte(*cp++);
    LogRawByte(0);
    bShowMe=true;
    BuildOne();
    bShowMe=false;

}


void IntrospecObject::Log(bool bForceLog)
{
if((!bLog) && (!bForceLog))return;
if(!pDesc)
{
pDesc=BuildOrFindDescription(m_name);
}
uint32_t * pstart;
uint32_t * pend;
GetExtent(pstart,pend);
pstart++;


if(!pDesc->bStructureLogged)
{
 OSCriticalSectionObj oc(LogCrit);
 LogStructure();
 pDesc->bStructureLogged=true;
}

{
OSCriticalSectionObj oc(LogCrit);
 LogEscapedByte(KEY_START);
 LogRaw32  (pDesc->KeyValue);
 LogRawData(pstart,pend);
}
}





void LogMessage(const char * cp,bool bForceLog)
{
if((!bLog) && (!bForceLog))return;
OSCriticalSectionObj oc(LogCrit);
LogEscapedByte(KEY_START);
LogRawByte(KEY_MESSAGE);
while(*cp)
        LogRawByte(*cp++);
    LogRawByte(0);

}
void LogEvent(bool bForceLog)
{
if((!bLog) && (!bForceLog))return;
OSCriticalSectionObj oc(LogCrit);
LogEscapedByte(KEY_START);
LogRawByte(KEY_EVENT);
}


void WriteLogToFd(int fd)
{
OSCriticalSectionObj oc(LogCrit);
 int g=LogGet;
 int p=LogPut;

 IntroSpecDescription * pItem=pDescHead;
 while(pItem)
 {
  pItem->bStructureLogged=false;
  pItem=pItem->pNext;
 }

int rv=0;

 if(g<p)
 {
    rv=writeall(fd,(const char *)LogData+g,(p-g));
	printf("RV=%d of %d\n",rv,(p-g));
 }
 else
 {
    rv=writeall(fd,(const char *)LogData+g,(LOG_SIZE-g));
	printf("RV1=%d of%d \r\n",rv,(LOG_SIZE-g));
    rv+=writeall(fd,(const char *)LogData,p);
	printf("RV2=%d of %d\r\n",rv,p);
 }
}

void ClearLog()
{
USER_ENTER_CRITICAL();
LogGet=LogPut=0;
USER_EXIT_CRITICAL();
}


int GetLogSize()
{
 OSCriticalSectionObj oc(LogCrit);
 int g=LogGet;
 int p=LogPut;
 if(p>g) return p-g;
 if(p==g) return 0;
return (LOG_SIZE-g)+p;
}

int GetLogPercent()
{
 return (GetLogSize()*100/LOG_SIZE);
}

#define READ_FILENAME "Log.bin"
#define CLEAR_FILENAME "clear"


void InitLogFtp(int PRIO)
{
    FTPDStart( 21, PRIO);
}


/*--------------------------------------------------------------------
 *  This function gets called by the FTP Server when a FTP client
 *  sends a file. File must be named WriteFile.txt.
 *--------------------------------------------------------------------*/
int FTPD_GetFileFromClient(const char *full_directory, const char *file_name, void *pSession, int fd)
{
    return FTPD_FAIL;
}



/*-------------------------------------------------------------------------
 *  This function gets called by the FTP Server to determine if it is ok to
 *  create a file on the system. In this case is will occur when a FTP
 *  client sends a file. File must be named WriteFile.txt.
 *-------------------------------------------------------------------------*/
int FTPD_AbleToCreateFile(const char *full_directory, const char *file_name, void *pSession)
{
    return FTPD_FAIL;
}



/*-------------------------------------------------------------------
 * The parameters passed to you in this function show the entered
 * user name, password and IP address they came from. You can
 * modify this function any way you wish for authentication.
 *
 * Return Values:
 *   0   = Authentication failed
 *   > 0 = Authentication passed
 * -----------------------------------------------------------------*/
void * FTPDSessionStart( const char *user, const char *passwd, const IPADDR4 hi_ip )
{
    return ( void * ) 1; //  Return a non zero value
}




/****************FTPD Functions that are not supported/used in this trivial case *******************/
void FTPDSessionEnd( void *pSession )
{
    /* Do nothing */
}

int FTPD_ListSubDirectories(const char *current_directory, void *pSession, FTPDCallBackReportFunct *pFunc, int handle)
{
    /* No directories to list */
    return FTPD_OK;
}

int FTPD_DirectoryExists( const char *full_directory, void *pSession )
{
    return FTPD_FAIL;
}


int FTPD_CreateSubDirectory(const char *current_directory, const char *new_dir, void *pSession)
{
    return FTPD_FAIL;
}

int FTPD_DeleteSubDirectory(const char *current_directory, const char *sub_dir, void *pSession)
{
    return FTPD_FAIL;
}

int FTPD_DeleteFile( const char *current_directory, const char *file_name, void *pSession )
{
    if ( strcmp( file_name, READ_FILENAME ) == 0 )
	{
		ClearLog();
		return FTPD_OK;

	}
    return FTPD_FAIL;
}

int FTPD_Rename( const char *current_directory,const char *cur_file_name,const char *new_file_name,void *pSession )
{
    return FTPD_FAIL;
}

int FTPD_GetFileSize( const char *full_directory,  const char *file_name)
{
    return FTPD_FILE_SIZE_NOSUCH_FILE;
}


void getdirstring( const char *FileName, long FileSize, char *DirStr )
{
    char tmp[80];

    DirStr[0] = '-';  // '-' for file, 'd' for directory
    DirStr[1] = 0;

    // permissions, hard link, user, group
    strcat( DirStr, "rw-rw-rw- 1 user group " );

    sniprintf( tmp, 80, "%9ld ", FileSize );
    strcat( DirStr, tmp );

    strcat( DirStr, "JAN 01 2000 " );

    strcat( DirStr, FileName );
}



/*-----------------------------------------------------------------------
 *  This function is called by the FTP Server in response to a FTP Client's
 *  request to list the files (e.g. the "ls" command)
 *------------------------------------------------------------------------*/
int FTPD_ListFile(const char *current_directory, void *pSession, FTPDCallBackReportFunct *pFunc, int handle)
{
    char DirStr[256];

    // Only one file exists ReadFile.txt
    getdirstring( READ_FILENAME, GetLogSize(), &DirStr[0] );
    pFunc( handle, DirStr );
    getdirstring( CLEAR_FILENAME, 10, &DirStr[0] );
    pFunc( handle, DirStr );
    return FTPD_OK;
}


/*-----------------------------------------------------------------------
 *  This function is called by the FTP Server in response to a FTP
 *  Client's request to receive a file. In this example, only ReadFile.txt
 *  is available for download, and it's contents are hard coded to the
 *  string in the writestring() function.
 *-----------------------------------------------------------------------*/
int FTPD_SendFileToClient(const char *full_directory, const char *file_name, void *pSession, int fd)
{
    // Only one file exists
    if ( strcmp( file_name, CLEAR_FILENAME ) == 0 )
    {
		writestring(fd,"Clear");
		ClearLog();
        // Now send the "file", which is just one line of text in this example
    }
    else
    {
        WriteLogToFd(fd);
    }
	return FTPD_OK;
}





/*--------------------------------------------------------------------------
 *  This function is called by the FTP Server to determine if a file exists.
 *-------------------------------------------------------------------------*/
int FTPD_FileExists( const char *full_directory, const char *file_name, void *pSession )
{
        return FTPD_OK;
}




static FileLog * pHead;


FileLog::FileLog(const char * Msg)
{
 LogMsg=Msg;
 pNext=pHead;
 pHead=this;
}

void LogFileVersions()
{
 FileLog * pl=pHead;
 USER_ENTER_CRITICAL();
 while(pl)
 {
  LogMessage(pl->LogMsg,true);
  pl=pl->pNext;
 }
 USER_EXIT_CRITICAL();
}



	  int LogEXRead( int fd, char *buf, int nbytes)  {return 0;}
	  int LogEXClose( int fd)  {return 0;}
	  int LogEXWrite( int fd, const char *buf, int nbytes )
	  {
		  for(int i=0; i<nbytes; i++) LogRawByte(buf[i]);
		return nbytes;
	  }


void LogAppRecords()
{
IoExpandStruct ioe;
ioe.read=LogEXRead;
ioe.close=LogEXClose;
ioe.write=LogEXWrite;

OSCriticalSectionObj oc(LogCrit);
LogEscapedByte(KEY_START);
LogRawByte(KEY_MESSAGE);
int LogFd=GetExtraFD( 0,&ioe);
appdata.RenderToFd(LogFd,true);
LogRawByte(0);
FreeExtraFd(LogFd);
}

