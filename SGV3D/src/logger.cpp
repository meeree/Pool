#include "logger.h"
#include <iostream> 
#include <cstdio>
#include <stdarg.h>
#include <algorithm>
#include <time.h>

unsigned Logger::msMaxRecord = 20000;
unsigned Logger::msMaxDuplicates = msMaxRecord; //FIX THIS
unsigned Logger::ms_loggingLevel = 4;
unsigned const k_maxMsgLen = 128;

bool Logger::init (const char* logFileName) 
{
    mLogFile.open(logFileName, std::ios::out | std::ios::trunc);
    return mLogFile.is_open();
}

std::string Logger::codeToString (eSeverity const& code)
{
    switch (code)
    {
        case INFO   : return "INFO"   ;
        case DEBUG  : return "DEBUG"  ;
        case WARNING: return "WARNING";
        case ERROR  : return "ERROR"  ;
        case LETHAL : return "LETHAL" ;
    }
}

Logger& Logger::singleton () 
{
    static Logger logger;
    return logger;
}

bool Logger::log (char const* fileName, char const* func, int const& line, Logger::eSeverity const& code, bool write, char const* msg,...)
{
    if((4-code) > ms_loggingLevel)
        return true;

    std::pair<std::string,unsigned> sig{std::make_pair(std::string(fileName),line)};
    auto lookup = mRecord.find(sig);
    unsigned& cnt{mRecord[sig]};
    if (lookup == mRecord.end())
        cnt = 1;
    else 
        if (++cnt > msMaxDuplicates)
            return true;

#if 0 
    va_list msgList;
    va_start(msgList, msg);
    std::cout<<va_arg(msgList, double)<<std::endl;
    int sz{vsnprintf(nullptr, 0, msg, msgList)};
    if (sz < 0)
    {
        ERROR("Error occured interpreting format string");
        return false;
    }
    va_end(msgList);

    char msgCstring[sz];
#else 
    char msgCstring[k_maxMsgLen];
#endif 

    va_list args;
    va_start(args, msg);
    vsprintf(msgCstring, msg, args);
    va_end(args);
    mLogFile<<cnt<<"<<"+codeToString(code)+">> "<<fileName<<"["<<line<<"], "<<func<<"(): "<<msgCstring<<"\n"; 
    if(write)
    {
        std::cout<<codeToString(code)<<" (view log for info): "<<msgCstring<<"\n"; 
    }

    if (mLogFile.bad())
    { 
        std::cerr<<"Failed to write to error log file"<<std::endl;
    }

#if EXIT_ON_ERR 
    if (code > WARNING) 
    {
        msLogFl.close();
        exit(0); //Maybe add exit codes??
    }
#else
    return true;
#endif
}
