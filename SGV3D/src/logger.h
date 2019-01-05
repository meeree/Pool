#ifndef  __LOGGER_H__
#define  __LOGGER_H__

#include <fstream>
#include <unordered_map>

#define EXIT_ON_ERR false

struct RecordHash 
{
    size_t operator() (std::pair<std::string,int> const& key) const
        {return std::hash<std::string>{}(key.first) ^ (std::hash<int>{}(key.second) << 1);}
};

class Logger
{
private:
    std::ofstream mLogFile;
    static unsigned ms_loggingLevel;
    static unsigned msMaxDuplicates;
    static unsigned msMaxRecord;
    std::unordered_map<std::pair<std::string, int>, unsigned, RecordHash> mRecord;

    ///\brief Private constructor so only one singleton is available.
    Logger () {};

public:
    ///\brief Enum used to determine the severity of the error.
    enum eSeverity : uint8_t
    {
        INFO     = 0, 
        DEBUG    = 1, 
        WARNING  = 2, 
        ERROR    = 3, 
        LETHAL   = 4 
    };

    ///\brief Singleton pattern; return a reference to a static logger. 
    static Logger& singleton ();

    ///\brief Get reference to static log file. 
    inline std::ofstream& getFile () {return mLogFile;}

    ///\brief Initialize Logger log file. Ussually initialized in main with command-line arguments. 
    ///\param [in] logFileName name of log file
    ///\return True if log file successfully opened 
    bool init (const char* logFileName);

    ///\brief Get name of severity code. 
    static std::string codeToString (eSeverity const&);

    ///\brief Log a message in the log file. 
    ///\param [in] fileName file name 
    ///\param [in] func function that called log 
    ///\param [in] line line number
    ///\param [in] code code specifying severity 
    ///\param [in] write bool determining if we should write the output to cout
    ///\param [in] msg variadic message 
    bool log (char const* fileName, char const* func, int const& line, eSeverity const& code, bool verbose, char const* msg,...);

    ///\brief Close log file.
    ~Logger() {mLogFile.close();}
};

#define INFO_MSG(msg...)     Logger::singleton().log(__FILE__, __func__, __LINE__, Logger::eSeverity::INFO   , false, msg)
#define DEBUG_MSG(msg...)    Logger::singleton().log(__FILE__, __func__, __LINE__, Logger::eSeverity::DEBUG  , false, msg)
#define WARNING(msg...)      Logger::singleton().log(__FILE__, __func__, __LINE__, Logger::eSeverity::WARNING, false, msg)
#define ERROR(msg...)        Logger::singleton().log(__FILE__, __func__, __LINE__, Logger::eSeverity::ERROR  , true,  msg)
#define LETHAL_ERROR(msg...) Logger::singleton().log(__FILE__, __func__, __LINE__, Logger::eSeverity::LETHAL , true,  msg)

#define ASSERT(cnd) if(!(cnd)) {LETHAL_ERROR("Assertion Failed: \"%s\"",  #cnd); exit(-1);}

#endif //__LOGGER_H__
