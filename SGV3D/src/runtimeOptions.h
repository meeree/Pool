#ifndef  __RUNTIME_OPTIONS_H__
#define  __RUNTIME_OPTIONS_H__

#include <string>

enum OptionsEnum 
{
    UNI_COLOR_SCHEME=0,
    UNI_CAMERA_POSITION,
    UNI_CAMERA_DIRECTION,
    UNI_MAXIMUM,
    UNI_MINIMUM,
    UNI_MODEL_MATRIX,
    UNI_VIEW_MATRIX,
    UNI_PROJECTION_MATRIX,
    OPTIONS_COUNT
};

class RuntimeOptions 
{
private:
    struct OptLookupPair 
    {
        std::string optName; 
        std::string optVal; 
    };
    OptLookupPair m_optCache[OptionsEnum::OPTIONS_COUNT];
    std::string m_configPath;

    ///\brief Parse config file into m_options 
    void ParseConfig ();

    ///\brief Setup m_options by parsing config file.
    ///       Note that this is private because RuntimeOptions is a singleton
    ///\param [in] optionsPath path to config file
    RuntimeOptions (const char* configPath="../config/config.txt"); 

public:
    ///\brief Get const singleton instance
    static RuntimeOptions& Get ();

    ///\brief Get string value defined in config
    std::string GetString (OptionsEnum const& lookup) const {return m_optCache[lookup].optVal;} 

    inline std::string GetConfigPath () const {return m_configPath;}
};

#endif //__RUNTIME_OPTIONS_H__
