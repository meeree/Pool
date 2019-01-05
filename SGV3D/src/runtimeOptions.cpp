#include "runtimeOptions.h"
#include "logger.h"

#include <iostream>

RuntimeOptions::RuntimeOptions (const char* configPath) : 
    m_configPath{configPath},

    //Default values for config strings
    m_optCache{{"color_scheme"     , "colorScheme"},
               {"camera_position"  , "camPos"     },
               {"camera_direction" , "camDir"     },
               {"maximum"          , "maxi"       },
               {"minimum"          , "mini"       },
               {"model_matrix"     , "model"      },
               {"view_matrix"      , "view"       },
               {"projection_matrix", "projection" }}
{
    ParseConfig();
}

void RuntimeOptions::ParseConfig ()
{
    INFO_MSG("Beginnning to parse config file");

    std::ifstream configStream(m_configPath);
    if(!configStream.is_open())
    {
        ERROR("Failed to open config file \"%s\": exiting", m_configPath.c_str());
        return;
    }

    std::string line;
    for(unsigned lineCnt = 0;; ++lineCnt)
    {
        if(getline(configStream, line).eof())
        {
            INFO_MSG("Hit end of config file");
            break;
        }

        if(!configStream.good())
        {
            ERROR("Encountered an error parsing config file on lineCnt %i: continuing", lineCnt);
            continue;
        }

        unsigned colonPos = line.find(":");
//        ASSERT(colonPos < line.size());
        std::string name = line.substr(0, colonPos).c_str();
        std::string val = line.substr(colonPos+2, line.length()-colonPos-3).c_str(); //note the +2 and not +1 to account for ". Same with end of substring

        bool hit{false};
        for(auto& optLookupPair: m_optCache)
        {
            if(optLookupPair.optName == name)
            {
                optLookupPair.optVal = val;
                hit = true;
                break;
            }
        }

        if(!hit)
            WARNING("Failed to match line to option: \"%s\"", line.c_str());
        else
            INFO_MSG("Parsed option: %s to value %s", name.c_str(), val.c_str());
    }

    INFO_MSG("Done parsing config file");
}

RuntimeOptions& RuntimeOptions::Get ()
{
    static RuntimeOptions options;
    return options;
}
