#ifndef ILOGGER_H
#define ILOGGER_H

#include <string>

namespace Drivers {
    class ILogger {
    public:
        virtual void log(std::string out) = 0;
        virtual void log(std::string out, std::string info) = 0;
    };
}

#endif