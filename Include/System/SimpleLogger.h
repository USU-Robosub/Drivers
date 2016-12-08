#ifndef SIMPLE_LOGGER_H
#define SIMPLE_LOGGER_H

#include <iostream>
#include "ILogger.h"

namespace Drivers {
    class SimpleLogger : public ILogger {
    public:
        SimpleLogger(){}
        void log(std::string out) {
            std::cerr << out << "\n";
        }
        void log(std::string out, std::string info) {
            log(out);
        }
    };
}

#endif