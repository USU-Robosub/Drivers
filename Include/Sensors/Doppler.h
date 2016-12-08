#ifndef DOPPLER_H
#define DOPPLER_H

#include "DopplerSerial.h"
#include "IDriver.h"
#include "ILogger.h"

namespace Drivers {
    class Doppler : public IDriver {
    public:
        // Creates a Doppler object from the device file name
        Doppler(std::string deviceName);
        // Creates a Doppler object from an existing ISerial object.
        Doppler(std::shared_ptr<ISerial> serial);
        // basic internal initialization
        void _initVars();
        // because there is no correct eof queue given, we need to know exactly
        // how many bytes to read from the stream with respect to the ensemble
        // type
        void _readBinaryEnsemble(std::string& ensemble, int& length);
        
        
        /* ------------------------------------------------------- *\
        |               CONFIGURATION & DATA STRUCTS                |
        \* ------------------------------------------------------- */
        
        /* WARNING: AutoEnsembleCycling is not supported. Leaving this enabled
         * may cause your program to hault.
         *
         * WARNING: Disabling SerialOutput is not supported. Disabling it may
         * result in a loss of data.
         */
        typedef struct FlowControl {
            bool autoEnsembleCycling;   // default: enabled
            bool autoPingCycling;       // default: enabled
            bool binaryDataOutput;      // default: enabled
            bool enableSerialOutput;    // default: enabled
            bool enableDataRecording;   // default: disabled
        } FlowControl;
        
        enum DataOutputFormat {
            PD0, PD4, PD5, PD6
        };
        
        
        /* ------------------------------------------------------- *\
        |                      DEVICE COMMANDS                      |
        \* ------------------------------------------------------- */
        
        void sendBreak();
        void sendUserReset();
        void sendFactoryReset();
        void sendPingStart(std::string& ensemble);
        void sendPingStart(std::string& ensemble, int& length);
        void setFlowControl(FlowControl config);
        void setDataOutputFormat(DataOutputFormat format);
        
        
        /* ------------------------------------------------------- *\
        |                      OTHER FUNCTIONS                      |
        \* ------------------------------------------------------- */
        
        void setLogger(std::shared_ptr<ILogger> logger);
        int WhoAmI();
        
    private:
        std::shared_ptr<ISerial> _dvl_;
        std::shared_ptr<ILogger> _log_;
        FlowControl _fc_;
        DataOutputFormat _dof_;
    };
}

#endif