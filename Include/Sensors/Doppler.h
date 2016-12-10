#ifndef DOPPLER_H
#define DOPPLER_H

#include "DopplerSerial.h"
#include "IDriver.h"
#include "ILogger.h"

namespace Drivers {
    class Doppler : public IDriver {
        // basic internal initialization
        void _initVars();
        // because there is no correct eof queue given, we need to know exactly
        // how many bytes to read from the stream with respect to the ensemble
        // type
        void _readBinaryEnsemble(std::string& ensemble, int& length);
        char _numToChar(unsigned int n);

    public:
        Doppler(std::string deviceName);
        Doppler(std::shared_ptr<ISerial> serial);


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
            bool autoEnsembleCycling;                   // default: enabled
            bool autoPingCycling;                       // default: enabled
            bool binaryDataOutput;                      // default: enabled
            bool enableSerialOutput;                    // default: enabled
            bool enableDataRecording;                   // default: disabled
        } FlowControl;

        enum MassMode { M_OFF, M_WB, M_LOSTB, M_W };    // default: M_OFF
        enum ResetType { USER, FACTORY };
        enum TurnkeyState { T_OFF, T_ON };              // default: T_ON
        enum DataOutputFormat { PD0, PD4, PD5, PD6 };   // default: PD0

        // Coordinate Transformation
        enum TransformType { NONE, INSTRUMENT, SHIP, EARTH};
        typedef struct Transformation {
            TransformType type;                         // default: EARTH
            bool useTilts;                              // default: true
            bool allow3Beam;                            // default: true
            bool allowBinMapping;                       // default: true
        } Transformation;

        // baud rate controls
        enum BaudRate {
            B300,   B1200,  B2400,                      // default: B9600
            B4800,  B9600,  B19200,
            B38400, B57600, B115200
        };
        enum Parity { NO_P, EVEN, ODD, LOW, HIGH };     // default: NO_P
        enum StopBits { BIT1, BIT2 };                   // default: BIT1


        /* ------------------------------------------------------- *\
        |                      DEVICE COMMANDS                      |
        \* ------------------------------------------------------- */

        // Rank 1 Commands
        void sendReset(ResetType type);
        void saveRemoteSettings();
        void sendPingStart(std::string& ensemble);
        void sendPingStart(std::string& ensemble, int& length);

        // Rank 2 Commands
        void sendBreak();
        void setBaudRate(BaudRate rate, Parity parity, StopBits bits);
        void setFlowControl(FlowControl config);
        void setBTPingsPerEnsemble(unsigned int pings);
        void setMaxTrackingDepth(unsigned int decimeters);
        void setHeadingAlignment(float degrees);
        void setTransducerDepth(unsigned int decimeters);
        void setSalinity(unsigned char partsPerK);
        void setCoordTransform(Transformation transform);
        void setTimePerEnsemble(unsigned char hours, unsigned char minutes,
            unsigned char seconds, unsigned char hundredths);
        void setTimeBetweenPings(unsigned char minutes, unsigned char seconds,
            unsigned char hundredths);

        // Rank 3 Commands
        void setMassMode(MassMode mode);
        void setMassLayer(unsigned int minSize, unsigned int nearBoundary,
            unsigned int farBoundary);
        void setTurnkeyOp(TurnkeyState state);
        void setHeadingVariation(float degrees);
        void setDataOutputFormat(DataOutputFormat format);

        // Other
        void sendUserCommand(std::string command);
        void readString(std::string& data);


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
        MassMode _mm_;
        TurnkeyState _ts_;
        Transformation _transform_;
    };
}

#endif