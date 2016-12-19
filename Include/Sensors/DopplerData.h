#ifndef DOPPLER_DATA
#define DOPPLER_DATA

#include "Doppler.h"
#include <string>
#include <algorithm>
#include <map>

#define uChar(d,i)  (unsigned char ) d[i];                idx++
#define uShort(d,i) (unsigned short)(d[i] | (d[i+1]<<8)); idx+=2
#define sShort(d,i) (         short)(d[i] | (d[i+1]<<8)); idx+=2
#define uInt(d,i)   (unsigned int  )(d[i] | (d[i+1]<<8) | (d[i+2]<<16) | (d[i+3]<<24)); idx+=4
#define PTR(X) std::shared_ptr<X>
#define CAST(X,Y) std::dynamic_pointer_cast<X>(Y)
#define LOG(X)  std::cerr << X
#define LOGN(X) std::cerr << X << std::endl

namespace Drivers {
    typedef struct FixedLeader {
        unsigned short id;                  // 0x0000
        unsigned char firmwareVersion;
        unsigned char firmwareRevision;
        unsigned short sysConfig;
        unsigned char realSimFlag;
        unsigned char lagLength;
        unsigned char numOfBeams;
        unsigned char numOfCells;
        unsigned short pingsPerEnsemble;
        unsigned short depthCellLength;     // LSD = 1 cm
        unsigned short blankAfterTransmit;  // LSD = 1 cm
        unsigned char profilingMode;
        unsigned char lowCorrThresh;
        unsigned char numOfCodeReps;
        unsigned short errMaxVelocity;      // LSD = 1 mm/s
        unsigned char tppMinutes;
        unsigned char tppSeconds;
        unsigned char tppHundredths;
        unsigned char coorTransform;
        unsigned short headingAlignment;    // LSD = 0.01 deg
        unsigned short headingBias;         // LSD = 0.01 deg
        unsigned char sensorSource;
        unsigned char sensorAvailable;
        unsigned short bin1Dist;            // LSD = 1 cm
        unsigned short xmitPulseLength;     // LSD = 1 cm
        unsigned short wpRefLayerAverage;
        unsigned char falseTargetThresh;
        unsigned short transmitLagDistance; // LSD = 1 cm
        unsigned short sysBandwidth;
        unsigned int sysSerialNumber;
    } FixedLeader;

    typedef struct VariableLeader {
        unsigned short id;                  // 0x8000
        unsigned short ensembleTag;
        unsigned char rtcYear;
        unsigned char rtcMonth;
        unsigned char rtcDay;
        unsigned char rtcHour;
        unsigned char rtcMinute;
        unsigned char rtcSecond;
        unsigned char rtcHundredths;
        unsigned char ensembleTagMsb;       // roll-over/overflow tag
        unsigned short bitResult;
        unsigned short speedOfSound;        // LSD = 1 m/s
        unsigned short transducerDepth;     // LSD = 1 dm
        short heading;                      // LSD = 0.01 deg
        short pitch;                        // LSD = 0.01 deg
        short roll;                         // LSD = 0.01 deg
        short salinity;                     // LSD = 1 PPK
        short temperature;                  // LSD = 0.01 deg
        unsigned char mptMinutes;
        unsigned char mptSeconds;
        unsigned char mptHundredths;
        unsigned char headingStdDev;        // LSD = 1 deg
        unsigned char pitchStdDev;          // LSD = 0.1 deg
        unsigned char rollStdDev;           // LSD = 0.1 deg
        unsigned char adc0;
        unsigned char adc1;
        unsigned char adc2;
        unsigned char adc3;
        unsigned char adc4;
        unsigned char adc5;
        unsigned char adc6;
        unsigned char adc7;
        unsigned int errStatus;
        unsigned int pressure;              // LSD = 1 dp
        unsigned int pressureVar;           // LSD = 1 dp
    } VariableLeader;

    typedef struct DepthCell {
        short velocity[4];
        short corrMagnitude[4];
        short echoIntensity[4];
        short percentGood[4];
    } CorrMagnitude;

    typedef struct BottomTrack {
        unsigned short id;                  // 0x0600
        unsigned short pingsPerEnsemble;
        unsigned short delayBeforeReAquire;
        unsigned char corrMagMin;
        unsigned char evalAmpMin;
        unsigned char percentGoodMin;
        unsigned char mode;
        unsigned short errVelMax;
        unsigned short range[4];
        unsigned short velocity[4];
        unsigned char correlation[4];
        unsigned char evalAmp[4];
        unsigned char percentGood[4];
        unsigned short refLayerMin;
        unsigned short refLayerNear;
        unsigned short refLayerFar;
        unsigned short refLayerVel[4];
        unsigned char refCorrelation[4];
        unsigned char refInt[4];
        unsigned char refPercentGood[4];
        unsigned short maxDepth;
        unsigned char rssiAmp[4];
        unsigned char gain;
        unsigned char rangeMSB[4];
    } BottomTrack;

    class IEnsemble {
    public:
        virtual int getType() = 0;
    };

    class DataStream0 : public IEnsemble {
    public:
        int getType() {
            return 0;
        }

        unsigned char headerId;
        unsigned char id;
        unsigned short length;
        unsigned char typeCount;

        FixedLeader fixedLeader;
        VariableLeader variableLeader;
        PTR(DepthCell) depthCells;
        PTR(BottomTrack) bottomTrack;

        unsigned short checksum;
        std::string raw;
    };

    class DataStream4 : public IEnsemble {
    public:
        int getType() {
            return 4;
        }

        unsigned char id;
        unsigned char structure;
        unsigned short length;
        unsigned char sysConfig;

        short xVelBtm; // east
        short yVelBtm; // north
        short zVelBtm; // up
        short eVelBtm; // LSD = 1 mm/s

        unsigned short bm1RngBtm;
        unsigned short bm2RngBtm;
        unsigned short bm3RngBtm;
        unsigned short bm4RngBtm; // LSD = 1 cm

        unsigned char btmStatus;

        short xVelRef; // east
        short yVelRef; // north
        short zVelRef; // up
        short eVelRef; // LSD = 1 mm/s

        unsigned short refLayerStart;
        unsigned short refLayerEnd; // LSD = 1 dm
        unsigned char refLayerStatus;

        unsigned char tofpHour;
        unsigned char tofpMinute;
        unsigned char tofpSecond;
        unsigned char tofpHundredths;

        unsigned short bitResults;
        unsigned short speedOfSound; // LSD = 1 m/s
        float temperature; // LSD = 0.01 C
        unsigned short checksum;
        std::string raw;
    };

    class DataStream5 : public IEnsemble {
    public:
        int getType() {
            return 5;
        }

        PTR(DataStream4) velocity;

        unsigned char salinity; // LSD = 1 ppt
        unsigned short depth; // LSD = 1 dm
        float pitch; // LSD = 0.01 deg
        float roll; // LSD = 0.01 deg
        float heading; // LSD = 0.01 deg

        unsigned int eastBtmDistGood;
        unsigned int northBtmDistGood;
        unsigned int upBtmDistGood;
        unsigned int errBtmDistGood; // LSD = 1 dm

        unsigned int eastRefDistGood;
        unsigned int northRefDistGood;
        unsigned int upRefDistGood;
        unsigned int errRefDistGood; // LSD = 1 dm
    };

    class DataStream6 : public IEnsemble {
    public:
        int getType() {
            return 6;
        }
        std::map<std::string, float> fmap;
        std::map<std::string, int> imap;
        std::map<std::string, char> cmap;
        std::string raw;
    };

    class DopplerData {
    public:
        static unsigned char hexToByte(char HEX);
        static unsigned char hexToByte(char MSB, char LSB);
        static std::string hexToBinary(std::string stream);

        static PTR(IEnsemble) composeEnsemble(Doppler & dvl);
        static void printEnsemble(PTR(IEnsemble) package);

    private:
        static void scrubPD6(std::string& stream);

        static FixedLeader      _cds0_getFixedLeader(int offset, std::string data);
        static VariableLeader   _cds0_getVariableLeader(int offset, std::string data);
        static PTR(BottomTrack) _cds0_getBottomTrack(int offset, std::string data);
        static int              _cds6_getTag(char* tag, std::stringstream & data);
        static int              _cds6_getValue(char  & value, std::stringstream & data);

        static PTR(DataStream0) composeDataStream0(std::string data);
        static PTR(DataStream4) composeDataStream4(std::string data);
        static PTR(DataStream5) composeDataStream5(std::string data);
        static PTR(DataStream6) composeDataStream6(std::string data);

        static void printDataStream0(PTR(DataStream0) data);
        static void printDataStream4(PTR(DataStream4) data);
        static void printDataStream5(PTR(DataStream5) data);
        static void printDataStream6(PTR(DataStream6) data);
    };
}

#endif
