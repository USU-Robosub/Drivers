#include "SimpleLogger.h"
#include "Doppler.h"

#define MSG(x) std::cout << x

std::shared_ptr<Drivers::Doppler> dvl;

void cleanup(int);
void initialize();

int main() {
    initialize();

    MSG("Sending factory reset... ");
    dvl->sendBreak();
    dvl->sendReset(Drivers::Doppler::ResetType::FACTORY);
    MSG("Done!\n");

    // saveRemoteSettings(); - ignored
    // setBaudRate(BaudRate rate, Parity parity, StopBits bits); - ignored

    MSG("setFlowControl: ");
    Drivers::Doppler::FlowControl control;
    control.autoEnsembleCycling = true;
    control.autoPingCycling = true;
    control.binaryDataOutput = true;
    control.enableSerialOutput = true;
    control.enableDataRecording = false;
    dvl->setFlowControl(control);           // CF11110
    MSG("Done!\n");

    MSG("setBTPingsPerEnsemble: ");
    dvl->setBTPingsPerEnsemble(0x1);        // BP1
    MSG("Done!\n");

    MSG("setMaxTrackingDepth: ");
    dvl->setMaxTrackingDepth(1000);         // BX01000
    MSG("Done!\n");

    MSG("setHeadingAlignment: ");
    dvl->setHeadingAlignment(0.00);         // EA+00000
    MSG("Done!\n");

    MSG("setTransducerDepth: ");
    dvl->setTransducerDepth(0x0);           // ED0000
    MSG("Done!\n");

    MSG("setSalinity: ");
    dvl->setSalinity(35);                   // ES35
    MSG("Done!\n");

    MSG("setCoordTransform: ");
    Drivers::Doppler::Transformation transform;
    transform.type = Drivers::Doppler::TransformType::EARTH;
    transform.useTilts = true;
    transform.allow3Beam = true;
    transform.allowBinMapping = true;
    dvl->setCoordTransform(transform);      // EX11111
    MSG("Done!\n");

    MSG("setTimePerEnsemble: ");            // TE00:00:00.00
    dvl->setTimePerEnsemble(0x00, 0x00, 0x00, 0x00);
    MSG("Done!\n");

    MSG("setTimeBetweenPings: ");           // TP00:00.00
    dvl->setTimeBetweenPings(0x00, 0x00, 0x00);
    MSG("Done!\n");

    MSG("setMassMode: ");                   // #BK0
    dvl->setMassMode(Drivers::Doppler::MassMode::M_OFF);
    MSG("Done!\n");

    MSG("setMassLayer: ");
    dvl->setMassLayer(20, 80, 160);         // #BL20,80,160
    MSG("Done!\n");

    MSG("setTurnkeyOp: ");                  // #CT1
    dvl->setTurnkeyOp(Drivers::Doppler::TurnkeyState::T_ON);
    MSG("Done!\n");

    MSG("setHeadingVariation: ");
    dvl->setHeadingVariation(0.00);         // #EV00000
    MSG("Done!\n");

    MSG("setDataOutputFormat: ");           // #PD0
    dvl->setDataOutputFormat(Drivers::Doppler::DataOutputFormat::PD0);
    MSG("Done!\n");

    MSG("Sending custom user command (\'CR1\\r\'): ");
    dvl->sendUserCommand("CR1\r");
    std::string buffer;
    dvl->readString(buffer);
    MSG(buffer);
    MSG(" Done!\n");

    MSG("Sensor ID: ");
    MSG(dvl->WhoAmI());
    MSG("\n");

    cleanup(0);
}

void initialize() {
    std::cout << "Begin Test!\n\n";
    std::string deviceName = "/dev/ttyUSB0";

    std::shared_ptr<Drivers::SimpleLogger> logger =
        std::make_shared<Drivers::SimpleLogger>();

    MSG("Initializing Doppler Object... ");
    try {
        dvl = std::make_shared<Drivers::Doppler>(deviceName);
    } catch( ... ) {
        MSG("Failed: could not construct Doppler object!\n");
        cleanup(-1);
    }
    dvl->setLogger(logger);
    MSG("Done!\n\n");

    MSG("NOTE: All commands sent during this test are configured with ");
    MSG("factory default values. Any changes made during this test will ");
    MSG("not be saved. However they may still exist while the sensor ");
    MSG("is in use, until reset or powered down.\n\n");
}

void cleanup(int code) {
    std::cout << "\nFinished!\n";
    exit(code);
}