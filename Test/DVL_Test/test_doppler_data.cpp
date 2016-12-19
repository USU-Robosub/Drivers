#include "SimpleLogger.h"
#include "DopplerData.h"

#define MSG(x) std::cout << x

void cleanup();

int main() {
    MSG("Begin Test!\n\n");

    std::string deviceName = "/dev/ttyUSB0";
    std::shared_ptr<Drivers::SimpleLogger> logger =
        std::make_shared<Drivers::SimpleLogger>();
    std::shared_ptr<Drivers::Doppler> dvl;

    MSG("Initializing Doppler Object... ");
    try {
        dvl = std::make_shared<Drivers::Doppler>(deviceName);
        dvl->setLogger(logger);
    } catch( ... ) {
        MSG("Failed: could not construct Doppler object!\n");
        cleanup();
    }
    MSG("Done!\n");

    MSG("Configuring Doppler Sensor... ");
    dvl->sendBreak();
    dvl->sendReset(Drivers::Doppler::ResetType::FACTORY);
    Drivers::Doppler::FlowControl ctrl; // 'CF01010'
    ctrl.autoEnsembleCycling = false;
    ctrl.autoPingCycling     = true;
    ctrl.binaryDataOutput    = false;
    ctrl.enableSerialOutput  = true;
    ctrl.enableDataRecording = false;
    dvl->setFlowControl(ctrl);
    MSG("Done!\n");

    std::shared_ptr<Drivers::IEnsemble> package;

    MSG("Testing PD0 Data Ensemble... ");
    dvl->setDataOutputFormat(Drivers::Doppler::DataOutputFormat::PD0);
    package = Drivers::DopplerData::composeEnsemble(*dvl);
    Drivers::DopplerData::printEnsemble(package);
    MSG("Done!\n\n");

    MSG("Testing PD4 Data Ensemble... ");
    dvl->setDataOutputFormat(Drivers::Doppler::DataOutputFormat::PD4);
    package = Drivers::DopplerData::composeEnsemble(*dvl);
    Drivers::DopplerData::printEnsemble(package);
    MSG("Done!\n\n");

    MSG("Testing PD5 Data Ensemble... ");
    dvl->setDataOutputFormat(Drivers::Doppler::DataOutputFormat::PD5);
    package = Drivers::DopplerData::composeEnsemble(*dvl);
    Drivers::DopplerData::printEnsemble(package);
    MSG("Done!\n\n");

    MSG("Testing PD6 Data Ensemble... ");
    dvl->setDataOutputFormat(Drivers::Doppler::DataOutputFormat::PD6);
    package = Drivers::DopplerData::composeEnsemble(*dvl);
    Drivers::DopplerData::printEnsemble(package);
    MSG("Done!\n\n");

    // test binary
    MSG("Testing Binary PD4 Data Ensemble... ");
    ctrl.binaryDataOutput = true;
    dvl->setFlowControl(ctrl);
    dvl->setDataOutputFormat(Drivers::Doppler::DataOutputFormat::PD4);
    package = Drivers::DopplerData::composeEnsemble(*dvl);
    Drivers::DopplerData::printEnsemble(package);
    MSG("Done!\n");

    cleanup();
}

void cleanup() {
    MSG("\nFinished!\n");
    exit(0);
}