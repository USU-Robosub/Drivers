#include "SimpleLogger.h"
#include "Doppler.h"

#define MSG(x) std::cout << x

void cleanup();

int main() {
    std::cout << "Begin Test!\n\n";
    std::string deviceName = "/dev/ttyUSB0";
    
    std::shared_ptr<Drivers::SimpleLogger> logger =
        std::make_shared<Drivers::SimpleLogger>();
    std::shared_ptr<Drivers::Doppler> dvl;
    
    
    MSG("Initializing Doppler Object... ");
    try {
        dvl = std::make_shared<Drivers::Doppler>(deviceName);
    } catch( ... ) {
        MSG("Failed: could not construct Doppler object!\n");
        cleanup();
    }
    MSG("Pass!\n");
    
    
    MSG("Setting logger... ");
    dvl->setLogger(logger);
    MSG("Pass!\n");
    
    
    MSG("Sending break command...\n");
    dvl->sendBreak();
    MSG("Pass!\n\n");
    
    
    MSG("Sending factory-config command...\n");
    dvl->sendFactoryReset();
    MSG("Pass!\n\n");
    
    
    MSG("Sending flow configuration (ASCII)...\n");
    Drivers::Doppler::FlowControl ctrl; // 'CF01010'
    ctrl.autoEnsembleCycling = false;
    ctrl.autoPingCycling     = true;
    ctrl.binaryDataOutput    = false;
    ctrl.enableSerialOutput  = true;
    ctrl.enableDataRecording = false;
    dvl->setFlowControl(ctrl);
    MSG("Pass!\n\n");
    
    
    MSG("Sending data output configuration...\n");
    dvl->setDataOutputFormat(Drivers::Doppler::DataOutputFormat::PD4);
    MSG("Pass!\n\n");
    
    
    MSG("Fetching data ensemble...\n");
    std::string ensemble;
    dvl->sendPingStart(ensemble);
    MSG("Pass!\n\n");
    
    
    MSG("Sending flow configuration (Binary)...\n"); // CF01110
    ctrl.binaryDataOutput    = true;
    dvl->setFlowControl(ctrl);
    MSG("Pass!\n\n");
    
    
    MSG("Fetching data ensemble...\n");
    int length = 0;
    dvl->sendPingStart(ensemble, length);
    std::cout << "Ensemble Lenght: " << length << " bytes\n";
    MSG("Pass!\n\n");
    
    
    cleanup();
    return 0;
}

void cleanup() {
    std::cout << "\nFinished!\n";
}