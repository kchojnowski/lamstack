#ifndef EVENTHANDLER_H
#define EVENTHANDLER_H

#include <stdint.h>
#include "RadioRxPacket.h"
#include "RadioConfig.h"

class EventHandler
{
public:
    EventHandler();
    void init(void);
    void packetReceived(RadioRxPacket& packet);
    void configChanged(RadioConfig& config);
    void transmissionStarted(void);
    void transmissionFinished(void);
};

#endif
