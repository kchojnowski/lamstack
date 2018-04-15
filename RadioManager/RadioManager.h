#ifndef RADIOMANAGER_H
#define RADIOMANAGER_H

#include <stdint.h>

#include "RadioConfig.h"
#include "RadioTxPacket.h"
#include "RadioRxPacket.h"
#include "Sx127x.h"
#include "Crc.h"

#define PACKET_ADDR_BROADCAST    255
#define PACKET_HEADER_LEN        9
#define PACKET_MAX_DATA_LEN      64
#define PACKET_MAX_LIFE          16
#define PACKET_RX_BUFFER_LEN     512
#define PACKET_TX_BUFFER_LEN     384
#define PACKET_TX_ACK_BUFFER_LEN 128
#define PACKET_TX_CALLBACKS_LEN PACKET_TX_BUFFER_LEN/PACKET_HEADER_LEN
#define PACKET_TX_ACK_CALLBACKS_LEN PACKET_TX_ACK_BUFFER_LEN/PACKET_HEADER_LEN

#define PACKET_FIELD_DST      0
#define PACKET_FIELD_SRC      1
#define PACKET_FIELD_DSTRT    2
#define PACKET_FIELD_SRCRT    3
#define PACKET_FIELD_ID       4
#define PACKET_FIELD_TYPE     5
#define PACKET_FIELD_LIFE     6
#define PACKET_FIELD_DATALEN  7
#define PACKET_FIELD_CRC      8
#define PACKET_FIELD_DATA     9

#define PACKET_TYPE           0x07
#define PACKET_TYPE_DATA      0x05
#define PACKET_TYPE_STAT      0x07

#define PACKET_FLAGS            0xF8
#define PACKET_FLAGS_SETROUTE   0x08
#define PACKET_FLAGS_ACK_REQ    0x10
#define PACKET_FLAGS_ACK_RSP    0x20
#define PACKET_FLAGS_NOROUTE    0x40

#define PACKET_STAT_ACTIVE          0x01
#define PACKET_STAT_GET_RTABLE      0x03
#define PACKET_STAT_SET_RTABLE      0x04
#define PACKET_STAT_GET_RSSI        0x05
#define PACKET_STAT_GET_SNR         0x06
#define PACKET_STAT_GET_DROP        0x08
#define PACKET_STAT_SET_CONFIG      0x09
#define PACKET_STAT_SET_CONFIG_ACK  0x0A

#define ACK_TIMEOUT_MS 30000
#define ACK_MAX_TRIES 5

#define LAST_PACKET_TIME_DISPLAY_PERIOD_MS 1000
#define LAST_PACKET_MAX_TIME_S 500

#define CONFIG_CHANGE_TIMEOUT_MS 30000
#define FREEZE_TIMEOUT_MS 8000

#define SEND_LOOP_TIMEOUT 100000


template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
class RadioManager
{
public:
    RadioManager(SpiCtrlTempl& spi, EventHndlTempl& eventHndl, LogTempl& log);
    void setConfig(RadioConfig config);
    RadioConfig getConfig(void);
    void init();
    uint8_t sendPacket(RadioTxPacket packet);
    void process(uint32_t elapsedMs);

private:
    RadioConfig radioConfig;
    RadioConfig radioConfigOld;
    Sx127x<SpiCtrlTempl, LogTempl> sx;
    EventHndlTempl& eventHndl;
    LogTempl& log;

    bool changeConfig;
    bool waitingConfigAck;

    uint32_t packetTimeTimer;
    uint32_t ackTimeTimer;
    uint32_t ackTimeout;
    uint32_t ackSendTries;
    uint32_t configChangeTimer;
    uint32_t freezeTimer;
    uint32_t freezeTimeout;
    bool startFreeze;
    bool freeze;

    uint16_t txBufferPointer;
    uint16_t txAckBufferPointer;
    uint8_t txBuffer[PACKET_TX_BUFFER_LEN];
    uint8_t txAckBuffer[PACKET_TX_ACK_BUFFER_LEN];
    uint16_t txCallbacksPointer;
    uint16_t txAckCallbacksPointer;
    TxFinishedCallback txCallbacks[PACKET_TX_CALLBACKS_LEN];
    TxFinishedCallback txAckCallbacks[PACKET_TX_ACK_CALLBACKS_LEN];
    uint8_t rxBuffer[PACKET_RX_BUFFER_LEN];
    uint8_t packetBuffer[PACKET_MAX_DATA_LEN];
    uint8_t packetId;

    uint8_t rssi[MAX_NODES];
    int8_t snr[MAX_NODES];
    uint8_t ackWaitId;

    uint16_t lastPacketTime[MAX_NODES];
    uint32_t packetsTransmitted;
    uint32_t packetsReceived;
    uint32_t packetsRtTransmitted;
    uint32_t packetsRtReceived;
    uint32_t packetsDropped;

    void addPacketToQueue(RadioTxPacket packet);

    void sendFirstPacket(uint8_t* buffer, uint16_t* bufferPointer);
    void dropFirstPacket(uint8_t* buffer, uint16_t* bufferPointer, TxResult result);

    void setRoute(uint8_t dst, uint8_t dstrt);
    uint8_t getRoute(uint8_t dst);

    void setRssi(uint8_t addr, uint8_t rssi);
    uint8_t getRssi(uint8_t addr);

    void setSnr(uint8_t addr, int8_t snr);
    int8_t getSnr(uint8_t addr);

    uint8_t getNextPacketId(void);

    void setLastPacketTime(uint8_t addr, uint16_t time);
    uint16_t getLastPacketTime(uint8_t addr);

    uint8_t handleMessage(uint8_t* msg);
    void handleDataMessage(uint8_t* msg);
    void handleStatMessage(uint8_t* msg);

    void logRoutingTable(void);
    void logRssi(void);
    void logCurrentRssiAndStatus(uint8_t currentRssi, uint8_t status);
    void logPacketTime(void);
    void logPacketStats(void);
    void updatePacketTime(uint32_t elapsedMs);
};

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::RadioManager(SpiCtrlTempl& spi, EventHndlTempl &eventHndl, LogTempl& log)
    : sx(spi, log), eventHndl(eventHndl), log(log)
{
    this->packetsReceived = 0;
    this->packetsTransmitted = 0;
    this->packetsRtReceived = 0;
    this->packetsRtTransmitted = 0;
    this->packetsDropped = 0;
    this->ackWaitId = 0;
    this->changeConfig = false;
    this->waitingConfigAck = false;
    this->startFreeze = false;
    this->freeze = false;
    this->freezeTimer = 0;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::setConfig(RadioConfig config)
{
    this->radioConfig = config;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
RadioConfig RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::getConfig(void)
{
    return this->radioConfig;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::init()
{
    for(int i = 0; i < MAX_NODES; i++)
    {
        this->rssi[i] = 0;
        this->snr[i] = 0;
        this->lastPacketTime[i] = 0;
    }

    for(int i = 0; i < PACKET_TX_CALLBACKS_LEN; i++)
        txCallbacks[i] = NULL;
    for(int i = 0; i < PACKET_TX_ACK_CALLBACKS_LEN; i++)
        txAckCallbacks[i] = NULL;

    this->txAckBufferPointer = 0;
    this->txBufferPointer = 0;
    this->txCallbacksPointer = 0;
    this->txAckCallbacksPointer = 0;
    this->packetTimeTimer = 0;
    this->packetId = 0;
    this->ackWaitId = 0;
    this->sx.init(this->radioConfig.carrierFrequency, this->radioConfig.bandwidth, this->radioConfig.spreadingFactor,
                  this->radioConfig.codingRate, this->radioConfig.outputPower);
    this->sx.startRx();

    this->log.print("My address ");
    this->log.print((int)this->radioConfig.address);
    this->log.printWithNewline("");

    this->logRoutingTable();
    this->sx.printRegisters();
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
uint8_t RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::sendPacket(RadioTxPacket packet)
{
    if(packet.id == 0)
        packet.id = this->getNextPacketId();
    if(packet.src == 0)
        packet.src = this->radioConfig.address;
    this->addPacketToQueue(packet);
    return packet.id;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::addPacketToQueue(RadioTxPacket packet)
{
    uint16_t* bufferPointer;
    uint8_t* buffer;
    uint16_t* callbacksPointer;
    TxFinishedCallback* callbacks;

    if(packet.type & PACKET_FLAGS_ACK_RSP) {
        bufferPointer = &this->txAckBufferPointer;
        buffer = this->txAckBuffer;
        callbacksPointer = &this->txAckCallbacksPointer;
        callbacks = this->txAckCallbacks;
    } else {
        bufferPointer = &this->txBufferPointer;
        buffer = this->txBuffer;
        callbacksPointer = &this->txCallbacksPointer;
        callbacks = this->txCallbacks;
    }

    if((*bufferPointer + packet.len + PACKET_HEADER_LEN) >= PACKET_TX_BUFFER_LEN)
    {
        this->log.printWithNewline("RadioManager: TX BUFFER FULL");
        return;
    }

    if(packet.dst == PACKET_ADDR_BROADCAST) {
        buffer[*bufferPointer + PACKET_FIELD_DST] = PACKET_ADDR_BROADCAST;
        buffer[*bufferPointer + PACKET_FIELD_DSTRT] = PACKET_ADDR_BROADCAST;
    } else if (packet.dst >= MAX_NODES) {
        return;
    } else if(packet.type & PACKET_FLAGS_NOROUTE) {
        buffer[*bufferPointer + PACKET_FIELD_DST] = packet.dst;
        buffer[*bufferPointer + PACKET_FIELD_DSTRT] = packet.dst;
    } else if(packet.type & PACKET_FLAGS_SETROUTE) {
        if (packet.data[0] == 0) {
            return;
        } else if (packet.data[0] == 1) {
            if (packet.dst != packet.data[1])
                return;

            this->setRoute(packet.dst, packet.data[1]);
            this->logRoutingTable();

            eventHndl.configChanged(this->radioConfig);

            buffer[*bufferPointer + PACKET_FIELD_DST] = packet.dst;
            buffer[*bufferPointer + PACKET_FIELD_DSTRT] = packet.data[1];

            packet.len -= 2;
            for(int i = 0; i < packet.len; i++)
                packet.data[i] = packet.data[i + 2];
        } else {
            this->setRoute(packet.dst, packet.data[1]);
            this->logRoutingTable();

            eventHndl.configChanged(this->radioConfig);

            buffer[*bufferPointer + PACKET_FIELD_DST] = packet.dst;
            buffer[*bufferPointer + PACKET_FIELD_DSTRT] = packet.data[1];

            for(int i = 2; i < packet.len; i++)
                packet.data[i-1] = packet.data[i];
            packet.data[0]--;
            packet.len--;
        }
    } else if (this->getRoute(packet.dst) == 0) {
        return;
    } else {
        buffer[*bufferPointer + PACKET_FIELD_DST] = packet.dst;
        buffer[*bufferPointer + PACKET_FIELD_DSTRT] = this->getRoute(packet.dst);
    }

    buffer[*bufferPointer + PACKET_FIELD_SRCRT] = this->radioConfig.address;
    buffer[*bufferPointer + PACKET_FIELD_SRC] = packet.src;
    buffer[*bufferPointer + PACKET_FIELD_ID] = packet.id;
    buffer[*bufferPointer + PACKET_FIELD_TYPE] = packet.type;
    buffer[*bufferPointer + PACKET_FIELD_LIFE] = packet.life;
    buffer[*bufferPointer + PACKET_FIELD_DATALEN] = packet.len;
    for(int i = 0; i < packet.len; i++)
        buffer[*bufferPointer + PACKET_FIELD_DATA + i] = packet.data[i];

    buffer[*bufferPointer + PACKET_FIELD_CRC] = Crc::crc8(&buffer[*bufferPointer], PACKET_HEADER_LEN - 1);
    *bufferPointer += PACKET_HEADER_LEN + buffer[*bufferPointer + PACKET_FIELD_DATALEN];
    callbacks[*callbacksPointer] = packet.callback;
    (*callbacksPointer)++;

    if (this->radioConfig.address == packet.src)
        this->packetsTransmitted++;
    else
        this->packetsRtTransmitted++;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::sendFirstPacket(uint8_t* buffer, uint16_t* bufferPointer)
{
    this->eventHndl.transmissionStarted();

    this->sx.stopRx();

    this->sx.clearFlags(SX127X_TX_DONE);
    uint8_t packetLen = buffer[PACKET_FIELD_DATALEN] + PACKET_HEADER_LEN;

    while((this->sx.send(buffer, packetLen) != Sx127x<SpiCtrlTempl, LogTempl>::Sx127x_OK));

    while (((this->sx.getFlags() & SX127X_TX_DONE) == 0));

    this->sx.startRx();

    this->eventHndl.transmissionFinished();

    if(buffer[PACKET_FIELD_SRC] == this->radioConfig.address)
        this->startFreeze = true;

    this->ackTimeTimer = 0;
    this->ackTimeout = ACK_TIMEOUT_MS + this->sx.getRand30() * 100;
    if((buffer[PACKET_FIELD_SRC] == buffer[PACKET_FIELD_SRCRT]) && (buffer[PACKET_FIELD_TYPE] & PACKET_FLAGS_ACK_REQ))
        this->ackWaitId = buffer[PACKET_FIELD_ID];
    else
        this->dropFirstPacket(buffer, bufferPointer, TxResultNoAck);

    this->log.printWithDate("TX [");
    this->log.printAsHex(buffer, packetLen);
    this->log.printWithNewline("]");
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::dropFirstPacket(uint8_t* buffer, uint16_t* bufferPointer, TxResult result)
{
    TxFinishedCallback* callbacks;
    uint16_t* callbacksPointer;

    if(buffer[PACKET_FIELD_TYPE] & PACKET_FLAGS_ACK_RSP) {
        callbacks = this->txAckCallbacks;
        callbacksPointer = &this->txAckCallbacksPointer;
    } else {
        callbacks = this->txCallbacks;
        callbacksPointer = &this->txCallbacksPointer;
    }

    if(callbacks[0] != NULL) {
        RadioTxPacket txp;
        txp.dst = buffer[PACKET_FIELD_DST];
        txp.src = buffer[PACKET_FIELD_SRC];
        txp.type = buffer[PACKET_FIELD_TYPE];
        txp.id = buffer[PACKET_FIELD_ID];
        txp.life = buffer[PACKET_FIELD_LIFE];
        txp.len = buffer[PACKET_FIELD_DATALEN];
        txp.data = &buffer[PACKET_FIELD_DATA];
        callbacks[0](result, &txp);
    }

    for(uint16_t i=1; i<*callbacksPointer; i++)
        callbacks[i-1] = callbacks[i];
    (*callbacksPointer)--;

    uint8_t packetLen = buffer[PACKET_FIELD_DATALEN] + PACKET_HEADER_LEN;
    for(uint32_t i=packetLen; i<*bufferPointer; i++)
        buffer[i-packetLen] = buffer[i];
    *bufferPointer -= packetLen;
    this->ackWaitId = 0;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::process(uint32_t elapsedMs)
{
    this->updatePacketTime(elapsedMs);

    if(this->startFreeze) {
        this->freezeTimer = 0;
        this->freezeTimeout = FREEZE_TIMEOUT_MS + this->sx.getRand30() * 100;
        this->freeze = true;
        this->startFreeze = false;
    } else if(this->freeze) {
        this->freezeTimer += elapsedMs;
        if(this->freezeTimer >= this->freezeTimeout)
            this->freeze = false;
    }

    if(this->waitingConfigAck) {
        this->configChangeTimer += elapsedMs;
        if(this->configChangeTimer >= CONFIG_CHANGE_TIMEOUT_MS) {
            this->radioConfig = this->radioConfigOld;
            this->init();
            this->waitingConfigAck = false;
        }
    }

    uint8_t mode = this->sx.getMode();

    if((mode & SX127X_MODE) != SX127X_MODE_RXCONTINUOUS) {
        this->log.print("ERROR WRONG MODE ");
        this->log.printAsHex(&mode, 1);
        this->log.printWithNewline("");
    }


    if((mode & SX127X_MODE) == SX127X_MODE_STDBY)
        this->sx.startRx();

    uint8_t status = this->sx.getStatus();
    if(status & SX127X_MODEM_STATUS_CLEAR) {
        this->log.printWithNewline("MODEM CLEAR");
        this->init();
        return;
    }

    if(status & SX127X_MODEM_STATUS_SIGNAL_DETECTED)
        return;

    uint8_t flags = this->sx.getFlags();
    this->sx.clearFlags();

    if(this->txAckBufferPointer > 0 && !this->freeze) {
        this->sendFirstPacket(this->txAckBuffer, &this->txAckBufferPointer);
    } else if(this->txBufferPointer > 0 && !this->freeze) {
        if(this->ackWaitId == 0) {
            this->sendFirstPacket(this->txBuffer, &this->txBufferPointer);
            this->ackSendTries = 1;
        } else {
            this->ackTimeTimer += elapsedMs;
            if(this->ackTimeTimer >= this->ackTimeout) {
                if(this->ackSendTries < ACK_MAX_TRIES) {
                    this->sendFirstPacket(this->txBuffer, &this->txBufferPointer);
                    this->ackSendTries++;
                }  else {
                    this->dropFirstPacket(this->txBuffer, &this->txBufferPointer, TxResultFailed);
                    this->packetsDropped++;
                }
            }
        }
    } else if(this->changeConfig) {
        this->init();
        this->changeConfig = false;
        this->waitingConfigAck = true;
        this->configChangeTimer = 0;
    }

    if(flags & SX127X_RX_DONE)
    {
        uint8_t dataLen;
        this->sx.stopRx();
        this->sx.getData(this->rxBuffer, &dataLen);
        this->sx.startRx();

        this->log.printWithDate("RX [");
        this->log.printAsHex(this->rxBuffer, dataLen);
        this->log.printWithNewline("]");

        if(flags & SX127X_PAYLOAD_CRC_ERROR) {
            this->log.printWithNewline("BAD CRC");
        } else {
            uint16_t handledDataLen = 0;
            while (handledDataLen < dataLen)
                handledDataLen += this->handleMessage(this->rxBuffer + handledDataLen);
        }
    }
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
uint8_t RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::getNextPacketId(void)
{
    this->packetId++;
    if(this->packetId == 0)
        this->packetId++;
    return this->packetId;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::handleDataMessage(uint8_t* msg)
{
    if((msg[PACKET_FIELD_TYPE] & PACKET_FLAGS_ACK_RSP) && (ackWaitId == msg[PACKET_FIELD_ID]))
        this->dropFirstPacket(this->txBuffer, &this->txBufferPointer, TxResultOk);

    RadioRxPacket rxp;
    rxp.type = msg[PACKET_FIELD_TYPE];
    rxp.src = msg[PACKET_FIELD_SRC];
    rxp.srcrt = msg[PACKET_FIELD_SRCRT];
    rxp.dst = msg[PACKET_FIELD_DST];
    rxp.id = msg[PACKET_FIELD_ID];
    rxp.rssi = this->getRssi(msg[PACKET_FIELD_SRCRT]);
    rxp.snr = this->getSnr(msg[PACKET_FIELD_SRCRT]);
    rxp.life = msg[PACKET_FIELD_LIFE];
    rxp.data = &msg[PACKET_FIELD_DATA];
    rxp.len = msg[PACKET_FIELD_DATALEN];

    this->eventHndl.packetReceived(rxp);
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::handleStatMessage(uint8_t* msg)
{
    if(msg[PACKET_FIELD_TYPE] & PACKET_FLAGS_ACK_RSP) {

        if((msg[PACKET_FIELD_TYPE] & PACKET_FLAGS_ACK_RSP) && (ackWaitId == msg[PACKET_FIELD_ID]))
            this->dropFirstPacket(this->txBuffer, &this->txBufferPointer, TxResultOk);

        RadioRxPacket rxp;
        rxp.type = msg[PACKET_FIELD_TYPE];
        rxp.src = msg[PACKET_FIELD_SRC];
        rxp.srcrt = msg[PACKET_FIELD_SRCRT];
        rxp.dst = msg[PACKET_FIELD_DST];
        rxp.id = msg[PACKET_FIELD_ID];
        rxp.rssi = this->getRssi(msg[PACKET_FIELD_SRCRT]);
        rxp.snr = this->getSnr(msg[PACKET_FIELD_SRCRT]);
        rxp.life = msg[PACKET_FIELD_LIFE];
        rxp.data = &msg[PACKET_FIELD_DATA];
        rxp.len = msg[PACKET_FIELD_DATALEN];
        this->eventHndl.packetReceived(rxp);
        return;
    }

    if((msg[PACKET_FIELD_TYPE] & PACKET_FLAGS_ACK_REQ) == 0)
        return;

    RadioTxPacket txp;
    switch(msg[PACKET_FIELD_DATA]) {
    case PACKET_STAT_ACTIVE:
        txp.len = 4;
        this->packetBuffer[0] = PACKET_STAT_ACTIVE;
        this->packetBuffer[1] = this->radioConfig.nodeId;
        this->packetBuffer[2] = this->rssi[msg[PACKET_FIELD_SRCRT]];
        this->packetBuffer[3] = this->snr[msg[PACKET_FIELD_SRCRT]];
        break;
    case PACKET_STAT_GET_RTABLE:
        txp.len = MAX_NODES;
        this->packetBuffer[0] = PACKET_STAT_GET_RTABLE;
        for(int i = 1; i < MAX_NODES; i++)
            this->packetBuffer[i] = this->getRoute(i);
        break;
    case PACKET_STAT_SET_RTABLE:
        txp.len = 1;
        this->packetBuffer[0] = PACKET_STAT_SET_RTABLE;
        for(int i = 1; i < msg[PACKET_FIELD_DATALEN]; i++)
            this->setRoute(i, msg[PACKET_FIELD_DATA + i]);
        this->logRoutingTable();

        eventHndl.configChanged(this->radioConfig);

        break;
    case PACKET_STAT_GET_RSSI:
        txp.len = MAX_NODES + 1;
        this->packetBuffer[0] = PACKET_STAT_GET_RSSI;
        for(int i = 0; i < MAX_NODES; i++)
            this->packetBuffer[i+1] = this->rssi[i];
        break;
    case PACKET_STAT_GET_SNR:
        txp.len = MAX_NODES + 1;
        this->packetBuffer[0] = PACKET_STAT_GET_SNR;
        for(int i = 0; i < MAX_NODES; i++)
            this->packetBuffer[i+1] = this->snr[i];
        break;
    case PACKET_STAT_GET_DROP:
        txp.len = 5;
        this->packetBuffer[0] = PACKET_STAT_GET_DROP;
        this->packetBuffer[1] = (this->packetsDropped >> 24) & 0xFF;
        this->packetBuffer[2] = (this->packetsDropped >> 16) & 0xFF;
        this->packetBuffer[3] = (this->packetsDropped >> 8) & 0xFF;
        this->packetBuffer[4] = this->packetsDropped & 0xFF;
        break;
    case PACKET_STAT_SET_CONFIG:
        txp.len = 1;
        this->packetBuffer[0] = PACKET_STAT_SET_CONFIG;

        this->changeConfig = true;
        this->radioConfigOld = this->radioConfig;
        this->radioConfig.bandwidth = (Sx127x_BW)msg[PACKET_FIELD_DATA + 1];
        this->radioConfig.spreadingFactor = (Sx127x_SF)msg[PACKET_FIELD_DATA + 2];
        this->radioConfig.codingRate = (Sx127x_CR)msg[PACKET_FIELD_DATA + 3];
        this->radioConfig.outputPower = msg[PACKET_FIELD_DATA + 4];
        break;
    case PACKET_STAT_SET_CONFIG_ACK:
        txp.len = 1;
        this->packetBuffer[0] = PACKET_STAT_SET_CONFIG_ACK;
        this->waitingConfigAck = false;

        eventHndl.configChanged(this->radioConfig);

        break;
    }

    txp.type = PACKET_TYPE_STAT | PACKET_FLAGS_ACK_RSP;
    txp.src = this->radioConfig.address;
    txp.dst = msg[PACKET_FIELD_SRC];
    txp.life = PACKET_MAX_LIFE;
    txp.id = msg[PACKET_FIELD_ID];
    txp.data = this->packetBuffer;
    txp.callback = NULL;
    this->addPacketToQueue(txp);
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
uint8_t RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::handleMessage(uint8_t* msg)
{
    uint8_t crc = Crc::crc8(msg, PACKET_HEADER_LEN - 1);
    if(crc != msg[PACKET_FIELD_CRC]) {
        this->log.printWithNewline("BAD HEADER CRC");
        return 0xFF;
    }

    uint8_t dstrt = msg[PACKET_FIELD_DSTRT];
    uint8_t dst = msg[PACKET_FIELD_DST];
    uint8_t srcrt = msg[PACKET_FIELD_SRCRT];

    this->setRssi(srcrt, this->sx.getRssi());
    this->setSnr(srcrt, this->sx.getSnr());
    this->setLastPacketTime(srcrt, 0);

    if(msg[PACKET_FIELD_LIFE] > 0)
        msg[PACKET_FIELD_LIFE]--;

    if(dst == PACKET_ADDR_BROADCAST || (dstrt == this->radioConfig.address && dst == this->radioConfig.address)) {
        if (msg[PACKET_FIELD_TYPE] & PACKET_FLAGS_SETROUTE) {
            this->setRoute(msg[PACKET_FIELD_SRC], srcrt);
            this->logRoutingTable();

            eventHndl.configChanged(this->radioConfig);
        }

        this->packetsReceived++;
        switch(msg[PACKET_FIELD_TYPE] & PACKET_TYPE) {
        case PACKET_TYPE_DATA:
            this->handleDataMessage(msg);
            break;
        case PACKET_TYPE_STAT:
            this->handleStatMessage(msg);
            break;
        }
    } else if(dstrt == this->radioConfig.address && msg[PACKET_FIELD_LIFE] > 0) {
        if (msg[PACKET_FIELD_TYPE] & PACKET_FLAGS_SETROUTE) {
            this->setRoute(msg[PACKET_FIELD_SRC], srcrt);
            this->logRoutingTable();

            eventHndl.configChanged(this->radioConfig);
        }

        this->packetsRtReceived++;
        RadioTxPacket p;
        p.type = msg[PACKET_FIELD_TYPE];
        p.src = msg[PACKET_FIELD_SRC];
        p.dst = msg[PACKET_FIELD_DST];
        p.life = msg[PACKET_FIELD_LIFE];
        p.id = msg[PACKET_FIELD_ID];
        p.data = &msg[PACKET_FIELD_DATA];
        p.len = msg[PACKET_FIELD_DATALEN];
        p.callback = NULL;
        this->addPacketToQueue(p);
    }

    return msg[PACKET_FIELD_DATALEN] + PACKET_HEADER_LEN;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::setRoute(uint8_t dst, uint8_t dstrt)
{
    if(dst >= MAX_NODES || dstrt >= MAX_NODES)
        return;
    this->radioConfig.routingTable[dst] = dstrt;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
uint8_t RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::getRoute(uint8_t dst)
{
    if(dst >= MAX_NODES)
        return 0;
    return this->radioConfig.routingTable[dst];
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::setRssi(uint8_t addr, uint8_t rssi)
{
    if(addr >= MAX_NODES)
        return;
    this->rssi[addr] = rssi;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
uint8_t RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::getRssi(uint8_t addr)
{
    if(addr >= MAX_NODES)
        return 0;
    return this->rssi[addr];
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::setSnr(uint8_t addr, int8_t snr)
{
    if(addr >= MAX_NODES)
        return;
    this->snr[addr] = snr;
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
int8_t RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::getSnr(uint8_t addr)
{
    if(addr >= MAX_NODES)
        return 0;
    return this->snr[addr];
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::setLastPacketTime(uint8_t addr, uint16_t time)
{
    if(addr >= MAX_NODES || time > LAST_PACKET_MAX_TIME_S)
        return;
    this->lastPacketTime[addr] = time;

}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
uint16_t RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::getLastPacketTime(uint8_t addr)
{
    if(addr >= MAX_NODES)
        return LAST_PACKET_MAX_TIME_S;
    return this->lastPacketTime[addr];
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::logRoutingTable(void)
{
    this->log.printWithDate("RT [");
    this->log.printAsHex(this->radioConfig.routingTable, MAX_NODES);
    this->log.printWithNewline("]");
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::logRssi(void)
{
    int rIndex;

    this->log.printWithDate("RSSI [");

    for(rIndex = 1; rIndex < MAX_NODES; rIndex++) {
        this->log.print((int)this->getRssi(rIndex) - 164);
        this->log.print(" ");
    }
    this->log.printWithNewline("]");

    this->log.printWithDate("SNR [");
    for(rIndex = 1; rIndex < MAX_NODES; rIndex++) {
        this->log.print((int)this->getSnr(rIndex)/4);
        this->log.print(" ");
    }
    this->log.printWithNewline("]");
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::logCurrentRssiAndStatus(uint8_t currentRssi, uint8_t status)
{
    this->log.print("Current RSSI ");
    this->log.print(currentRssi - 164);
    this->log.print("\t");
    this->log.printAsHex(status);
    this->log.printWithNewline("");
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::logPacketStats(void)
{
    this->log.print("Packets RX[");
    this->log.print((int)this->packetsReceived);
    this->log.print("] TX[");
    this->log.print((int)this->packetsTransmitted);
    this->log.print("] RXRT[");
    this->log.print((int)this->packetsRtReceived);
    this->log.print("] TXRT[");
    this->log.print((int)this->packetsRtTransmitted);
    this->log.print("] TXD[");
    this->log.print((int)this->packetsDropped);
    this->log.printWithNewline("]");
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::updatePacketTime(uint32_t elapsedMs)
{
    this->packetTimeTimer += elapsedMs;

    if(this->packetTimeTimer >= LAST_PACKET_TIME_DISPLAY_PERIOD_MS) {
        this->packetTimeTimer = 0;
        for(int i=0; i<MAX_NODES; i++)
            if(this->lastPacketTime[i] < LAST_PACKET_MAX_TIME_S)
                this->lastPacketTime[i]++;
    }
}

template <class SpiCtrlTempl, class EventHndlTempl, class LogTempl>
void RadioManager<SpiCtrlTempl, EventHndlTempl, LogTempl>::logPacketTime(void)
{
    int rIndex;
    this->log.printWithDate("LAST PCK TIME [");

    for(rIndex = 1; rIndex < MAX_NODES; rIndex++) {
        this->log.print((int)this->lastPacketTime[rIndex]);
        this->log.print(" ");
    }
    this->log.printWithNewline("]");
}

#endif

