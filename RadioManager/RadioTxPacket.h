#ifndef RADIOTXPACKET_H
#define RADIOTXPACKET_H

#define RADIOTXPACKET_FIELDS 6

typedef enum {
    TxResultOk,
    TxResultFailed,
    TxResultNoAck,
} TxResult;

typedef struct RadioTxPacket RadioTxPacket;

typedef void (*TxFinishedCallback)(TxResult result, RadioTxPacket* txp);

struct RadioTxPacket{
    uint8_t dst;
    uint8_t src;
    uint8_t type;
    uint8_t life;
    uint8_t id;
    uint8_t len;
    uint8_t* data;
    TxFinishedCallback callback;
};

#endif
