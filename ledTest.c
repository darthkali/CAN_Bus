// Embedded Systems 2 - Bussysteme 2021
// Niclas Jarowsky
// Danny Steinbrecher

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>


// ---------------------------------------------
// MicroMod
// ---------------------------------------------
#define MM_NODE_1 0x110
#define MM_PERIODIC_INPUTS_1  MM_NODE_1 + 1
#define MM_DIGITAL_OUTPUTS_1  MM_NODE_1 + 2
#define MM_ANALOG_OUTPUTS_1   MM_NODE_1 + 3

#define MM_NODE_2 0x120
#define MM_PERIODIC_INPUTS_2 MM_NODE_2 + 1
#define MM_DIGITAL_OUTPUTS_2 MM_NODE_2 + 2
#define MM_ANALOG_OUTPUTS_2  MM_NODE_2 + 3

#pragma pack(1)
typedef struct {
    unsigned int SW1: 1; // 1 bit
    unsigned int SW2: 1; // 1 bit
    unsigned int SW3: 1; // 1 bit
    unsigned int SW4: 1; // 1 bit
    unsigned int SW5: 1; // 1 bit
    unsigned int SW6: 1; // 1 bit
    unsigned int SW7: 1; // 1 bit
    unsigned int SW8: 1; // 1 bit
    unsigned int RV1: 10; // 10 bit
    unsigned int RV2: 10; // 10 bit
    unsigned int RV3: 10; // 10 bit
    unsigned int RV4: 10; // 10 bit
} CAN_MM_PeriodicInputsT;

#pragma pack(1)
typedef struct {
    unsigned int LED1: 1; // 1 bit
    unsigned int LED2: 1; // 1 bit
    unsigned int LED3: 1; // 1 bit
    unsigned int LED4: 1; // 1 bit
    unsigned int LED5: 1; // 1 bit
    unsigned int LED6: 1; // 1 bit
    unsigned int LED7: 1; // 1 bit
    unsigned int LED8: 1; // 1 bit
} CAN_MM_DigitalOutputsT;

#pragma pack(1)
typedef struct {
    unsigned int PWM1: 8; // 8 bit
    unsigned int PWM2: 8; // 8 bit
    unsigned int PWM3: 8; // 8 bit
    unsigned int PWM4: 8; // 8 bit
} CAN_MM_AnalogOutputsT;



// ---------------------------------------------
// TinkerKit
// ---------------------------------------------
#define TK_NODE_1 0x210
#define TK_PERIODIC_INPUTS_1    TK_NODE_1 + 1
#define TK_TRIGGERED_INPUTS_1   TK_NODE_1 + 2
#define TK_DIGITAL_OUTPUTS_1    TK_NODE_1 + 3
#define TK_ANALOG_OUTPUTS_1     TK_NODE_1 + 4

#pragma pack(1)
typedef struct {
    unsigned int SW1: 1; // 1 bit
    unsigned int RV1: 10; // 10 bit
    unsigned int RV2: 10; // 10 bit
    unsigned int SL1: 10; // 10 bit
    unsigned int ST1: 10; // 10 bit
} CAN_TK_PeriodicInputsT;

#pragma pack(1)
typedef struct {
    unsigned int SW1: 1; // 1 bit
} CAN_TK_TriggeredInputsT;

#pragma pack(1)
typedef struct {
    unsigned int LED1: 1; // 1 bit
    unsigned int LED2: 1; // 1 bit
    unsigned int LED3: 1; // 1 bit
    unsigned int LED4: 1; // 1 bit
    unsigned int RL1: 1; // 1 bit
} CAN_TK_DigitalOutputsT;

#pragma pack(1)
typedef struct {
    unsigned int LED4: 8; // 8 bit
} CAN_TK_AnalogOutputsT;



// ---------------------------------------------
// DrDuino
// ---------------------------------------------
#define DD_NODE_1 0x310
#define DD_PERIODIC_INPUTS_1    DD_NODE_1 + 1
#define DD_TRIGGERED_INPUTS_1   DD_NODE_1 + 2
#define DD_OUTPUTS_1            DD_NODE_1 + 3
#define DD_DIGITAL_OUTPUTS_1    DD_NODE_1 + 4
#define DD_ANALOG_OUTPUTS_1     DD_NODE_1 + 5

#pragma pack(1)
typedef struct {
    unsigned int SW1: 1; // 1 bit
    unsigned int SW2: 1; // 1 bit
    unsigned int SW3: 1; // 1 bit
    unsigned int SW4: 1; // 1 bit
    unsigned int RV1: 10; // 10 bit
    unsigned int RV2: 10; // 10 bit
    unsigned int RV3: 10; // 10 bit
    unsigned int RV4: 10; // 10 bit
    unsigned int RV5: 10; // 10 bit
    unsigned int RV6: 10; // 10 bit
} CAN_DD_PeriodicInputsT;

#pragma pack(1)
typedef struct {
    unsigned int SW1: 1; // 1 bit
    unsigned int SW2: 1; // 1 bit
    unsigned int SW3: 1; // 1 bit
    unsigned int SW4: 1; // 1 bit
} CAN_DD_TriggeredInputsT;

#pragma pack(1)
typedef struct {
    unsigned int LED1: 1; // 1 bit
    unsigned int LED2: 1; // 1 bit
    unsigned int LED3: 1; // 1 bit
    unsigned int LED4: 1; // 1 bit
    unsigned int SP1: 8; // 8 bit
} CAN_DD_OutputsT;

#pragma pack(1)
typedef struct {
    unsigned int LED1: 1; // 1 bit
    unsigned int LED2: 1; // 1 bit
    unsigned int LED3: 1; // 1 bit
    unsigned int LED4: 1; // 1 bit
} CAN_DD_DigitalOutputsT;

#pragma pack(1)
typedef struct {
    unsigned int SP1: 8; // 8 bit
} CAN_DD_AnalogOutputsT;


#define bool unsigned char
#define true 1
#define false 0
int skt; // CAN-Socket


bool switchLEDs(bool pValue) {
    struct can_frame frame;
    int bytes_sent;

    // ---------------------------------------------
    // MicroMod
    // ---------------------------------------------
    CAN_MM_DigitalOutputsT tMMDOutputs;
    CAN_MM_AnalogOutputsT tMMAOutputs;

    tMMDOutputs.LED1 = pValue;
    tMMDOutputs.LED2 = pValue;
    tMMDOutputs.LED3 = pValue;
    tMMDOutputs.LED4 = pValue;
    tMMDOutputs.LED5 = pValue;
    tMMDOutputs.LED6 = pValue;
    tMMDOutputs.LED7 = pValue;
    tMMDOutputs.LED8 = pValue;

    frame.can_dlc = sizeof(CAN_MM_DigitalOutputsT);
    memcpy(frame.data, (unsigned char *) &tMMDOutputs, frame.can_dlc);

    frame.can_id = MM_DIGITAL_OUTPUTS_1;
    bytes_sent = write(skt, &frame, sizeof(frame));

    frame.can_id = MM_DIGITAL_OUTPUTS_2;
    bytes_sent = write(skt, &frame, sizeof(frame));

    tMMAOutputs.PWM1 = pValue ? 255 : 0;
    tMMAOutputs.PWM2 = pValue ? 255 : 0;
    tMMAOutputs.PWM3 = pValue ? 255 : 0;
    tMMAOutputs.PWM4 = pValue ? 255 : 0;


    frame.can_dlc = sizeof(CAN_MM_AnalogOutputsT);
    memcpy(frame.data, (unsigned char *) &tMMAOutputs, frame.can_dlc);

    frame.can_id = MM_ANALOG_OUTPUTS_1;
    bytes_sent = write(skt, &frame, sizeof(frame));

    frame.can_id = MM_ANALOG_OUTPUTS_2;
    bytes_sent = write(skt, &frame, sizeof(frame));


    // ---------------------------------------------
    // TinkerKit
    // ---------------------------------------------
    CAN_TK_DigitalOutputsT tTKDOutputs;

    tTKDOutputs.LED1 = pValue;
    tTKDOutputs.LED2 = pValue;
    tTKDOutputs.LED3 = pValue;
    tTKDOutputs.LED4 = pValue;
    tTKDOutputs.RL1 = 0;

    frame.can_dlc = sizeof(CAN_TK_DigitalOutputsT);
    memcpy(frame.data, (unsigned char *) &tTKDOutputs, frame.can_dlc);

    frame.can_id = TK_DIGITAL_OUTPUTS_1;
    bytes_sent = write(skt, &frame, sizeof(frame));


    // ---------------------------------------------
    // DrDuino
    // ---------------------------------------------
    CAN_DD_OutputsT tDDOutputs;

    tDDOutputs.LED1 = pValue;
    tDDOutputs.LED2 = pValue;
    tDDOutputs.LED3 = pValue;
    tDDOutputs.LED4 = pValue;
    tDDOutputs.SP1 = 0;

    frame.can_dlc = sizeof(CAN_DD_OutputsT);
    memcpy(frame.data, (unsigned char *) &tDDOutputs, frame.can_dlc);

    frame.can_id = DD_OUTPUTS_1;
    bytes_sent = write(skt, &frame, sizeof(frame));

    return true;
}

int main(int argc, char *argv[]) {
    skt = socket(PF_CAN, SOCK_RAW, CAN_RAW); // create blocking socket

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(skt, SIOCGIFINDEX, &ifr); // search for interface index

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(skt, (struct sockaddr *) &addr, sizeof(addr)); // bind socket

    bool sLEDstate = false;

    switchLEDs(sLEDstate); // initial LED state

    while (1) {
        struct can_frame frame;
        int bytes_read = read(skt, &frame, sizeof(frame));

        if (frame.can_id == TK_TRIGGERED_INPUTS_1) {
            CAN_TK_TriggeredInputsT *tInputs = (CAN_TK_TriggeredInputsT *) frame.data;

            if (!tInputs->SW1) { // falling edge event
                sLEDstate = !sLEDstate;
                switchLEDs(sLEDstate);
            }
        }
    }

    return 0;
}
