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

// --- Types ---
enum STATE {
    stINIT, stSIGNAL, stGREEN, stYELLOW, stRED, stYELLOW_RED
};

// --- Variables ---
enum STATE enState = stINIT;

unsigned int nTime = 0;
unsigned int nStateChanged = 0;
unsigned int button = 0;
unsigned int timeRV2 = 0;

unsigned long sCycleCounter = 0;
unsigned long sLastEvent1000 = 0;


bool SetLed(bool pValueRed, bool pValueYellow, bool pValueGreen, bool pValueSignal) {
    struct can_frame frame;
    int bytes_sent;

    // -- Digital Outputs TinkerKit

    CAN_TK_DigitalOutputsT tTKDOutputs;

    tTKDOutputs.LED1 = pValueRed;
    tTKDOutputs.LED2 = pValueYellow;
    tTKDOutputs.LED3 = pValueGreen;
    tTKDOutputs.LED4 = pValueSignal;
    tTKDOutputs.RL1 = 0;


    frame.can_id = TK_DIGITAL_OUTPUTS_1;
    frame.can_dlc = sizeof(CAN_TK_DigitalOutputsT);
    memcpy(frame.data, (unsigned char *) &tTKDOutputs, frame.can_dlc);
    bytes_sent = write(skt, &frame, sizeof(frame));

    return true;
}


void SetNextStateAndTheWaitTime(enum STATE state, int time) {
    nStateChanged = nTime + time;
    enState = state;
}


void StateMachine() {
    switch (enState) {
        case stINIT:
            SetLed(1, 0, 0, 0);
            enState = stRED;
            break;

        case stRED:
            if (button) {
                SetLed(1, 0, 0, 1);
                SetNextStateAndTheWaitTime(stSIGNAL, 2);
            }
            break;

        case stSIGNAL:
            if (nStateChanged - nTime <= 0) {
                SetLed(1, 1, 0, 1);
                SetNextStateAndTheWaitTime(stYELLOW_RED, 2);
            }
            break;

        case stYELLOW_RED:
            if (nStateChanged - nTime <= 0) {
                SetLed(0, 0, 1, 0);
                SetNextStateAndTheWaitTime(stGREEN, timeRV2);
            }
            break;

        case stGREEN:
            if (nStateChanged - nTime <= 0) {
                SetLed(0, 1, 0, 0);
                SetNextStateAndTheWaitTime(stYELLOW, 2);
            }
            break;

        case stYELLOW:
            if (nStateChanged - nTime <= 0) {
                SetLed(1, 0, 0, 0);
                button = 0;
                enState = stRED;
            }
            break;
    }
}

int main(int argc, char *argv[]) {
    skt = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(skt, SIOCGIFINDEX, &ifr); // search for interface index

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(skt, (struct sockaddr *) &addr, sizeof(addr)); // bind socket

    while (1) {
        sCycleCounter++;
        struct can_frame frame;
        int bytes_read = read(skt, &frame, sizeof(frame));

        if (bytes_read > 0) { // if something received
            if (frame.can_id == TK_TRIGGERED_INPUTS_1) {
                CAN_TK_TriggeredInputsT *tInputs = (CAN_TK_TriggeredInputsT *) frame.data;
                CAN_TK_PeriodicInputsT *pInputs = (CAN_TK_PeriodicInputsT *) frame.data;

                if (!tInputs->SW1) { // falling edge event
                    button = 1;
                }
            } else if (frame.can_id == TK_PERIODIC_INPUTS_1) {
                CAN_TK_PeriodicInputsT *tInputs = (CAN_TK_PeriodicInputsT *) frame.data;
                timeRV2 = (((tInputs->RV2 * 10) / 1023) + 5);
            }
        }
        if (sCycleCounter - sLastEvent1000 >= 700) {
            sLastEvent1000 = sCycleCounter;
            nTime += 1;
        }

        StateMachine();
        usleep(1000); // cycle time 1 ms
    }
    return 0;
}

