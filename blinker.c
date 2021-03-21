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


// --- Types ---
enum DIRECTION {
    LEFT, RIGHT
};
enum MODE {
    SINGLE, STRIP, BLINK
};
enum INDICATOR {
    LEFT_INDICATOR, RIGHT_INDICATOR
};

// --- Variables ---
enum DIRECTION direction = LEFT;
enum MODE mode = SINGLE;

int skt; // CAN-Socket
unsigned int nTime = 0;
unsigned int nStateChanged = 0;
unsigned int button = 0;
unsigned int timeRV2 = 0;

unsigned long sCycleCounter = 0;
unsigned long sLastEvent = 0;
unsigned int interval = 0;

int blinkLightMM1[8];
int blinkLightMM2[8];

unsigned int mmLeftMode = 0;
unsigned int mmLeftMultiple = 0;
unsigned int mmLeftDirection = 0;

unsigned int mmRightMode = 0;
unsigned int mmRightMultiple = 0;
unsigned int mmRightDirection = 0;

static unsigned int countMM = 0;
bool startIndicator = false;

void setArrayToNull(int *array) {
    for (int i = 0; i <= 7; i++) {
        array[i] = 0;
    }
}

void setBlink(int mmDirection, int mmMode, int mmMultiple, int *blinkLightArray) {

    
    if (!mmMode) { // Simple Blink-Mode
        if (countMM < 4) {
            for (int i = 0; i <= 7; i++) {
                blinkLightArray[i] = 1;
            }
        } else {
            for (int i = 0; i <= 7; i++) {
                blinkLightArray[i] = 0;
            }
        }
    } else if (mmDirection == 0) { //Running Light Left to Right
        switch (countMM) {
            case 1:
                blinkLightArray[0] = 1;
                break;
            case 0: 
                setArrayToNull(blinkLightArray);
                break;
            default:
                for (int i = 7; i >= 0; i--) {
                    blinkLightArray[i] = blinkLightArray[i - 1];
                }
                blinkLightArray[0] = mmMultiple; // Single or Strip
                break;
        }
    } else {//Running Light Right to Left
        switch (countMM) {
            case 1:
                blinkLightArray[7] = 1;
                break;
            case 0:
                setArrayToNull(blinkLightArray);
                break;
            default:
                for (int i = 0; i < 7; i++) {
                    blinkLightArray[i] = blinkLightArray[i + 1];
                }
                blinkLightArray[7] = mmMultiple; // Single or Strip
                break;
        }
    }
}

//reset all LED´s 
void stopIndicator() {

    struct can_frame frame;
    int bytes_sent;

    CAN_MM_DigitalOutputsT tMMDOutputs;

    tMMDOutputs.LED1 = 0;
    tMMDOutputs.LED2 = 0;
    tMMDOutputs.LED3 = 0;
    tMMDOutputs.LED4 = 0;
    tMMDOutputs.LED5 = 0;
    tMMDOutputs.LED6 = 0;
    tMMDOutputs.LED7 = 0;
    tMMDOutputs.LED8 = 0;

    frame.can_dlc = sizeof(CAN_MM_DigitalOutputsT);
    memcpy(frame.data, (unsigned char *) &tMMDOutputs, frame.can_dlc);

    frame.can_id = MM_DIGITAL_OUTPUTS_1;
    bytes_sent = write(skt, &frame, sizeof(frame));

    frame.can_id = MM_DIGITAL_OUTPUTS_2;
    bytes_sent = write(skt, &frame, sizeof(frame));

    countMM = 0;
    setArrayToNull(blinkLightMM1);
    setArrayToNull(blinkLightMM2);
}


void setMM(enum INDICATOR indicator) {

    struct can_frame frame;
    int bytes_sent;

    CAN_MM_DigitalOutputsT tMMDOutputs;

    switch (indicator) {
        case LEFT_INDICATOR:
            frame.can_id = MM_DIGITAL_OUTPUTS_1;
            setBlink(mmLeftDirection, mmLeftMode, mmLeftMultiple, blinkLightMM1);
            tMMDOutputs.LED1 = blinkLightMM1[0];
            tMMDOutputs.LED2 = blinkLightMM1[1];
            tMMDOutputs.LED3 = blinkLightMM1[2];
            tMMDOutputs.LED4 = blinkLightMM1[3];
            tMMDOutputs.LED5 = blinkLightMM1[4];
            tMMDOutputs.LED6 = blinkLightMM1[5];
            tMMDOutputs.LED7 = blinkLightMM1[6];
            tMMDOutputs.LED8 = blinkLightMM1[7];
            break;
        case RIGHT_INDICATOR:
            frame.can_id = MM_DIGITAL_OUTPUTS_2;
            setBlink(mmRightDirection, mmRightMode, mmRightMultiple, blinkLightMM2);
            tMMDOutputs.LED1 = blinkLightMM2[0];
            tMMDOutputs.LED2 = blinkLightMM2[1];
            tMMDOutputs.LED3 = blinkLightMM2[2];
            tMMDOutputs.LED4 = blinkLightMM2[3];
            tMMDOutputs.LED5 = blinkLightMM2[4];
            tMMDOutputs.LED6 = blinkLightMM2[5];
            tMMDOutputs.LED7 = blinkLightMM2[6];
            tMMDOutputs.LED8 = blinkLightMM2[7];
            break;
    }

    frame.can_dlc = sizeof(CAN_MM_DigitalOutputsT);
    memcpy(frame.data, (unsigned char *) &tMMDOutputs, frame.can_dlc);
    bytes_sent = write(skt, &frame, sizeof(frame));
}

bool switchTK_RL1() {
    struct can_frame frame;
    int bytes_sent;
    
    // TinkerKit
    CAN_TK_DigitalOutputsT tTKDOutputs;

    if (countMM < 4) {
        tTKDOutputs.RL1 = 1;
    } else {
        tTKDOutputs.RL1 = 0;
    }

    tTKDOutputs.LED1 = 0;
    tTKDOutputs.LED2 = 0;
    tTKDOutputs.LED3 = 0;
    tTKDOutputs.LED4 = 0;

    frame.can_dlc = sizeof(CAN_TK_DigitalOutputsT);
    memcpy(frame.data, (unsigned char *) &tTKDOutputs, frame.can_dlc);

    frame.can_id = TK_DIGITAL_OUTPUTS_1;
    bytes_sent = write(skt, &frame, sizeof(frame));

    return true;
}

int main(int argc, char *argv[]) {

    setArrayToNull(blinkLightMM1);
    setArrayToNull(blinkLightMM2);

    skt = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(skt, SIOCGIFINDEX, &ifr); // search for interface index

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(skt, (struct sockaddr *) &addr, sizeof(addr)); // bind socket
    stopIndicator();
    int takt = 0;

    while (1) {
        sCycleCounter++;
        struct can_frame frame;
        int bytes_read = read(skt, &frame, sizeof(frame));
        if (bytes_read > 0) { // if something received


            if (frame.can_id == DD_PERIODIC_INPUTS_1) {
                CAN_DD_PeriodicInputsT *tInputs = (CAN_DD_PeriodicInputsT *) frame.data;
                interval = ((tInputs->RV6 * 3000) / 1023) + 1000; //calc the correct frequence: 0 - 1023 bit -> 1Hz - 4Hz
            } else if (frame.can_id == DD_TRIGGERED_INPUTS_1) {
                CAN_DD_TriggeredInputsT *tInputs = (CAN_DD_TriggeredInputsT *) frame.data;

                if (!tInputs->SW1) {
                    startIndicator = !startIndicator;
                    if (!startIndicator) {
                        stopIndicator();
                    }
                }
            } else if (frame.can_id == MM_PERIODIC_INPUTS_1) {
                CAN_MM_PeriodicInputsT *tInputs = (CAN_MM_PeriodicInputsT *) frame.data;
                mmLeftMode = tInputs->SW1;
                mmLeftMultiple = tInputs->SW2;
                mmLeftDirection = tInputs->SW3;
            } else if (frame.can_id == MM_PERIODIC_INPUTS_2) {
                CAN_MM_PeriodicInputsT *tInputs = (CAN_MM_PeriodicInputsT *) frame.data;
                mmRightMode = tInputs->SW1;
                mmRightMultiple = tInputs->SW2;
                mmRightDirection = tInputs->SW3;
            }
        }

        if (sCycleCounter - sLastEvent >= interval / 54) { //we need every phase 9 Sections (and a adjustmant to have a period from 1 second) 
            sLastEvent = sCycleCounter;
            if (startIndicator) {
                setMM(LEFT_INDICATOR);
                setMM(RIGHT_INDICATOR);
                switchTK_RL1();
                countMM = (countMM < 8) ? countMM + 1 : 0; //count 0 to 8
            }
        }
        usleep(1000); // cycle time 1 ms
    }

    return 0;
}
