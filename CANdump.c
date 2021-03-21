// Embedded Systems 2 - Bussysteme 2021
// Niclas Jarowsky
// Danny Steinbrecher

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, char *argv[]) {

    /* Create the socket */
    int skt = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    /* Locate the interface you wish to use */
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(skt, SIOCGIFINDEX, &ifr); /* get device's index */

    /* Bind the socket to that interface */
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(skt, (struct sockaddr *) &addr, sizeof(addr));

    struct can_frame frame;

    while (1) {
        int bytes_read = read(skt, &frame, sizeof(frame));

        printf("CAN Frame Id=%04x DLC=%d ", frame.can_id, frame.can_dlc);
        for (int k = 0; k < frame.can_dlc; k++)
            printf("%02x ", frame.data[k]);
        printf("\n");
    }

    return 0;
}
