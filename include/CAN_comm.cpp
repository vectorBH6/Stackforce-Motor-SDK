#include "CAN_comm.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace {
int g_can_socket = -1;
constexpr const char* kDefaultCanIf = "can0";

void close_can_socket() {
    if (g_can_socket >= 0) {
        close(g_can_socket);
        g_can_socket = -1;
    }
}
}  // namespace

void CANInit(){
    if(g_can_socket >= 0){
        return;
    }

    g_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(g_can_socket < 0){
        std::perror("socket");
        return;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, kDefaultCanIf, IFNAMSIZ - 1);
    if(ioctl(g_can_socket, SIOCGIFINDEX, &ifr) < 0){
        std::perror("ioctl SIOCGIFINDEX");
        close_can_socket();
        return;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(g_can_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0){
        std::perror("bind");
        close_can_socket();
        return;
    }

    int flags = fcntl(g_can_socket, F_GETFL, 0);
    if(flags >= 0){
        fcntl(g_can_socket, F_SETFL, flags | O_NONBLOCK);
    }

    std::printf("CAN socket initialized on %s\n", kDefaultCanIf);
}

void recCANMessage(){
    if(g_can_socket < 0){
        return;
    }

    struct can_frame rxFrame{};
    ssize_t nbytes = recv(g_can_socket, &rxFrame, sizeof(rxFrame), MSG_DONTWAIT);
    if(nbytes < 0){
        if(errno == EAGAIN || errno == EWOULDBLOCK){
            return;
        }
        std::perror("recv");
        return;
    }

    if(static_cast<size_t>(nbytes) < sizeof(struct can_frame)){
        return;
    }

    recNum++;
    bool isExtended = (rxFrame.can_id & CAN_EFF_FLAG) != 0;
    bool isRTR = (rxFrame.can_id & CAN_RTR_FLAG) != 0;
    if(!isExtended && !isRTR){
        uint8_t DLC = rxFrame.can_dlc;
        uint8_t nodeID = rxFrame.can_id & 0x7F;
        uint32_t funcID = rxFrame.can_id & 0x780;
        if(DLC >= 6 && funcID == HEARTBEAT_FUNC_ID){
            MITState_callback(nodeID, rxFrame.data);
        }
    }
}


void sendCANCommand(uint32_t nodeID, uint32_t msgID, uint8_t *data){
    if(g_can_socket < 0){
        std::fprintf(stderr, "CAN socket not initialized\n");
        return;
    }

    struct can_frame txFrame{};
    txFrame.can_id = (msgID + nodeID) & CAN_SFF_MASK;
    txFrame.can_dlc = 8;
    std::memcpy(txFrame.data, data, 8);

    if (write(g_can_socket, &txFrame, sizeof(txFrame)) == sizeof(txFrame)) {
        sendNum++;
    } else {
        std::perror("write");
    }
}

void MITState_callback(uint8_t nodeID, uint8_t *data){
    uint32_t posInt =  (data[1]<<8) | data[2];
    uint32_t velInt = (data[3]<<4) | (data[4]>>4 & 0xF);
    uint32_t torInt = ((data[4]&0xF)<<8) | data[5];

    uint8_t index = nodeID - 1;
    if(index < (sizeof(devicesState)/sizeof(devicesState[0]))){
        devicesState[index].pos = uint_to_float(posInt,devicesState[index].posMin, devicesState[index].posMax, 16);
        devicesState[index].vel = uint_to_float(velInt,devicesState[index].velMin, devicesState[index].velMax, 12);
        devicesState[index].tor = uint_to_float(torInt,devicesState[index].torMin, devicesState[index].torMax, 12);
    }

}




void sendMITCommand(uint8_t nodeID,MIT command){
    uint8_t MITcommand[8];

    LIMIT_MIN_MAX(command.pos, command.posMin, command.posMax);
    LIMIT_MIN_MAX(command.vel, command.velMin, command.velMax);
    LIMIT_MIN_MAX(command.kp, command.kpMin, command.kpMax);
    LIMIT_MIN_MAX(command.kd, command.kdMin, command.kdMax);
    LIMIT_MIN_MAX(command.tor, command.torMin, command.torMax);

    uint32_t posInt = float_to_uint(command.pos, command.posMin, command.posMax, 16);
    uint32_t velInt = float_to_uint(command.vel, command.velMin, command.velMax, 12);
    uint32_t kpInt = float_to_uint(command.kp, command.kpMin, command.kpMax, 12);
    uint32_t kdInt = float_to_uint(command.kd, command.kdMin, command.kdMax, 12);
    uint32_t torInt = float_to_uint(command.tor, command.torMin, command.torMax, 12);

    MITcommand[0] = posInt>>8;
    MITcommand[1] = posInt&0xFF;
    MITcommand[2] = velInt>>4;
    MITcommand[3] = ((velInt&0xF)<<4) | (kpInt>>8);
    MITcommand[4] = kpInt&0xFF;
    MITcommand[5] = kdInt>>4;
    MITcommand[6] = ((kdInt&0xF)<<4) | (torInt>>8);
    MITcommand[7] = torInt&0xFF;

    sendCANCommand(nodeID, FUNC_ID_RPDO1, MITcommand);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

void disable(uint8_t nodeID){
    uint8_t MITcommand[8];
    MITcommand[0] = 0xFF;
    MITcommand[1] = 0xFF;
    MITcommand[2] = 0xFF;
    MITcommand[3] = 0xFF;
    MITcommand[4] = 0xFF;
    MITcommand[5] = 0xFF;
    MITcommand[6] = 0xFF;
    MITcommand[7] = 0xFD;
    sendCANCommand(nodeID, FUNC_ID_NMT, MITcommand);
}

void enable(uint8_t nodeID){
    uint8_t MITcommand[8];
    MITcommand[0] = 0xFF;
    MITcommand[1] = 0xFF;
    MITcommand[2] = 0xFF;
    MITcommand[3] = 0xFF;
    MITcommand[4] = 0xFF;
    MITcommand[5] = 0xFF;
    MITcommand[6] = 0xFF;
    MITcommand[7] = 0xFC;
    sendCANCommand(nodeID, FUNC_ID_NMT, MITcommand);
}

void zeroPos(uint8_t nodeID){
    uint8_t MITcommand[8];
    MITcommand[0] = 0xFF;
    MITcommand[1] = 0xFF;
    MITcommand[2] = 0xFF;
    MITcommand[3] = 0xFF;
    MITcommand[4] = 0xFF;
    MITcommand[5] = 0xFF;
    MITcommand[6] = 0xFF;
    MITcommand[7] = 0xFE;

    sendCANCommand(nodeID, FUNC_ID_RPDO1, MITcommand);
}