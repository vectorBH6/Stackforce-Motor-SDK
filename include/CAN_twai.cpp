#include "CAN_twai.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

CAN_twai::CAN_twai(int num)
    : _num(num), _socket_fd(-1) {}

CAN_twai::~CAN_twai() {
    if (_socket_fd >= 0) {
        close(_socket_fd);
        _socket_fd = -1;
    }
}

void CAN_twai::CAN_Init(uint8_t TX_PIN,uint8_t RX_PIN)
{
  (void)TX_PIN;
  (void)RX_PIN;

  if (_socket_fd >= 0) {
    return;
  }

  _socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (_socket_fd < 0) {
    std::perror("socket");
    return;
  }

  char ifname[IFNAMSIZ] = {};
  std::snprintf(ifname, sizeof(ifname), "can%d", _num);

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
  if (ioctl(_socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    std::perror("ioctl SIOCGIFINDEX");
    close(_socket_fd);
    _socket_fd = -1;
    return;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(_socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::perror("bind");
    close(_socket_fd);
    _socket_fd = -1;
    return;
  }

  int flags = fcntl(_socket_fd, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK);
  }

  std::printf("CAN_twai initialized on %s\n", ifname);
}


void CAN_twai::CAN_Send(uint32_t* id_buf,uint8_t* buf) {
  if (_socket_fd < 0) {
    std::fprintf(stderr, "CAN_twai socket not initialized\n");
    return;
  }

  struct can_frame message{};
  message.can_id = id_buf ? (id_buf[0] & CAN_SFF_MASK) : 0;
  message.can_dlc = 8;
  if (buf) {
    std::memcpy(message.data, buf, 8);
  }

  if (write(_socket_fd, &message, sizeof(message)) == sizeof(message)) {
    std::printf("Message queued for transmission\n");
  } else {
    std::perror("write");
  }
}