#ifndef SOCKET_CAN_HPP
#define SOCKET_CAN_HPP

#include <string>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace arm_hardware_interface
{
    class SocketCan
    {
    public:
        SocketCan() : socket_fd_(-1) {}
        ~SocketCan() { close_socket(); }

        bool open(const std::string& interface_name)
        {
            socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (socket_fd_ < 0) return false;

            struct ifreq ifr;
            std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ);
            if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

            struct sockaddr_can addr;
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) return false;

            // Set non-blocking
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 1000; // 1ms timeout for safety
            setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

            return true;
        }

        void close_socket()
        {
            if (socket_fd_ >= 0) {
                close(socket_fd_);
                socket_fd_ = -1;
            }
        }

        bool write_frame(uint32_t can_id, const uint8_t data[8], bool is_extended = false)
        {
            struct can_frame frame;
            frame.can_id = can_id;
            if (is_extended) frame.can_id |= CAN_EFF_FLAG;
            frame.can_dlc = 8;
            std::memcpy(frame.data, data, 8);

            return write(socket_fd_, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
        }

        int read_frame(struct can_frame &frame)
        {
            return read(socket_fd_, &frame, sizeof(struct can_frame));
        }

    private:
        int socket_fd_;
    };
}

#endif
