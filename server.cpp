#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <iostream>

using namespace std;

const uint64_t datarate = 4000000;  // 4Mbps
const uint32_t fps = 30;            // fps
const uint32_t scale = 3;           // I frame size scale

// 8.16 fixed second
inline void getAbsTime(uint8_t* buf)
{
    static struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    const double nanosec2fraction = 6.5536e-5;
    uint16_t fraction = ts.tv_nsec * nanosec2fraction;

    buf[0] = (uint8_t)ts.tv_sec;
    buf[1] = (fraction >> 8) & 0xff;
    buf[2] = fraction & 0xff;
}

int main(int argc, char** argv)
{
    if (argc < 3)
        return 1;

    srand(time(nullptr));

    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock == -1)
    {
        cerr << "udp socket create failed with" << strerror(errno) << endl;
        return 1;
    }

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(argv[1]);
    addr.sin_port = htons(atoi(argv[2]));

    int res = bind(udp_sock, (sockaddr*)&addr, sizeof(addr));
    if (res == -1)
    {
        cerr << "udp listen failed with" << strerror(errno) << endl;
        return 2;
    }

    socklen_t socklen = sizeof(addr);
    int len = 0;
    char buf[1024 * 1024];
    while (true)
    {
        len = recvfrom(udp_sock, buf, sizeof(buf), 0, (sockaddr*)&addr,
                       &socklen);
        if (len == -1)
        {
            cerr << "udp recv failed with" << strerror(errno) << endl;
            return 2;
        }

        if (len != 4 || (memcmp(buf, "INIT", 4) != 0))
        {
            cerr << "udp msg invalid" << endl;
            return 2;
        }

        res = connect(udp_sock, (sockaddr*)&addr, sizeof(addr));
        if (res != 0)
        {
            cerr << "udp connect failed." << endl;
            return 3;
        }

        len = (rand() % 100) + 1000;
        getAbsTime((uint8_t*)&buf[len - 3]);
        send(udp_sock, buf, len, 0);

        uint32_t sleepms = 42;
        usleep(sleepms*1000);
    }
}
