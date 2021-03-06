#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <iostream>

using namespace std;

const uint64_t datarate = 4000000;  // 4Mbps
const uint32_t fps = 30;            // fps
const uint32_t scale = 3;           // I frame size scale

// 6.18 fixed second
inline void getAbsTime(uint8_t* buf)
{
    static struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    const double nanosec2fraction = (double)(1 << 18) / 1000000000;
    uint32_t fraction = ts.tv_nsec * nanosec2fraction;

    buf[0] = (((uint8_t)ts.tv_sec) & 0x3f) << 2 | ((fraction >> 16) & 0x03);
    buf[1] = (fraction >> 8) & 0xff;
    buf[2] = fraction & 0xff;
}

void enable_core_dump()
{
    struct rlimit flimit;
    flimit.rlim_cur = RLIM_INFINITY;
    flimit.rlim_max = RLIM_INFINITY;

    if (setrlimit(RLIMIT_CORE, &flimit) < 0)
    {
        fprintf(stderr, "setrlimit2 error\n");
        exit(1);
    };

    prctl(PR_SET_DUMPABLE, 1);
}

int main(int argc, char** argv)
{
    if (argc < 3)
        return 1;

    enable_core_dump();

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
    bool connected = false;
    int len = 0;
    char buf[1024 * 1024];
    while (true)
    {
        if (!connected)
        {
            len = recvfrom(udp_sock, buf, sizeof(buf), 0, (sockaddr*)&addr,
                           &socklen);
            if (len == -1)
            {
                cerr << "udp recv failed with" << strerror(errno) << endl;
                continue;
            }

            if (len != 4 || (memcmp(buf, "INIT", 4) != 0))
            {
                cerr << "udp msg invalid" << endl;
                continue;
            }

            cout << "new conn from:" << inet_ntoa(addr.sin_addr) << ":"
                 << ntohs(addr.sin_port) << endl;
            res = connect(udp_sock, (sockaddr*)&addr, sizeof(addr));
            if (res != 0)
            {
                cerr << "udp connect failed." << endl;
                continue;
            }
            connected = true;
        }

        len = (rand() % 100) + 1000;
        getAbsTime((uint8_t*)&buf[len - 3]);
        send(udp_sock, buf, len, 0);

        uint32_t sleepms = 42;
        usleep(sleepms * 1000);
    }
}
