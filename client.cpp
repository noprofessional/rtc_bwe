#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/socket.h>

#include <iostream>

#include "common.h"
#include "remote_bitrate_estimator.h"

using namespace std;

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

    int res = connect(udp_sock, (sockaddr*)&addr, sizeof(addr));
    if (res == -1)
    {
        cerr << "udp connect failed with" << strerror(errno) << endl;
        return 2;
    }

    const char* INIT = "INIT";
    send(udp_sock, INIT, strlen(INIT), 0);
    EstimatorPtr estimator = make_unique<RemoteBitrateEstimatorAbsSendTime>();
    char buf[1024 * 1024];
    while (true)
    {
        int len = recv(udp_sock, buf, sizeof(buf), 0);
        if (len == -1)
        {
            cerr << "udp connect failed with" << strerror(errno) << endl;
            return 2;
        }

        if (len < 3)
        {
            cerr << "udp msg invalid" << endl;
            return 2;
        }

        // ignore msg
        // just do statics
        uint32_t send_time_24bits = (((uint32_t)(uint8_t)(buf[len - 3])) << 16) |
                                    (((uint32_t)(uint8_t)(buf[len - 2])) << 8) |
                                    ((uint32_t)(uint8_t)buf[len - 1]);
        cout << send_time_24bits << endl;

        estimator->IncomingPacketInfo(getNowMs(), send_time_24bits, len, 0);
    }
}
