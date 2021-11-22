#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>

#include <iostream>

#include "common.h"
#include "remote_bitrate_estimator.h"

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3)
        return 1;

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
    send(udp_sock, INIT, sizeof(INIT), 0);
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
        uint32_t send_time_24bits = (((uint32_t)(buf[len - 3])) << 16) |
                                    (((uint32_t)(buf[len - 2])) << 8) |
                                    ((uint32_t)buf[len - 1]);

        estimator->IncomingPacketInfo(getNowMs(), send_time_24bits, len, 0);
    }
}