#include <assert.h>

#include <iomanip>
#include <iostream>

using namespace std;

// 8.16 fixed second
inline void getAbsTime(uint8_t* buf)
{
    static struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    cout << "ts.tv_sec:" << hex << ts.tv_sec << endl;
    cout << "ts.tv_nsec:" << dec << ts.tv_nsec << endl;

    const double nanosec2fraction = 6.5536e-5;
    uint16_t fraction = ts.tv_nsec * nanosec2fraction;

    buf[0] = (uint8_t)ts.tv_sec;
    buf[1] = (fraction >> 8) & 0xff;
    buf[2] = fraction & 0xff;
}

int main(int argc, char** argv)
{
    uint8_t buf[3];
    getAbsTime(buf);
    for (int i = 0; i < 3; ++i)
    {
        cout << hex << setfill('0') << setw(2) << (uint32_t)buf[i] << endl;
    }
    return 0;
}
