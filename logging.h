#pragma once

#include <iostream>
#include <sstream>

#define RTC_LOG(x) RTC_LOG_##x

class Logger
{
public:
    Logger(int level) : level_(level) {}
    template<class T>
    Logger& operator<<(const T& val)
    {
        os_ << val;
        return *this;
    }
    ~Logger()
    {
        if (level_ > 2)
            std::cout << os_.str() << std::endl;
        else
            std::cerr << os_.str() << std::endl;
    }
    char level_;
    std::ostringstream os_;
};
#define RTC_LOG_LS_WARNING Logger(1)
#define RTC_LOG_LS_ERROR Logger(2)
#define RTC_LOG_LS_INFO Logger(3)
