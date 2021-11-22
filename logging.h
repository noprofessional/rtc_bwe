#pragma once

#include <iostream>

#define RTC_LOG(x) RTC_LOG_##x

#define RTC_LOG_LS_WARNING std::cerr
#define RTC_LOG_LS_ERROR std::cerr
#define RTC_LOG_LS_INFO std::cout

