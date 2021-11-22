#pragma once
#include <assert.h>

#include <limits>
#include <map>
#define RTC_CHECK(x) assert(x)
#define RTC_DCHECK(x) assert(x)
#define RTC_DCHECK_GE(x, y) assert((x) > (y))
#define RTC_DCHECK_LE(x, y) assert((x) < (y))

template<typename U>
inline bool IsNewer(U value, U prev_value)
{
    static_assert(!std::numeric_limits<U>::is_signed, "U must be unsigned");
    // kBreakpoint is the half-way mark for the type U. For instance, for a
    // uint16_t it will be 0x8000, and for a uint32_t, it will be 0x8000000.
    constexpr U kBreakpoint = (std::numeric_limits<U>::max() >> 1) + 1;
    // Distinguish between elements that are exactly kBreakpoint apart.
    // If t1>t2 and |t1-t2| = kBreakpoint: IsNewer(t1,t2)=true,
    // IsNewer(t2,t1)=false
    // rather than having IsNewer(t1,t2) = IsNewer(t2,t1) = false.
    if (value - prev_value == kBreakpoint)
    {
        return value > prev_value;
    }
    return value != prev_value &&
           static_cast<U>(value - prev_value) < kBreakpoint;
}

inline uint32_t LatestTimestamp(uint32_t timestamp1, uint32_t timestamp2)
{
    return IsNewer(timestamp1, timestamp2) ? timestamp1 : timestamp2;
}

template<class T>
class optional
{
    T val;
    bool hasValue;

public:
    optional() : val(0), hasValue(false) {}
    optional(T val_) : val(val_), hasValue(false) {}
    T operator*() const
    {
        assert(hasValue);
        return val;
    }
    operator bool() const { return hasValue; }
    void clear() { hasValue = false; }
    void operator=(T val_)
    {
        val = val_;
        hasValue = true;
    }

    template<class U>
    optional(const optional<U>& other)
    {
        if (other)
        {
            val = static_cast<T>(*other);
            hasValue = true;
        }
        else
        {
            val = 0;
            hasValue = false;
        }
    }
};

template<typename Dst, typename Src>
inline constexpr Dst dchecked_cast(Src value)
{
    return static_cast<Dst>(value);
}

#define RTC_DISALLOW_DEFAULT_CTOR(x) x() = delete

#define RTC_DISALLOW_COPY_AND_ASSIGN(x) \
    x(const x& rhs) = delete;           \
    x& operator=(const x& rhs) = delete

template<class T>
T clamp(T ori, T min, T max)
{
    if (ori > max)
        return max;
    else if (ori < min)
        return min;
    else
        return ori;
}

inline uint64_t getNowMs()
{
    static struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1000 * static_cast<int64_t>(ts.tv_sec) +
           static_cast<int64_t>(ts.tv_nsec) / 1000000;
}

class WebRtcKeyValueConfig
{
public:
    virtual ~WebRtcKeyValueConfig() = default;
    // The configured value for the given key. Defaults to an empty string.
    virtual std::string Lookup(const std::string& key) const = 0;
};

class FieldTrialBasedConfig : public WebRtcKeyValueConfig
{
public:
    FieldTrialBasedConfig() : dumb(0) {}
    std::string Lookup(const std::string& key) const override
    {
        static std::map<std::string, std::string> dict{};
        auto iter = dict.find(key);
        if (iter != dict.end())
            return iter->second;
        else
            return {};
    }
    char dumb;
};
