#pragma once

#include <stddef.h>
#include <stdint.h>

#include <list>
#include <map>
#include <memory>
#include <vector>

#include "inter_arrival.h"
#include "network_state_predictor.h"
#include "overuse_detector.h"
#include "overuse_estimator.h"
#include "rate_statics.h"

typedef uint64_t Timestamp;  // ms
typedef uint64_t DataRate;   // bps
typedef uint32_t DataSize;   // bytes
typedef int64_t TimeDelta;

struct Probe
{
    Probe(Timestamp send_time, Timestamp recv_time, DataSize payload_size)
        : send_time(send_time), recv_time(recv_time), payload_size(payload_size)
    {}

    Timestamp send_time;
    Timestamp recv_time;
    DataSize payload_size;
};

struct RateControlInput
{
    RateControlInput(BandwidthUsage bw_state_,
                     const optional<uint64_t>& estimated_throughput_)
        : bw_state(bw_state_), estimated_throughput(estimated_throughput_)
    {}

    BandwidthUsage bw_state;
    optional<uint64_t> estimated_throughput;
};

class AimdRateControl
{
public:
    AimdRateControl();
    // ms
    uint64_t GetFeedbackInterval() const;
    // Returns true if the bitrate estimate hasn't been changed for more than
    // an RTT, or if the estimated_throughput is less than half of the current
    // estimate. Should be used to decide if we should reduce the rate further
    // when over-using.
    bool TimeToReduceFurther(uint64_t at_time,
                             uint64_t estimated_throughput) const;

    bool ValidEstimate() const;
    DataRate LatestEstimate() const;

    uint64_t Update(const RateControlInput* input, uint64_t at_time);

    void SetRtt(TimeDelta rtt);

    void SetEstimate(DataRate bitrate, Timestamp at_time);

    DataRate ClampBitrate(DataRate new_bitrate) const;

private:
    Timestamp time_last_bitrate_change_;
    Timestamp time_first_throughput_estimate_;

    bool bitrate_is_initialized_;
    uint64_t current_bitrate_;
    TimeDelta rtt_;

    bool estimate_bounded_increase_;
    Timestamp time_last_bitrate_decrease_;

};

class RemoteBitrateEstimatorAbsSendTime
{
public:
    enum class ProbeResult
    {
        kBitrateUpdated,
        kNoUpdate
    };

    struct Cluster
    {
        DataRate SendBitrate() const { return mean_size / send_mean; }
        DataRate RecvBitrate() const { return mean_size / recv_mean; }

        TimeDelta send_mean = 0;
        TimeDelta recv_mean = 0;
        DataSize mean_size = 0;
        int count = 0;
        int num_above_min_delta = 0;
    };

public:
    RemoteBitrateEstimatorAbsSendTime();
    RTC_DISALLOW_COPY_AND_ASSIGN(RemoteBitrateEstimatorAbsSendTime);

    void IncomingPacketInfo(Timestamp arrival_time,
                            uint32_t send_time_24bits,
                            DataSize payload_size,
                            uint32_t ssrc);

    ProbeResult ProcessClusters(Timestamp now);
    std::list<Cluster> ComputeClusters() const;

    const Cluster* FindBestProbe(const std::list<Cluster>& clusters) const;

    bool IsBitrateImproving(DataRate probe_bitrate) const;

    bool IsWithinClusterBounds(TimeDelta send_delta,
                               const Cluster& cluster_aggregate) const;

    void MaybeAddCluster(const Cluster& cluster_aggregate,
                         std::list<Cluster>& clusters) const;

private:
    RateStatistics incoming_bitrate_;
    bool incoming_bitrate_initialized_;
    optional<uint64_t> first_packet_time_;
    optional<uint64_t> last_update_;
    AimdRateControl remote_rate_;

    size_t total_probes_received_;
    std::list<Probe> probes_;

    const FieldTrialBasedConfig field_trials_;

    std::unique_ptr<InterArrival> inter_arrival_;
    std::unique_ptr<OveruseEstimator> estimator_;
    OveruseDetector detector_;
};

typedef std::unique_ptr<RemoteBitrateEstimatorAbsSendTime> EstimatorPtr;
