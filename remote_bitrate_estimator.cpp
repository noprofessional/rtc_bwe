#include "remote_bitrate_estimator.h"

#include "logging.h"

constexpr int kTimestampGroupLengthMs = 5;
constexpr int kAbsSendTimeInterArrivalUpshift = 8;
constexpr int kAbsSendTimeFraction = 18;
constexpr int kInterArrivalShift = kAbsSendTimeFraction +
                                   kAbsSendTimeInterArrivalUpshift;
constexpr int kMinClusterSize = 4;
constexpr int kMaxProbePackets = 15;
constexpr int kExpectedNumberOfProbes = 3;
constexpr double kTimestampToMs = 1000.0 /
                                  static_cast<double>(1 << kInterArrivalShift);
constexpr int64_t kBitrateWindowMs = 1000;
constexpr uint32_t kMinProbePacketSize = 200;
constexpr uint64_t kInitializationTime = 5000;       // 5s = 5000ms
constexpr int64_t kMinClusterDelta = 1;              // 1ms
constexpr TimeDelta kDefaultRtt = 200;               // 200ms
constexpr TimeDelta kInitialProbingInterval = 2000;  // 2s = 2000ms

AimdRateControl::AimdRateControl()
    : time_last_bitrate_change_(UINT64_MAX),
      time_first_throughput_estimate_(UINT64_MAX),
      bitrate_is_initialized_(false),
      current_bitrate_(30000000),  // 30Mbps
      rtt_(kDefaultRtt)
{}
uint64_t AimdRateControl::GetFeedbackInterval() const
{
    // Estimate how often we can send RTCP if we allocate up to 5% of bandwidth
    // to feedback.
    const uint32_t kRtcpSize = 80;  // 80 bits? 10bytes
    const uint64_t rtcp_bitrate = current_bitrate_ * 0.05;
    const uint64_t interval = 1000 * kRtcpSize / rtcp_bitrate;  // s
    // const TimeDelta kMinFeedbackInterval = TimeDelta::Millis(200);
    // const TimeDelta kMaxFeedbackInterval = TimeDelta::Millis(1000);

    return clamp(interval, (uint64_t)200, (uint64_t)1000);
}

bool AimdRateControl::TimeToReduceFurther(uint64_t at_time,
                                          uint64_t estimated_throughput) const
{
    uint64_t bitrate_reduction_interval = clamp(rtt_, (TimeDelta)10,
                                                (TimeDelta)200);
    if (at_time - time_last_bitrate_change_ >= bitrate_reduction_interval)
    {
        return true;
    }
    if (ValidEstimate())
    {
        // TODO(terelius/holmer): Investigate consequences of increasing
        // the threshold to 0.95 * LatestEstimate().
        const DataRate threshold = 0.5 * LatestEstimate();
        return estimated_throughput < threshold;
    }
    return false;
}

DataRate AimdRateControl::LatestEstimate() const { return current_bitrate_; }

bool AimdRateControl::ValidEstimate() const { return bitrate_is_initialized_; }

uint64_t AimdRateControl::Update(const RateControlInput* input,
                                 uint64_t at_time)
{
    RTC_CHECK(input);

    // Set the initial bit rate value to what we're receiving the first half
    // second.
    // TODO(bugs.webrtc.org/9379): The comment above doesn't match to the code.
    if (!bitrate_is_initialized_)
    {
        RTC_DCHECK_LE(kBitrateWindowMs, kInitializationTime);
        if (!time_first_throughput_estimate_)
        {
            if (input->estimated_throughput)
                time_first_throughput_estimate_ = at_time;
        }
        else if (at_time - time_first_throughput_estimate_ >
                     kInitializationTime &&
                 input->estimated_throughput)
        {
            current_bitrate_ = *input->estimated_throughput;
            bitrate_is_initialized_ = true;
        }
    }

    // ChangeBitrate(*input, at_time);
    return current_bitrate_;
}

void AimdRateControl::SetRtt(TimeDelta rtt) { rtt_ = rtt; }

RemoteBitrateEstimatorAbsSendTime::RemoteBitrateEstimatorAbsSendTime()
    : incoming_bitrate_(kBitrateWindowMs, 8000),
      incoming_bitrate_initialized_(false),
      detector_(&field_trials_)
{}

void RemoteBitrateEstimatorAbsSendTime::IncomingPacketInfo(
    Timestamp arrival_time,
    uint32_t send_time_24bits,
    DataSize payload_size,
    uint32_t ssrc)
{
    (void)(ssrc);
    RTC_CHECK(send_time_24bits < (1ul << 24));
    /*
    if (!uma_recorded_)
    {
        RTC_HISTOGRAM_ENUMERATION(kBweTypeHistogram,
                                  BweNames::kReceiverAbsSendTime,
                                  BweNames::kBweNamesMax);
        uma_recorded_ = true;
    }
    */
    //  Shift up send time to use the full 32 bits that inter_arrival works
    //  with, so wrapping works properly.
    uint32_t timestamp = send_time_24bits << kAbsSendTimeInterArrivalUpshift;
    Timestamp send_time = (static_cast<int64_t>(timestamp) * kTimestampToMs);

    Timestamp now = getNowMs();
    // TODO(holmer): SSRCs are only needed for REMB, should be broken out from
    // here.

    // Check if incoming bitrate estimate is valid, and if it needs to be reset.
    optional<int64_t> incoming_bitrate = incoming_bitrate_.Rate(arrival_time);
    if (incoming_bitrate)
    {
        incoming_bitrate_initialized_ = true;
    }
    else if (incoming_bitrate_initialized_)
    {
        // Incoming bitrate had a previous valid value, but now not enough data
        // point are left within the current window. Reset incoming bitrate
        // estimator so that the window size will only contain new data points.
        incoming_bitrate_.Reset();
        incoming_bitrate_initialized_ = false;
    }
    incoming_bitrate_.Update(payload_size, arrival_time);

    if (!first_packet_time_)
    {
        first_packet_time_ = now;
    }

    uint32_t ts_delta = 0;
    int64_t t_delta = 0;
    int size_delta = 0;
    bool update_estimate = false;
    DataRate target_bitrate = 0;
    std::vector<uint32_t> ssrcs;
    {
        // MutexLock lock(&mutex_);

        // TimeoutStreams(now);
        RTC_DCHECK(inter_arrival_);
        RTC_DCHECK(estimator_);
        // TODO(danilchap): Replace 5 lines below with insert_or_assign when
        // that c++17 function is available.
        /*
        auto inserted = ssrcs_.insert(std::make_pair(ssrc, now));
        if (!inserted.second)
        {
            // Already inserted, update.
            inserted.first->second = now;
        }
        */

        // For now only try to detect probes while we don't have a valid
        // estimate. We currently assume that only packets larger than 200 bytes
        // are paced by the sender.
        if (payload_size > kMinProbePacketSize &&
            (!remote_rate_.ValidEstimate() ||
             now - first_packet_time_ < kInitialProbingInterval))
        {
            // TODO(holmer): Use a map instead to get correct order?
            if (total_probes_received_ < kMaxProbePackets)
            {
                TimeDelta send_delta = -1;
                TimeDelta recv_delta = -1;
                if (!probes_.empty())
                {
                    send_delta = send_time - probes_.back().send_time;
                    recv_delta = arrival_time - probes_.back().recv_time;
                }
                RTC_LOG(LS_INFO)
                    << "Probe packet received: send time=" << send_time
                    << " ms, recv time=" << arrival_time
                    << " ms, send delta=" << send_delta
                    << " ms, recv delta=" << recv_delta << " ms.";
            }
            probes_.emplace_back(send_time, arrival_time, payload_size);
            ++total_probes_received_;
            // Make sure that a probe which updated the bitrate immediately has
            // an effect by calling the OnReceiveBitrateChanged callback.
            if (ProcessClusters(now) == ProbeResult::kBitrateUpdated)
                update_estimate = true;
        }
        if (inter_arrival_->ComputeDeltas(timestamp, arrival_time, now,
                                          payload_size, &ts_delta, &t_delta,
                                          &size_delta))
        {
            double ts_delta_ms = (1000.0 * ts_delta) /
                                 (1 << kInterArrivalShift);
            estimator_->Update(t_delta, ts_delta_ms, size_delta,
                               detector_.State(), arrival_time);
            detector_.Detect(estimator_->offset(), ts_delta_ms,
                             estimator_->num_of_deltas(), arrival_time);
        }

        if (!update_estimate)
        {
            // Check if it's time for a periodic update or if we should update
            // because of an over-use.
            if (!last_update_ ||
                now - last_update_ > remote_rate_.GetFeedbackInterval())
            {
                update_estimate = true;
            }
            else if (detector_.State() == BandwidthUsage::kBwOverusing)
            {
                optional<uint32_t> incoming_rate = incoming_bitrate_.Rate(
                    arrival_time);
                if (incoming_rate &&
                    remote_rate_.TimeToReduceFurther(now, *incoming_rate))
                {
                    update_estimate = true;
                }
            }
        }

        if (update_estimate)
        {
            // The first overuse should immediately trigger a new estimate.
            // We also have to update the estimate immediately if we are
            // overusing and the target bitrate is too high compared to what we
            // are receiving.
            const RateControlInput input(detector_.State(),
                                         incoming_bitrate_.Rate(arrival_time));
            target_bitrate = remote_rate_.Update(&input, now);
            update_estimate = remote_rate_.ValidEstimate();
        }
    }
    if (update_estimate)
    {
        last_update_ = now;
        std::cout << "new bitrate:" << target_bitrate << std::endl;
    }
}

RemoteBitrateEstimatorAbsSendTime::ProbeResult
RemoteBitrateEstimatorAbsSendTime::ProcessClusters(Timestamp now)
{
    std::list<Cluster> clusters = ComputeClusters();
    if (clusters.empty())
    {
        // If we reach the max number of probe packets and still have no
        // clusters, we will remove the oldest one.
        if (probes_.size() >= kMaxProbePackets)
            probes_.pop_front();
        return ProbeResult::kNoUpdate;
    }

    if (const Cluster* best = FindBestProbe(clusters))
    {
        DataRate probe_bitrate = std::min(best->SendBitrate(),
                                          best->RecvBitrate());
        // Make sure that a probe sent on a lower bitrate than our estimate
        // can't reduce the estimate.
        if (IsBitrateImproving(probe_bitrate))
        {
            RTC_LOG(LS_INFO)
                << "Probe successful, sent at " << best->SendBitrate()
                << " bps, received at " << best->RecvBitrate()
                << " bps. Mean send delta: " << best->send_mean
                << " ms, mean recv delta: " << best->recv_mean
                << " ms, num probes: " << best->count;
            remote_rate_.SetEstimate(probe_bitrate, now);
            return ProbeResult::kBitrateUpdated;
        }
    }

    // Not probing and received non-probe packet, or finished with current set
    // of probes.
    if (clusters.size() >= kExpectedNumberOfProbes)
        probes_.clear();
    return ProbeResult::kNoUpdate;
}

std::list<RemoteBitrateEstimatorAbsSendTime::Cluster>
RemoteBitrateEstimatorAbsSendTime::ComputeClusters() const
{
    std::list<Cluster> clusters;
    Cluster cluster_aggregate;
    Timestamp prev_send_time = UINT64_MAX;
    Timestamp prev_recv_time = UINT64_MAX;
    for (const Probe& probe : probes_)
    {
        if (prev_send_time != UINT64_MAX)
        {
            TimeDelta send_delta = probe.send_time - prev_send_time;
            TimeDelta recv_delta = probe.recv_time - prev_recv_time;
            if (send_delta >= kMinClusterDelta &&
                recv_delta >= kMinClusterDelta)
            {
                ++cluster_aggregate.num_above_min_delta;
            }
            if (!IsWithinClusterBounds(send_delta, cluster_aggregate))
            {
                MaybeAddCluster(cluster_aggregate, clusters);
                cluster_aggregate = Cluster();
            }
            cluster_aggregate.send_mean += send_delta;
            cluster_aggregate.recv_mean += recv_delta;
            cluster_aggregate.mean_size += probe.payload_size;
            ++cluster_aggregate.count;
        }
        prev_send_time = probe.send_time;
        prev_recv_time = probe.recv_time;
    }
    MaybeAddCluster(cluster_aggregate, clusters);
    return clusters;
}

bool RemoteBitrateEstimatorAbsSendTime::IsWithinClusterBounds(
    TimeDelta send_delta, const Cluster& cluster_aggregate) const
{
    if (cluster_aggregate.count == 0)
        return true;
    TimeDelta cluster_mean = cluster_aggregate.send_mean /
                             cluster_aggregate.count;
    return abs(send_delta - cluster_mean) < 2.5;  // TimeDelta::Micros(2'500);
}

void RemoteBitrateEstimatorAbsSendTime::MaybeAddCluster(
    const Cluster& cluster_aggregate, std::list<Cluster>& clusters) const
{
    if (cluster_aggregate.count < kMinClusterSize ||
        cluster_aggregate.send_mean <= 0 || cluster_aggregate.recv_mean <= 0)
    {
        return;
    }

    Cluster cluster;
    cluster.send_mean = cluster_aggregate.send_mean / cluster_aggregate.count;
    cluster.recv_mean = cluster_aggregate.recv_mean / cluster_aggregate.count;
    cluster.mean_size = cluster_aggregate.mean_size / cluster_aggregate.count;
    cluster.count = cluster_aggregate.count;
    cluster.num_above_min_delta = cluster_aggregate.num_above_min_delta;
    clusters.push_back(cluster);
}

const RemoteBitrateEstimatorAbsSendTime::Cluster*
RemoteBitrateEstimatorAbsSendTime::FindBestProbe(
    const std::list<Cluster>& clusters) const
{
    DataRate highest_probe_bitrate = 0;
    const Cluster* best = nullptr;
    for (const auto& cluster : clusters)
    {
        if (cluster.send_mean == 0 || cluster.recv_mean == 0)
        {
            continue;
        }
        if (cluster.num_above_min_delta > cluster.count / 2 &&
            (cluster.recv_mean - cluster.send_mean <= 2 /*TimeDelta::Millis(2)*/
             && cluster.send_mean - cluster.recv_mean <=
                    5 /*TimeDelta::Millis(5)*/))
        {
            DataRate probe_bitrate = std::min(cluster.SendBitrate(),
                                              cluster.RecvBitrate());
            if (probe_bitrate > highest_probe_bitrate)
            {
                highest_probe_bitrate = probe_bitrate;
                best = &cluster;
            }
        }
        else
        {
            RTC_LOG(LS_INFO)
                << "Probe failed, sent at " << cluster.SendBitrate()
                << " bps, received at " << cluster.RecvBitrate()
                << " bps. Mean send delta: " << cluster.send_mean
                << " ms, mean recv delta: " << cluster.recv_mean
                << " ms, num probes: " << cluster.count;
            break;
        }
    }
    return best;
}

bool RemoteBitrateEstimatorAbsSendTime::IsBitrateImproving(
    DataRate probe_bitrate) const
{
    bool initial_probe = !remote_rate_.ValidEstimate() && probe_bitrate > 0;
    bool bitrate_above_estimate = remote_rate_.ValidEstimate() &&
                                  probe_bitrate > remote_rate_.LatestEstimate();
    return initial_probe || bitrate_above_estimate;
}

void AimdRateControl::SetEstimate(DataRate bitrate, Timestamp at_time)
{
    bitrate_is_initialized_ = true;
    DataRate prev_bitrate = current_bitrate_;
    current_bitrate_ = ClampBitrate(bitrate);
    time_last_bitrate_change_ = at_time;
    if (current_bitrate_ < prev_bitrate)
    {
        time_last_bitrate_decrease_ = at_time;
    }
}

DataRate AimdRateControl::ClampBitrate(DataRate new_bitrate) const
{
    return new_bitrate;
}
