/*
 * tcp-mrvhs-opt.h
 *
 * Optimized MRVHS congestion control — extends the original TcpMrvhs with:
 *
 *   Opt-1  Pluggable cwnd-growth formula  (--cwndFormula=sqrt | linear | logdamp)
 *          sqrt    : a = sqrt(MSS * TimeRatio)          [original paper]
 *          linear  : a = (MSS * TimeRatio) / cwnd       [linear in cwnd]
 *          logdamp : a = sqrt(MSS) * log(TimeRatio + 1) [log dampening at high RTT]
 *
 *   Opt-2  Adaptive β (decrease factor)
 *          Instead of the fixed HighSpeed table lookup for b, β is scaled
 *          by the observed PER proxy (RTT inflation ratio).  Flows with heavy
 *          congestion back off harder; lightly-loaded flows back off less.
 *          β_adaptive = β_table * clamp(RTTcur / RTTmax, β_min, 1.0)
 *
 *   Opt-3  ECN-aware early reaction
 *          When an ECN CE mark is received (before actual packet loss), MRVHS
 *          applies a mild multiplicative decrease (factor = 0.875, i.e. 7/8)
 *          rather than waiting for 3DACKs / RTO.  This reduces latency spikes
 *          in lightly-loaded scenarios (addresses the null/small-PER latency
 *          regression noted in Table 3 of the paper).
 *
 * Compile: drop tcp-mrvhs-opt.cc + tcp-mrvhs-opt.h alongside the originals
 *          and add them to your CMakeLists / wscript exactly as the originals.
 *
 * Usage in simulation:
 *   Config::SetDefault("ns3::TcpL4Protocol::SocketType",
 *                      TypeIdValue(TcpMrvhsOpt::GetTypeId()));
 *   // optionally tune:
 *   Config::SetDefault("ns3::TcpMrvhsOpt::CwndFormula", StringValue("logdamp"));
 *   Config::SetDefault("ns3::TcpMrvhsOpt::AdaptiveBeta", BooleanValue(true));
 *   Config::SetDefault("ns3::TcpMrvhsOpt::EcnReaction",  BooleanValue(true));
 */

#ifndef TCP_MRVHS_OPT_H
#define TCP_MRVHS_OPT_H

#include "tcp-mrvhs.h"          // inherits from original MRVHS
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/double.h"

namespace ns3
{

/**
 * \ingroup congestionOps
 *
 * \brief Optimized MRVHS TCP congestion control.
 *
 * Adds three independently toggleable optimisations over the base MRVHS:
 *   - Pluggable cwnd growth formula (Opt-1)
 *   - Adaptive β decrease factor    (Opt-2)
 *   - ECN-aware early reaction       (Opt-3)
 *
 * All three default to the original paper behaviour so that disabling all
 * flags reproduces the published results exactly.
 */
class TcpMrvhsOpt : public TcpMrvhs
{
  public:
    static TypeId GetTypeId();

    TcpMrvhsOpt();
    TcpMrvhsOpt(const TcpMrvhsOpt& sock);
    ~TcpMrvhsOpt() override;

    std::string GetName() const override;
    Ptr<TcpCongestionOps> Fork() override;

    // -----------------------------------------------------------------------
    // Opt-2: override GetSsThresh to implement adaptive β
    // -----------------------------------------------------------------------
    uint32_t GetSsThresh(Ptr<const TcpSocketState> tcb,
                         uint32_t bytesInFlight) override;

    // -----------------------------------------------------------------------
    // Opt-3: ECN reaction hook
    //        ns-3 calls CwndEvent with CA_EVENT_ECN_IS_CE when a CE mark
    //        arrives.  We apply a gentle multiplicative decrease here instead
    //        of waiting for loss.
    // -----------------------------------------------------------------------
    void CwndEvent(Ptr<TcpSocketState> tcb,
                   const TcpSocketState::TcpCAEvent_t event) override;

  protected:
    // -----------------------------------------------------------------------
    // Opt-1 + partial Opt-2: override CongestionAvoidance for the growth formula
    // -----------------------------------------------------------------------
    void CongestionAvoidance(Ptr<TcpSocketState> tcb,
                             uint32_t segmentsAcked) override;

  private:
    // --- Opt-1 attributes ---
    std::string m_cwndFormula; ///< "sqrt" | "linear" | "logdamp"

    // --- Opt-2 attributes ---
    bool   m_adaptiveBeta;    ///< enable adaptive β
    double m_betaMin;         ///< floor for the adaptive β scaling factor

    // --- Opt-3 attributes ---
    bool   m_ecnReaction;     ///< enable ECN early reaction
    double m_ecnDecrFactor;   ///< multiplicative decrease on ECN CE (default 0.875)

    // --- internal RTT tracking (mirrors base class but kept here for clarity) ---

    // --- helper ---
    /**
     * Compute the 'a' coefficient according to the selected formula.
     * \param mss        segment size in bytes (double)
     * \param timeRatio  (RTTmax + RTTbase) / RTTcur
     * \param segCwnd    current cwnd in segments (used only by "linear")
     * \return  increment coefficient a
     */
    Time m_rttBase;
    Time m_rttMax;
    double ComputeA(double mss, double timeRatio, double segCwnd) const;
};

} // namespace ns3

#endif // TCP_MRVHS_OPT_H
