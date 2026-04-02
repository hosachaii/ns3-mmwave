/*
 * tcp-mrvhs-opt.cc
 *
 * Optimized MRVHS congestion control implementation.
 * See tcp-mrvhs-opt.h for full description of the three optimisations.
 *
 * NOTE: This file is a modification / extension of tcp-mrvhs.cc which is
 *       itself a modification of tcp-highspeed.cc.
 */

#include "tcp-mrvhs-opt.h"
#include "tcp-mrvhs.h" 
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpMrvhsOpt");
NS_OBJECT_ENSURE_REGISTERED(TcpMrvhsOpt);

TypeId
TcpMrvhsOpt::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TcpMrvhsOpt")
            .SetParent<TcpMrvhs>()
            .AddConstructor<TcpMrvhsOpt>()
            .SetGroupName("Internet")
            // -----------------------------------------------------------
            // Opt-1: cwnd growth formula selector
            // -----------------------------------------------------------
            .AddAttribute(
                "CwndFormula",
                "Cwnd growth formula variant: "
                "\"sqrt\" (original paper: a=sqrt(MSS*TimeRatio)), "
                "\"linear\" (a=(MSS*TimeRatio)/cwnd), "
                "\"logdamp\" (a=sqrt(MSS)*log(TimeRatio+1))",
                StringValue("sqrt"),
                MakeStringAccessor(&TcpMrvhsOpt::m_cwndFormula),
                MakeStringChecker())
            // -----------------------------------------------------------
            // Opt-2: adaptive beta
            // -----------------------------------------------------------
            .AddAttribute(
                "AdaptiveBeta",
                "Enable adaptive β decrease factor (scales HighSpeed table β "
                "by RTT inflation ratio so congested flows cut more aggressively)",
                BooleanValue(false),          // off → exact paper behaviour
                MakeBooleanAccessor(&TcpMrvhsOpt::m_adaptiveBeta),
                MakeBooleanChecker())
            .AddAttribute(
                "BetaMin",
                "Floor for the adaptive β scaling multiplier [0,1]",
                DoubleValue(0.5),
                MakeDoubleAccessor(&TcpMrvhsOpt::m_betaMin),
                MakeDoubleChecker<double>(0.0, 1.0))
            // -----------------------------------------------------------
            // Opt-3: ECN early reaction
            // -----------------------------------------------------------
            .AddAttribute(
                "EcnReaction",
                "Enable gentle multiplicative cwnd decrease on ECN CE marks "
                "(before actual packet loss occurs)",
                BooleanValue(false),          // off → exact paper behaviour
                MakeBooleanAccessor(&TcpMrvhsOpt::m_ecnReaction),
                MakeBooleanChecker())
            .AddAttribute(
                "EcnDecrFactor",
                "Multiplicative decrease factor applied on ECN CE event (0 < f < 1). "
                "Default 0.875 (= 7/8) is intentionally mild.",
                DoubleValue(0.875),
                MakeDoubleAccessor(&TcpMrvhsOpt::m_ecnDecrFactor),
                MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

TcpMrvhsOpt::TcpMrvhsOpt()
    : TcpMrvhs(),
      m_cwndFormula("sqrt"),
      m_adaptiveBeta(false),
      m_betaMin(0.5),
      m_ecnReaction(false),
      m_ecnDecrFactor(0.875)
{
    m_rttBase = Time(0);
    m_rttMax  = Time(0);
}

TcpMrvhsOpt::TcpMrvhsOpt(const TcpMrvhsOpt& sock)
    : TcpMrvhs(sock),
      m_cwndFormula(sock.m_cwndFormula),
      m_adaptiveBeta(sock.m_adaptiveBeta),
      m_betaMin(sock.m_betaMin),
      m_ecnReaction(sock.m_ecnReaction),
      m_ecnDecrFactor(sock.m_ecnDecrFactor),
      m_rttBase(sock.m_rttBase),
      m_rttMax(sock.m_rttMax)
{
}

TcpMrvhsOpt::~TcpMrvhsOpt()
{
    NS_LOG_FUNCTION(this);
}

Ptr<TcpCongestionOps>
TcpMrvhsOpt::Fork()
{
    return CopyObject<TcpMrvhsOpt>(this);
}

std::string
TcpMrvhsOpt::GetName() const
{
    return "TcpMrvhsOpt";
}

// ---------------------------------------------------------------------------
// Opt-1: pluggable 'a' computation
// ---------------------------------------------------------------------------
double
TcpMrvhsOpt::ComputeA(double mss, double timeRatio, double segCwnd) const
{
    if (m_cwndFormula == "linear")
    {
        // a = (MSS * TimeRatio) / cwnd
        // Growth is proportional to MSS*TimeRatio but dampened by the window
        // size — avoids the explosive growth sqrt can produce at large cwnd.
        if (segCwnd < 1.0)
            segCwnd = 1.0;
        return (mss * timeRatio) / segCwnd;
    }
    else if (m_cwndFormula == "logdamp")
    {
        // a = sqrt(MSS) * log(TimeRatio + 1)
        // Logarithm clamps the boost at very high RTT ratios, providing more
        // conservative growth when RTT_max >> RTT_cur (deep congestion proxy).
        return std::sqrt(mss) * std::log(timeRatio + 1.0) * std::sqrt(timeRatio);
    }
    else
    {
        // "sqrt" — original paper formula: a = sqrt(MSS * TimeRatio)
        return std::sqrt(mss * timeRatio);
    }
}

// ---------------------------------------------------------------------------
// Opt-1 (growth formula) + RTT tracking override of CongestionAvoidance
// ---------------------------------------------------------------------------
void
TcpMrvhsOpt::CongestionAvoidance(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked);

    if (segmentsAcked == 0)
        return;

    Time rtt = tcb->m_lastRtt;
    if (rtt.IsZero())
        return;

    // Update RTT history — duplicated from base class because the base class
    // private members are not accessible here.
    if (m_rttBase.IsZero() || rtt < m_rttBase)
        m_rttBase = rtt;
    if (rtt > m_rttMax)
        m_rttMax = rtt;

    // Compute TimeRatio  (Eq. 1 from paper)
    double timeRatio =
        (m_rttMax.GetSeconds() + m_rttBase.GetSeconds()) / rtt.GetSeconds();

    double mss     = static_cast<double>(tcb->m_segmentSize);
    double segCwnd = static_cast<double>(tcb->m_cWnd) / mss;
    if (segCwnd < 1.0)
        segCwnd = 1.0;

    // Opt-1: choose formula
    double a = ComputeA(mss, timeRatio, segCwnd);

    // Update cwnd  (Eq. 5 from paper: cwnd = cwnd + a/cwnd)
    segCwnd += a / segCwnd;

    tcb->m_cWnd = static_cast<uint32_t>(std::round(segCwnd * mss));

    NS_LOG_INFO("MRVHSOpt [" << m_cwndFormula << "] cwnd=" << tcb->m_cWnd
                << " a=" << a << " TimeRatio=" << timeRatio);
}

// ---------------------------------------------------------------------------
// Opt-2: adaptive β  — override GetSsThresh
// ---------------------------------------------------------------------------
uint32_t
TcpMrvhsOpt::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
    double segCwnd = static_cast<double>(tcb->m_cWnd) /
                     static_cast<double>(tcb->m_segmentSize);

    // Base β from HighSpeed table (inherited from TcpMrvhs)
    double b = TcpMrvhs::TableLookupB(static_cast<uint32_t>(segCwnd));

    if (m_adaptiveBeta && !m_rttMax.IsZero())
    {
        // RTT inflation ratio: how much has RTT grown above baseline?
        // Higher ratio → more congestion → reduce more aggressively.
        // rttInflation ∈ [0, 1]; at full inflation (RTTcur == RTTmax) = 1.0.
        double rttCur  = tcb->m_lastRtt.Get().GetSeconds();
        double rttMax  = m_rttMax.GetSeconds();
        // The inflation ratio should be how far RTTcur is FROM RTTbase, not relative to RTTmax
        // High inflate = congested = bigger b = more backoff
        double rttBase = m_rttBase.GetSeconds();
        double inflate = (rttMax > rttBase)
                         ? std::min((rttCur - rttBase) / (rttMax - rttBase), 1.0)
                         : 0.0;
        // More inflation → scale closer to 1 (use full table beta)
        // Less inflation → scale down beta (back off less when not congested)
        double scale = m_betaMin + (1.0 - m_betaMin) * inflate;
        b = b * scale;
        NS_LOG_INFO("MRVHSOpt AdaptiveBeta: b_table=" << TcpMrvhs::TableLookupB(
                        static_cast<uint32_t>(segCwnd))
                    << " scale=" << scale << " b_eff=" << b
                    << " rttInflate=" << inflate);
    }

    uint32_t ssThresh = static_cast<uint32_t>(
        std::max(2.0, segCwnd * (1.0 - b)) * tcb->m_segmentSize);

    return ssThresh;
}

// ---------------------------------------------------------------------------
// Opt-3: ECN CE early reaction
// ---------------------------------------------------------------------------
void
TcpMrvhsOpt::CwndEvent(Ptr<TcpSocketState> tcb,
                        const TcpSocketState::TcpCAEvent_t event)
{
    // Forward all events to the parent first so normal HighSpeed/MRVHS
    // bookkeeping (e.g. CA_EVENT_TX_START) is preserved.
    TcpMrvhs::CwndEvent(tcb, event);

    if (!m_ecnReaction)
        return;

    // CA_EVENT_ECN_IS_CE fires when an ECN Congestion Experienced mark is
    // received — i.e. a router signalled congestion *before* dropping packets.
    if (event == TcpSocketState::CA_EVENT_ECN_IS_CE)
    {
        // Apply a mild multiplicative decrease.  We deliberately do NOT call
        // GetSsThresh here — ECN is an early warning, not a full loss event.
        // The factor 0.875 (=7/8) matches DCTCP's default.
        double newCwnd = static_cast<double>(tcb->m_cWnd) * m_ecnDecrFactor;

        // Never reduce below 2 segments
        uint32_t minCwnd = 2 * tcb->m_segmentSize;
        tcb->m_cWnd      = std::max(static_cast<uint32_t>(newCwnd), minCwnd);

        NS_LOG_INFO("MRVHSOpt ECN CE: cwnd reduced by factor "
                    << m_ecnDecrFactor << " → " << tcb->m_cWnd);
    }
}

} // namespace ns3
