// NOTE: tcp-mrvhs.cc is a modification of the tcp-highspeed.cc file

/*
 * Copyright (c) 2014 Natale Patriciello, <natale.patriciello@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "tcp-mrvhs.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/simulator.h"
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpMrvhs");
NS_OBJECT_ENSURE_REGISTERED(TcpMrvhs);

TypeId
TcpMrvhs::GetTypeId()
{
    static TypeId tid = TypeId("ns3::TcpMrvhs")
                            .SetParent<TcpHighSpeed>()
                            .AddConstructor<TcpMrvhs>()
                            .SetGroupName("Internet");
    return tid;
}

TcpMrvhs::TcpMrvhs()
    : TcpHighSpeed()
{
    m_rttBase = Time(0);
    m_rttMax = Time(0);
}

TcpMrvhs::TcpMrvhs(const TcpMrvhs& sock)
    : TcpHighSpeed(sock)
{
    m_rttBase = sock.m_rttBase;
    m_rttMax = sock.m_rttMax;
}

TcpMrvhs::~TcpMrvhs()
{
    NS_LOG_FUNCTION(this);
}

Ptr<TcpCongestionOps>
TcpMrvhs::Fork()
{
    return CopyObject<TcpMrvhs>(this);
}

/**
 * \brief Congestion avoidance of TcpHighSpeed
 *
 * As implementation choice, we increment cWnd only by MSS, when the right
 * number of ACK has been received. At this point, the important question is:
 * what is the "right number of ACK" ?
 *
 * As you can recall from RFC, Highspeed works this way:
 *
 *               w = w + a(w)/w
 *
 * Let's start when a(w) is 1 (so it is classical NewReno). The formula then is
 * the classical text-book version for NewReno:
 *
 *               w = w + 1 / w
 *
 * So, for each segment acked, we increase the window by the quantity 1/w. Or,
 * instead of adding the 1/w quantity for each segment acked, we can track the
 * number of segments acked (m_ackCnt) and increment by 1 MSS when m_ackCnt
 * reaches w.
 *
 * When a(w) > 1, it means that each segment acked has a different "weight".
 * For instance, when it is equal to 2, we need to increase the window by the
 * quantity 2/w. But, this means that one segment acked is equivalent (from
 * the point of view of incrementing cWnd) to two segments acked in NewReno
 * (1/w + 1/w). That a coefficient is, in other word, the weight of each segment
 * acked. More weight, less ACK are necessary to increment cWnd, which is
 * exactly the Highspeed principle.
 *
 * \param tcb internal congestion state
 * \param segmentsAcked count of segments acked
 */
void
TcpMrvhs::CongestionAvoidance(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked);

    if (segmentsAcked == 0)
        return;

    Time rtt = tcb->m_lastRtt;

    if (rtt.IsZero())
        return;

    // Update RTTbase (minimum RTT)
    if (m_rttBase.IsZero() || rtt < m_rttBase)
    {
        m_rttBase = rtt;
    }

    // Update RTTmax (maximum RTT)
    if (rtt > m_rttMax)
    {
        m_rttMax = rtt;
    }

    // Compute TimeRatio
    double timeRatio =
        (m_rttMax.GetSeconds() + m_rttBase.GetSeconds()) /
        rtt.GetSeconds();

    // MSS (bytes)
    double mss = tcb->m_segmentSize;

    // Compute a
    double a = std::sqrt(mss * timeRatio);
    
    // Convert cwnd to segments
    double segCwnd = tcb->m_cWnd / mss;

    if (segCwnd < 1.0)
        segCwnd = 1.0;

    // Update cwnd in segment domain
    segCwnd += a / segCwnd;

    // Convert back to bytes
    tcb->m_cWnd = segCwnd * mss;

    NS_LOG_INFO("MRVHS cwnd updated to " << tcb->m_cWnd);
}

std::string
TcpMrvhs::GetName() const
{
    return "TcpMrvhs";
}

/**
 * \brief Get slow start threshold following HighSpeed principles
 *
 * \param tcb internal congestion state
 * \param bytesInFlight Bytes in flight
 *
 * \return the slow start threshold value
 */
uint32_t
TcpMrvhs::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
    double segCwnd = tcb->m_cWnd / tcb->m_segmentSize;

    double b = TableLookupB(segCwnd);

    uint32_t ssThresh = static_cast<uint32_t>(
        std::max(2.0, segCwnd * (1 - b)) * tcb->m_segmentSize);

    return ssThresh;
}

double
TcpMrvhs::TableLookupB(uint32_t w)
{
    // NOLINTBEGIN(bugprone-branch-clone)
    if (w <= 38)
    {
        return 0.50;
    }
    else if (w <= 118)
    {
        return 0.44;
    }
    else if (w <= 221)
    {
        return 0.41;
    }
    else if (w <= 347)
    {
        return 0.38;
    }
    else if (w <= 495)
    {
        return 0.37;
    }
    else if (w <= 663)
    {
        return 0.35;
    }
    else if (w <= 851)
    {
        return 0.34;
    }
    else if (w <= 1058)
    {
        return 0.33;
    }
    else if (w <= 1284)
    {
        return 0.32;
    }
    else if (w <= 1529)
    {
        return 0.31;
    }
    else if (w <= 1793)
    {
        return 0.30;
    }
    else if (w <= 2076)
    {
        return 0.29;
    }
    else if (w <= 2378)
    {
        return 0.28;
    }
    else if (w <= 2699)
    {
        return 0.28;
    }
    else if (w <= 3039)
    {
        return 0.27;
    }
    else if (w <= 3399)
    {
        return 0.27;
    }
    else if (w <= 3778)
    {
        return 0.26;
    }
    else if (w <= 4177)
    {
        return 0.26;
    }
    else if (w <= 4596)
    {
        return 0.25;
    }
    else if (w <= 5036)
    {
        return 0.25;
    }
    else if (w <= 5497)
    {
        return 0.24;
    }
    else if (w <= 5979)
    {
        return 0.24;
    }
    else if (w <= 6483)
    {
        return 0.23;
    }
    else if (w <= 7009)
    {
        return 0.23;
    }
    else if (w <= 7558)
    {
        return 0.22;
    }
    else if (w <= 8130)
    {
        return 0.22;
    }
    else if (w <= 8726)
    {
        return 0.22;
    }
    else if (w <= 9346)
    {
        return 0.21;
    }
    else if (w <= 9991)
    {
        return 0.21;
    }
    else if (w <= 10661)
    {
        return 0.21;
    }
    else if (w <= 11358)
    {
        return 0.20;
    }
    else if (w <= 12082)
    {
        return 0.20;
    }
    else if (w <= 12834)
    {
        return 0.20;
    }
    else if (w <= 13614)
    {
        return 0.19;
    }
    else if (w <= 14424)
    {
        return 0.19;
    }
    else if (w <= 15265)
    {
        return 0.19;
    }
    else if (w <= 16137)
    {
        return 0.19;
    }
    else if (w <= 17042)
    {
        return 0.18;
    }
    else if (w <= 17981)
    {
        return 0.18;
    }
    else if (w <= 18955)
    {
        return 0.18;
    }
    else if (w <= 19965)
    {
        return 0.17;
    }
    else if (w <= 21013)
    {
        return 0.17;
    }
    else if (w <= 22101)
    {
        return 0.17;
    }
    else if (w <= 23230)
    {
        return 0.17;
    }
    else if (w <= 24402)
    {
        return 0.16;
    }
    else if (w <= 25618)
    {
        return 0.16;
    }
    else if (w <= 26881)
    {
        return 0.16;
    }
    else if (w <= 28193)
    {
        return 0.16;
    }
    else if (w <= 29557)
    {
        return 0.15;
    }
    else if (w <= 30975)
    {
        return 0.15;
    }
    else if (w <= 32450)
    {
        return 0.15;
    }
    else if (w <= 33986)
    {
        return 0.15;
    }
    else if (w <= 35586)
    {
        return 0.14;
    }
    else if (w <= 37253)
    {
        return 0.14;
    }
    else if (w <= 38992)
    {
        return 0.14;
    }
    else if (w <= 40808)
    {
        return 0.14;
    }
    else if (w <= 42707)
    {
        return 0.13;
    }
    else if (w <= 44694)
    {
        return 0.13;
    }
    else if (w <= 46776)
    {
        return 0.13;
    }
    else if (w <= 48961)
    {
        return 0.13;
    }
    else if (w <= 51258)
    {
        return 0.13;
    }
    else if (w <= 53667)
    {
        return 0.12;
    }
    else if (w <= 56230)
    {
        return 0.12;
    }
    else if (w <= 58932)
    {
        return 0.12;
    }
    else if (w <= 61799)
    {
        return 0.12;
    }
    else if (w <= 64851)
    {
        return 0.11;
    }
    else if (w <= 68113)
    {
        return 0.11;
    }
    else if (w <= 71617)
    {
        return 0.11;
    }
    else if (w <= 75401)
    {
        return 0.10;
    }
    else if (w <= 79517)
    {
        return 0.10;
    }
    else if (w <= 84035)
    {
        return 0.10;
    }
    else if (w <= 89053)
    {
        return 0.10;
    }
    else if (w <= 94717)
    {
        return 0.09;
    }
    else
    {
        return 0.09;
    }
    // NOLINTEND(bugprone-branch-clone)
}

} // namespace ns3
