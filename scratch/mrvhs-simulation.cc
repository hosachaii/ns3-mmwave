#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/error-model.h"
#include "ns3/mmwave-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"

#include <fstream>
#include <map>
#include <sys/stat.h>

using namespace ns3;
using namespace ns3::mmwave;

NS_LOG_COMPONENT_DEFINE("MrvhsMmwaveFull");

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static std::ofstream cwndFile;
static std::ofstream rttFile;
static std::ofstream instThroughputFile;
static double   g_totalRttMs = 0.0;
static uint32_t g_rttCount   = 0;
static uint64_t g_lastTotalRx = 0;

// ---------------------------------------------------------------------------
// Trace callbacks
// ---------------------------------------------------------------------------
static void
CwndTracer(std::string /*context*/, uint32_t /*oldCwnd*/, uint32_t newCwnd)
{
    cwndFile << Simulator::Now().GetSeconds() << "," << newCwnd << "\n";
}

static void
RttTracer(std::string /*context*/, Time /*oldRtt*/, Time newRtt)
{
    double ms = newRtt.GetMilliSeconds();
    rttFile << Simulator::Now().GetSeconds() << "," << ms << "\n";
    g_totalRttMs += ms;
    ++g_rttCount;
}

// ---------------------------------------------------------------------------
// Deferred trace connection
// ---------------------------------------------------------------------------
static void
ConnectTracers()
{
    Config::ConnectFailSafe(
        "/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow",
        MakeCallback(&CwndTracer));

    Config::ConnectFailSafe(
        "/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/RTT",
        MakeCallback(&RttTracer));
}

// ---------------------------------------------------------------------------
// Instantaneous throughput sampler (from file 2)
// ---------------------------------------------------------------------------
static void
CalculateInstantaneousThroughput(Ptr<PacketSink> sink, double interval)
{
    uint64_t currentRx = sink->GetTotalRx();
    double mbps = ((currentRx - g_lastTotalRx) * 8.0) / (interval * 1000000.0);
    instThroughputFile << Simulator::Now().GetSeconds() << "," << mbps << "\n";
    g_lastTotalRx = currentRx;
    Simulator::Schedule(Seconds(interval), &CalculateInstantaneousThroughput, sink, interval);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int
main(int argc, char* argv[])
{
    double      simTime  = 60.0;
    std::string tcpType  = "ns3::TcpCubic";
    std::string perLevel = "null";
    std::string outDir   = "results";

    CommandLine cmd;
    cmd.AddValue("simTime",  "Simulation duration (s)",              simTime);
    cmd.AddValue("tcpType",  "TCP congestion control TypeId string", tcpType);
    cmd.AddValue("perLevel", "PER scenario: null/small/medium/high", perLevel);
    cmd.AddValue("outDir",   "Directory for output CSV files",       outDir);
    cmd.Parse(argc, argv);

    // -----------------------------------------------------------------------
    // PER mapping — realistic residual error rates after HARQ
    // -----------------------------------------------------------------------
    std::map<std::string, double> perMap = {
        {"null",   0.0},
        {"small",  1e-4},
        {"medium", 1e-3},
        {"high",   1e-2}
    };
    // Validate perLevel early (from file 1)
    if (perMap.find(perLevel) == perMap.end())
    {
        std::cerr << "Unknown --perLevel '" << perLevel
                  << "'. Choose: null | small | medium | high\n";
        return 1;
    }
    double errorRate = perMap[perLevel];

    // -----------------------------------------------------------------------
    // Validate TCP TypeId early so the error is readable (from file 1)
    // -----------------------------------------------------------------------
    TypeId tcpTid;
    if (!TypeId::LookupByNameFailSafe(tcpType, &tcpTid))
    {
        NS_FATAL_ERROR("TCP type not found: " << tcpType
                       << ". Ensure tcp-mrvhs.cc is compiled into ns-3.");
    }

    // -----------------------------------------------------------------------
    // Output files
    // -----------------------------------------------------------------------
    mkdir(outDir.c_str(), 0777);
    std::string cleanTcp = tcpType;
    {
        auto p = cleanTcp.find("ns3::");
        if (p != std::string::npos) cleanTcp.erase(p, 5);
    }

    cwndFile.open(outDir + "/cwnd_" + cleanTcp + "_" + perLevel + ".csv");
    rttFile .open(outDir + "/rtt_"  + cleanTcp + "_" + perLevel + ".csv");
    instThroughputFile.open(outDir + "/inst_tp_" + cleanTcp + "_" + perLevel + ".csv");
    cwndFile           << "Time_s,CWND_bytes\n";
    rttFile            << "Time_s,RTT_ms\n";
    instThroughputFile << "Time_s,Throughput_Mbps\n";

    // -----------------------------------------------------------------------
    // TCP global defaults
    // -----------------------------------------------------------------------
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(tcpTid));
    Config::SetDefault("ns3::TcpSocketBase::WindowScaling", BooleanValue(true));

    // TcpMrvhsOpt-specific settings (from file 1)
    if (tcpType == "ns3::TcpMrvhsOpt")
    {
        Config::SetDefault("ns3::TcpMrvhsOpt::CwndFormula",  StringValue("logdamp"));
        Config::SetDefault("ns3::TcpMrvhsOpt::AdaptiveBeta", BooleanValue(true));
        Config::SetDefault("ns3::TcpMrvhsOpt::EcnReaction",  BooleanValue(true));
        Config::SetDefault("ns3::TcpSocketBase::UseEcn",      StringValue("On"));
    }

    // 1. Massive TCP Buffers for 10 Gbps @ 100 ms RTT (file 1: 50 MB; kept larger)
    Config::SetDefaultFailSafe("ns3::TcpTxBuffer::MaxBufferSize", UintegerValue(50 * 1024 * 1024));
    Config::SetDefaultFailSafe("ns3::TcpRxBuffer::MaxBufferSize", UintegerValue(50 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(50 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(50 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1400));

    // 2. Disable router drops — expand all queues (from file 1)
    Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", StringValue("1000000p"));
    Config::SetDefault("ns3::FqCoDelQueueDisc::MaxSize",      StringValue("1000000p"));
    Config::SetDefault("ns3::PfifoFastQueueDisc::MaxSize",    StringValue("1000000p"));

    // 3. Disable radio tower drops — mmWave RLC buffers
    Config::SetDefaultFailSafe("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(100 * 1024 * 1024));
    Config::SetDefaultFailSafe("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(100 * 1024 * 1024));

    // 4. Unlock the internal core network — S1-U interface
    //    Both the generic and mmWave-specific EPC helpers are covered.
    Config::SetDefault        ("ns3::PointToPointEpcHelper::S1uLinkDataRate",      DataRateValue(DataRate("10Gbps")));
    Config::SetDefaultFailSafe("ns3::MmWavePointToPointEpcHelper::S1uLinkDataRate",DataRateValue(DataRate("10Gbps")));
    Config::SetDefaultFailSafe("ns3::MmWavePointToPointEpcHelper::S1uLinkDelay",   TimeValue(MilliSeconds(1)));
    Config::SetDefault        ("ns3::PointToPointEpcHelper::S1uLinkDelay",         TimeValue(MilliSeconds(1)));

    // 5. Unlock mmWave radio bandwidth — 1 GHz @ 28 GHz
    Config::SetDefaultFailSafe("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(28e9));
    Config::SetDefaultFailSafe("ns3::MmWavePhyMacCommon::Bandwidth",  DoubleValue(1e9));

    // -----------------------------------------------------------------------
    // Nodes
    // -----------------------------------------------------------------------
    NodeContainer ueNodes, enbNodes, remoteHostContainer;
    ueNodes.Create(1);
    enbNodes.Create(1);
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // -----------------------------------------------------------------------
    // mmWave helper + EPC
    // -----------------------------------------------------------------------
    Ptr<MmWaveHelper>                  mmwaveHelper = CreateObject<MmWaveHelper>();
    Ptr<MmWavePointToPointEpcHelper>   epcHelper    = CreateObject<MmWavePointToPointEpcHelper>();
    mmwaveHelper->SetEpcHelper(epcHelper);
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // -----------------------------------------------------------------------
    // Internet stack
    // -----------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(remoteHostContainer);

    // -----------------------------------------------------------------------
    // PGW <-> Remote host: 10 Gbps, 40 ms one-way => RTT_min ~80 ms
    // -----------------------------------------------------------------------
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay",    StringValue("40ms"));

    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Apply residual error on the P2P interface
    if (errorRate > 0.0)
    {
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetAttribute("ErrorRate", DoubleValue(errorRate));
        em->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        internetDevices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    }

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("11.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIfaces = ipv4h.Assign(internetDevices);

    // -----------------------------------------------------------------------
    // Static routing: remote host -> UE subnet 7.0.0.0/8
    // -----------------------------------------------------------------------
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"),
        Ipv4Address("11.0.0.1"), 1);

    // -----------------------------------------------------------------------
    // Buildings
    // -----------------------------------------------------------------------
    Ptr<Building> b1 = CreateObject<Building>();
    b1->SetBoundaries(Box(20.0, 40.0, 1.0, 19.0, 0.0, 20.0));
    b1->SetBuildingType(Building::Residential);
    b1->SetExtWallsType(Building::ConcreteWithWindows);

    Ptr<Building> b2 = CreateObject<Building>();
    b2->SetBoundaries(Box(60.0, 70.0, 1.0, 19.0, 0.0, 20.0));
    b2->SetBuildingType(Building::Residential);
    b2->SetExtWallsType(Building::ConcreteWithWindows);

    // -----------------------------------------------------------------------
    // Mobility
    // -----------------------------------------------------------------------
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbNodes);
    enbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 1.5));

    MobilityHelper ueMobilityHelper;
    ueMobilityHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobilityHelper.Install(ueNodes);

    Ptr<ConstantVelocityMobilityModel> cvmm =
        ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    cvmm->SetPosition(Vector(10.0, 20.0, 1.5));
    cvmm->SetVelocity(Vector(0.0,  0.0,  0.0));

    Simulator::Schedule(Seconds(10.0), &ConstantVelocityMobilityModel::SetVelocity,
                        cvmm, Vector(2.0, 0.0, 0.0));
    Simulator::Schedule(Seconds(20.0), &ConstantVelocityMobilityModel::SetVelocity,
                        cvmm, Vector(0.0, 0.0, 0.0));
    Simulator::Schedule(Seconds(30.0), &ConstantVelocityMobilityModel::SetVelocity,
                        cvmm, Vector(2.0, 0.0, 0.0));
    Simulator::Schedule(Seconds(40.0), &ConstantVelocityMobilityModel::SetVelocity,
                        cvmm, Vector(2.5, 0.0, 0.0));

    BuildingsHelper::Install(enbNodes);
    BuildingsHelper::Install(ueNodes);

    // -----------------------------------------------------------------------
    // mmWave radio devices
    // -----------------------------------------------------------------------
    NetDeviceContainer enbDevs = mmwaveHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs  = mmwaveHelper->InstallUeDevice(ueNodes);

    Ipv4InterfaceContainer ueIpIface =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    mmwaveHelper->AttachToClosestEnb(ueDevs, enbDevs);

    // -----------------------------------------------------------------------
    // Application: BulkSend downlink (remote host -> UE)
    // -----------------------------------------------------------------------
    uint16_t port = 5000;

    PacketSinkHelper sinkHelper(
        "ns3::TcpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    BulkSendHelper source(
        "ns3::TcpSocketFactory",
        InetSocketAddress(ueIpIface.GetAddress(0), port));
    source.SetAttribute("MaxBytes", UintegerValue(0));
    source.SetAttribute("SendSize", UintegerValue(1400));
    ApplicationContainer sourceApp = source.Install(remoteHost);
    sourceApp.Start(Seconds(3.0));
    sourceApp.Stop(Seconds(simTime));

    // Schedule trace connection slightly after app start so sockets exist
    Simulator::Schedule(Seconds(3.00001), &ConnectTracers);

    // Get sink pointer before Run() for the throughput sampler
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    // Start instantaneous throughput sampler at 3.1 s, every 0.1 s
    Simulator::Schedule(Seconds(3.1), &CalculateInstantaneousThroughput, sink, 0.1);

    // -----------------------------------------------------------------------
    // Run
    // -----------------------------------------------------------------------
    std::cout << "Starting simulation\n"
              << "  TCP       : " << tcpType                      << "\n"
              << "  PER level : " << perLevel << " (" << errorRate << ")\n"
              << "  Remote IP : " << internetIfaces.GetAddress(1) << "\n"
              << "  UE IP     : " << ueIpIface.GetAddress(0)      << "\n"
              << "  Output    : " << outDir << "/\n";

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // -----------------------------------------------------------------------
    // Results
    // -----------------------------------------------------------------------
    double activeTime     = simTime - 3.0;
    double throughputMbps = (sink->GetTotalRx() * 8.0) / activeTime / 1e6;
    double avgLatencyMs   = (g_rttCount > 0)
                            ? (g_totalRttMs / g_rttCount) / 2.0
                            : 0.0;

    std::cout << "------------------------------------------\n"
              << "TCP Variant     : " << tcpType           << "\n"
              << "PER             : " << errorRate          << "\n"
              << "Total RX        : " << sink->GetTotalRx() << " bytes\n"
              << "Throughput      : " << throughputMbps     << " Mbps\n"
              << "Avg latency     : " << avgLatencyMs       << " ms (RTT/2)\n"
              << "------------------------------------------\n";

    // Summary CSV (one line per run, easy to aggregate)
    std::ofstream sumFile(outDir + "/summary_" + cleanTcp + "_" + perLevel + ".csv");
    sumFile << "tcp,per_level,per,throughput_mbps,avg_latency_ms\n"
            << tcpType << "," << perLevel << "," << errorRate << ","
            << throughputMbps << "," << avgLatencyMs << "\n";

    cwndFile.close();
    rttFile .close();
    instThroughputFile.close();
    sumFile .close();

    Simulator::Destroy();
    return 0;
}
