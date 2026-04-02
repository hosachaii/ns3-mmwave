/*
 * mrvhs-simulation.cc
 *
 * Reproduces the experiment from:
 *   Alramli et al. (2025) "An adaptive congestion control algorithm for improving
 *   Transmission Control Protocol performance over cellular-to-cloud networks"
 *   PeerJ Computer Science, DOI 10.7717/peerj-cs.2956
 *
 * Key paper parameters reproduced:
 *   - 28 GHz carrier, 1 GHz bandwidth
 *   - One-way delay PGW->Server = 40 ms -> min RTT ~80 ms
 *   - 60 s simulation, BulkSend downlink (remote host -> UE)
 *   - Two buildings causing LoS/NLoS transitions
 *   - UE moves through LoS -> NLoS -> stop -> resume phases
 *   - PER applied on the mmWave UE device (where radio errors actually occur)
 *   - Four PER levels: null=0, small=1e-4, medium=1e-3, high=1e-2
 *   - Outputs: cwnd CSV, RTT CSV, summary (throughput + latency)
 *
 * Usage:
 *   ./ns3 run "mrvhs-simulation \
 *       --tcpType=ns3::TcpMrvhs \
 *       --perLevel=high \
 *       --outDir=results/"
 */

/*
 * mrvhs-simulation.cc
 *
 * Reproduces the experiment from:
 *   Alramli et al. (2025) "An adaptive congestion control algorithm for improving
 *   Transmission Control Protocol performance over cellular-to-cloud networks"
 *   PeerJ Computer Science, DOI 10.7717/peerj-cs.2956
 *
 * Usage:
 *   ./ns3 run "mrvhs-simulation \
 *       --tcpType=ns3::TcpMrvhs \
 *       --perLevel=high \
 *       --outDir=results"
 *
 * Supported tcpType:
 *   ns3::TcpMrvhs  ns3::TcpHighSpeed  ns3::TcpCubic  ns3::TcpNewReno  ns3::TcpBbr
 *
 * Supported perLevel:
 *   null (0)  small (1e-4)  medium (1e-3)  high (1e-2)
 */

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
static double   g_totalRttMs = 0.0;
static uint32_t g_rttCount   = 0;

// ---------------------------------------------------------------------------
// Trace callbacks  (context-aware: first arg is the path string)
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
// Use ConnectFailSafe so missing sockets don't abort the simulation.
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
// main
// ---------------------------------------------------------------------------
int
main(int argc, char* argv[])
{
    double      simTime  = 60.0;
    std::string tcpType  = "ns3::TcpMrvhs";
    std::string perLevel = "null";
    std::string outDir   = "results";

    CommandLine cmd;
    cmd.AddValue("simTime",  "Simulation duration (s)",              simTime);
    cmd.AddValue("tcpType",  "TCP congestion control TypeId string", tcpType);
    cmd.AddValue("perLevel", "PER scenario: null/small/medium/high", perLevel);
    cmd.AddValue("outDir",   "Directory for output CSV files",       outDir);
    cmd.Parse(argc, argv);

    // -----------------------------------------------------------------------
    // PER mapping  — realistic residual error rates after HARQ
    // -----------------------------------------------------------------------
    std::map<std::string, double> perMap = {
        {"null",   0.0},
        {"small",  1e-4},
        {"medium", 1e-3},
        {"high",   1e-2}
    };
    if (perMap.find(perLevel) == perMap.end())
    {
        std::cerr << "Unknown --perLevel '" << perLevel
                  << "'. Choose: null | small | medium | high\n";
        return 1;
    }
    double errorRate = perMap[perLevel];

    // -----------------------------------------------------------------------
    // Validate TCP TypeId early so the error is readable
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
    cwndFile << "Time_s,CWND_bytes\n";
    rttFile  << "Time_s,RTT_ms\n";

    // -----------------------------------------------------------------------
    // TCP global defaults
    // -----------------------------------------------------------------------
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(tcpTid));
    Config::SetDefault("ns3::TcpSocketBase::WindowScaling", BooleanValue(true));
    
    // 1. Massive TCP Buffers for 10Gbps @ 100ms RTT
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(50 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(50 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1400));

    // 2. DISABLE ROUTER DROPS (The ns-3.42 Traffic Control Trap)
    // Expands all queues to 1 million packets so bursts don't get truncated
    Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", StringValue("1000000p"));
    Config::SetDefault("ns3::FqCoDelQueueDisc::MaxSize", StringValue("1000000p"));
    Config::SetDefault("ns3::PfifoFastQueueDisc::MaxSize", StringValue("1000000p"));

    // 3. DISABLE RADIO TOWER DROPS (mmWave RLC Buffers)
    // Prevents the eNB from dropping packets before they can be beamed over the air
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(100 * 1024 * 1024));
    Config::SetDefault("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(100 * 1024 * 1024));
    
    // 4. UNLOCK THE INTERNAL CORE NETWORK (S1-U Interface)
    // The EPC automatically creates hidden links between the Tower, SGW, and PGW.
    // We must force these hidden links to 10 Gbps so they don't bottleneck the radio!
    Config::SetDefault("ns3::PointToPointEpcHelper::S1uLinkDataRate", DataRateValue(DataRate("10Gbps")));
    Config::SetDefault("ns3::PointToPointEpcHelper::S1uLinkDelay", TimeValue(MilliSeconds(1)));

    // 5. UNLOCK MMWAVE RADIO BANDWIDTH
    // The mmWave module inherits from 4G LTE. If not explicitly overridden, it defaults 
    // to a tiny 20 MHz band. The paper explicitly specifies a massive 1 GHz bandwidth!
    Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(28e9));
    Config::SetDefault("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue(1e9));

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
    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();
    Ptr<MmWavePointToPointEpcHelper> epcHelper =
        CreateObject<MmWavePointToPointEpcHelper>();
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

    // Apply residual error on the P2P interface (simulates end-to-end IP loss
    // after mmWave MAC HARQ retransmissions have already corrected physical errors)
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

    // -------- BUILDINGS --------
    Ptr<Building> b1 = CreateObject<Building>();
    // Set Y from 1.0 to 19.0 (keeps it off the street)
    b1->SetBoundaries(Box(20.0, 40.0, 1.0, 19.0, 0.0, 20.0));
    b1->SetBuildingType(Building::Residential);
    b1->SetExtWallsType(Building::ConcreteWithWindows);

    Ptr<Building> b2 = CreateObject<Building>();
    // Set Y from 1.0 to 19.0 
    b2->SetBoundaries(Box(60.0, 70.0, 1.0, 19.0, 0.0, 20.0));
    b2->SetBuildingType(Building::Residential);
    b2->SetExtWallsType(Building::ConcreteWithWindows);

    // -------- MOBILITY --------
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbNodes);
    enbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 1.5));

    MobilityHelper ueMobilityHelper;
    ueMobilityHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobilityHelper.Install(ueNodes);
    
    Ptr<ConstantVelocityMobilityModel> cvmm =
        ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
        
    // CRITICAL FIX: Put the UE on the "street" at Y = 20.0 so it stays outdoors!
    cvmm->SetPosition(Vector(10.0, 20.0, 1.5));
    cvmm->SetVelocity(Vector(0.0,  0.0, 0.0));

    // The velocity keeps it walking perfectly straight along the Y=20.0 line
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
    sourceApp.Start(Seconds(3.0));   // 3 s for EPC bearer setup
    sourceApp.Stop(Seconds(simTime));

    // Schedule trace connection slightly after app start so sockets exist
    Simulator::Schedule(Seconds(3.00001), &ConnectTracers);

    // -----------------------------------------------------------------------
    // Run
    // -----------------------------------------------------------------------
    std::cout << "Starting simulation\n"
              << "  TCP       : " << tcpType   << "\n"
              << "  PER level : " << perLevel  << " (" << errorRate << ")\n"
              << "  Remote IP : " << internetIfaces.GetAddress(1) << "\n"
              << "  UE IP     : " << ueIpIface.GetAddress(0)      << "\n"
              << "  Output    : " << outDir << "/\n";

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // -----------------------------------------------------------------------
    // Results
    // -----------------------------------------------------------------------
    Ptr<PacketSink> sink    = DynamicCast<PacketSink>(sinkApp.Get(0));
    double activeTime       = simTime - 3.0;
    double throughputMbps   = (sink->GetTotalRx() * 8.0) / activeTime / 1e6;
    double avgLatencyMs     = (g_rttCount > 0)
                              ? (g_totalRttMs / g_rttCount) / 2.0
                              : 0.0;

    std::cout << "------------------------------------------\n"
              << "TCP Variant     : " << tcpType          << "\n"
              << "PER             : " << errorRate         << "\n"
              << "Total RX        : " << sink->GetTotalRx()<< " bytes\n"
              << "Throughput      : " << throughputMbps    << " Mbps\n"
              << "Avg latency     : " << avgLatencyMs      << " ms (RTT/2)\n"
              << "------------------------------------------\n";

    // Summary CSV (one line per run, easy to aggregate)
    std::ofstream sumFile(outDir + "/summary_" + cleanTcp + "_" + perLevel + ".csv");
    sumFile << "tcp,per_level,per,throughput_mbps,avg_latency_ms\n"
            << tcpType << "," << perLevel << "," << errorRate << ","
            << throughputMbps << "," << avgLatencyMs << "\n";

    cwndFile.close();
    rttFile .close();
    sumFile .close();

    Simulator::Destroy();
    return 0;
}
