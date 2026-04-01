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

using namespace ns3;
using namespace ns3::mmwave;

NS_LOG_COMPONENT_DEFINE("MrvhsMmwaveFull");

std::ofstream cwndFile;
std::ofstream rttFile;

double g_totalRttMs = 0.0;
uint32_t g_rttCount = 0;

static void CwndTracer(std::string context, uint32_t oldCwnd, uint32_t newCwnd) {
    cwndFile << Simulator::Now().GetSeconds() << "," << newCwnd << "\n";
}

static void RttTracer(std::string context, Time oldRtt, Time newRtt) {
    double rttMs = newRtt.GetMilliSeconds();
    rttFile << Simulator::Now().GetSeconds() << "," << rttMs << "\n";
    g_totalRttMs += rttMs;
    g_rttCount++;
}

static void ConnectTracers() {
    Config::ConnectFailSafe("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow", MakeCallback(&CwndTracer));
    Config::ConnectFailSafe("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/RTT", MakeCallback(&RttTracer));
}

int main(int argc, char *argv[])
{
    double simTime = 60.0;
    std::string tcpType = "ns3::TcpMrvhs";
    std::string perLevel = "null";

    CommandLine cmd;
    cmd.AddValue("tcpType", "TCP variant", tcpType);
    cmd.AddValue("perLevel", "PER: null, small, medium, high", perLevel);
    cmd.Parse(argc, argv);

    TypeId tcpTid;
    if (!TypeId::LookupByNameFailSafe(tcpType, &tcpTid)) {
        NS_FATAL_ERROR("TCP Type Not Found: " << tcpType);
    }
    
    // -------- CORE TCP CONFIGURATIONS --------
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(tcpTid));
    Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue(Seconds(1.0)));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocketBase::WindowScaling", BooleanValue(true));
    // CRITICAL: Lower MTU to prevent silent drops inside the EPC GTP tunnels
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1400));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));

    double perProbability = 0.0;
    if (perLevel == "small") perProbability = 0.01;
    else if (perLevel == "medium") perProbability = 0.05;
    else if (perLevel == "high") perProbability = 0.10;

    cwndFile.open("cwnd_" + tcpType + "_" + perLevel + ".csv");
    rttFile.open("rtt_" + tcpType + "_" + perLevel + ".csv");
    cwndFile << "Time,CWND\n";
    rttFile << "Time,RTT_ms\n";

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    NodeContainer remoteHostContainer;
    ueNodes.Create(1);
    enbNodes.Create(1);
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // -------- MMWAVE + EPC --------
    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();
    Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper>();
    mmwaveHelper->SetEpcHelper(epcHelper);
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // -------- INTERNET STACK --------
    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(remoteHostContainer);

    // -------- P2P LINK --------
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("40ms"));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    if (perProbability > 0.0) {
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetAttribute("ErrorRate", DoubleValue(perProbability));
        em->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        internetDevices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    }

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("11.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIfaces = ipv4h.Assign(internetDevices);

    // -------- ROUTING --------
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), Ipv4Address("11.0.0.1"), 1);

    // -------- BUILDINGS --------
    BuildingContainer buildings;
    Ptr<Building> b1 = CreateObject<Building>();
    b1->SetBoundaries(Box(20.0, 40.0, -10.0, 10.0, 0.0, 20.0));
    buildings.Add(b1);
    
    Ptr<Building> b2 = CreateObject<Building>();
    b2->SetBoundaries(Box(60.0, 70.0, -10.0, 10.0, 0.0, 20.0));
    buildings.Add(b2);

    // -------- MOBILITY --------
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbNodes);
    enbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0,0,1.5));

    MobilityHelper ueMobilityHelper;
    ueMobilityHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobilityHelper.Install(ueNodes);
    
    Ptr<ConstantVelocityMobilityModel> cvmm = ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    cvmm->SetPosition(Vector(10.0, 0.0, 1.5));
    cvmm->SetVelocity(Vector(0.0, 0.0, 0.0));

    Simulator::Schedule(Seconds(10.0), &ConstantVelocityMobilityModel::SetVelocity, cvmm, Vector(2.0, 0.0, 0.0));
    Simulator::Schedule(Seconds(20.0), &ConstantVelocityMobilityModel::SetVelocity, cvmm, Vector(0.0, 0.0, 0.0));
    Simulator::Schedule(Seconds(30.0), &ConstantVelocityMobilityModel::SetVelocity, cvmm, Vector(2.0, 0.0, 0.0));
    Simulator::Schedule(Seconds(40.0), &ConstantVelocityMobilityModel::SetVelocity, cvmm, Vector(2.5, 0.0, 0.0));

    BuildingsHelper::Install(enbNodes);
    BuildingsHelper::Install(ueNodes);

    // -------- MMWAVE DEVICES --------
    NetDeviceContainer enbDevs = mmwaveHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs = mmwaveHelper->InstallUeDevice(ueNodes);

    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // 1. Attach to eNB (Automatically sets up the Default EPS Bearer for IP traffic!)
    mmwaveHelper->AttachToClosestEnb(ueDevs, enbDevs);

    // -------- APPLICATION --------
    uint16_t port = 5000;
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    BulkSendHelper source("ns3::TcpSocketFactory", InetSocketAddress(ueIpIface.GetAddress(0), port));
    source.SetAttribute("MaxBytes", UintegerValue(0));
    ApplicationContainer sourceApp = source.Install(remoteHost);
    sourceApp.Start(Seconds(3.0));
    sourceApp.Stop(Seconds(simTime));

    Simulator::Schedule(Seconds(3.00001), &ConnectTracers);

    std::cout << "Starting Simulation: " << tcpType << " | PER: " << perLevel << std::endl;
    std::cout << "Remote Host IP: " << internetIfaces.GetAddress(1) << " --> UE IP: " << ueIpIface.GetAddress(0) << std::endl;
    
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // -------- CALCULATE METRICS (GROUND TRUTH) --------
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApp.Get(0));
    uint64_t totalRxBytes = sink->GetTotalRx();
    
    double activeTime = simTime - 3.0; // Account for the 3-second application delay
    double groundTruthThroughput = (totalRxBytes * 8.0) / activeTime / 1e6; // Mbps
    
    double averageLatencyMs = (g_rttCount > 0) ? (g_totalRttMs / g_rttCount) : 0.0;

    std::cout << "------------------------------------------\n";
    std::cout << "TCP Variant          : " << tcpType << "\n";
    std::cout << "PER Condition        : " << perLevel << "\n";
    std::cout << "Total Bytes Received : " << totalRxBytes << " Bytes\n";
    std::cout << "Average Throughput   : " << groundTruthThroughput << " Mbps\n";
    std::cout << "Average Latency      : " << averageLatencyMs << " ms\n";
    std::cout << "------------------------------------------\n";

    cwndFile.close();
    rttFile.close();
    Simulator::Destroy();
    return 0;
}
