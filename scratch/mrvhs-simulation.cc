#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"

#include "ns3/mmwave-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"

using namespace ns3;
using namespace ns3::mmwave;

NS_LOG_COMPONENT_DEFINE("MrvhsMmwaveFull");

// -------- CWND TRACE --------
static void
CwndTracer(uint32_t oldCwnd, uint32_t newCwnd)
{
    std::cout << Simulator::Now().GetSeconds()
              << " CWND " << newCwnd << std::endl;
}

// -------- RTT TRACE --------
static void
RttTracer(Time oldRtt, Time newRtt)
{
    std::cout << Simulator::Now().GetSeconds()
              << " RTT(ms) " << newRtt.GetMilliSeconds()
              << std::endl;
}

int main(int argc, char *argv[])
{
    double simTime = 60.0;
    std::string tcpType = "ns3::TcpMrvhs";
    std::string scenario = "null";

    CommandLine cmd;
    cmd.AddValue("tcpType", "TCP variant", tcpType);
    cmd.AddValue("scenario", "null/low/medium/high", scenario);
    cmd.Parse(argc, argv);

    // -------- TCP selection --------
    Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                       TypeIdValue(TypeId::LookupByName(tcpType)));

    // -------- Nodes --------
    NodeContainer ueNodes;
    NodeContainer enbNodes;
    NodeContainer remoteHostContainer;

    ueNodes.Create(1);
    enbNodes.Create(1);
    remoteHostContainer.Create(1);

    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // -------- mmWave + EPC --------
    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();
    Ptr<MmWavePointToPointEpcHelper> epcHelper =
        CreateObject<MmWavePointToPointEpcHelper>();

    mmwaveHelper->SetEpcHelper(epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // -------- Internet stack --------
    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(remoteHostContainer);

    // -------- PGW <-> Remote Host (40 ms delay → RTT ≈ 80 ms) --------
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("40ms"));

    NetDeviceContainer internetDevices =
        p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIfaces =
        ipv4h.Assign(internetDevices);

    // -------- Routing --------
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());

    remoteHostStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"),
        Ipv4Mask("255.0.0.0"),
        1);

    // -------- Buildings --------
    BuildingContainer buildings;

    // Building 1
    Ptr<Building> b1 = CreateObject<Building>();
    b1->SetBoundaries(Box(25.0, 40.0, -10.0, 10.0, 0.0, 20.0));
    buildings.Add(b1);

    // Building 2
    Ptr<Building> b2 = CreateObject<Building>();
    b2->SetBoundaries(Box(50.0, 70.0, -10.0, 10.0, 0.0, 20.0));
    buildings.Add(b2);

    // -------- Mobility --------
    MobilityHelper mobility;

    // gNB fixed
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(enbNodes);

    // UE fixed (scenario-based position)
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodes);

    // -------- Scenario-based UE placement --------
    Vector uePos;

    if (scenario == "null")
    {
        uePos = Vector(10, 0, 0);   // strong LoS
    }
    else if (scenario == "low")
    {
        uePos = Vector(30, 0, 0);   // weaker LoS
    }
    else if (scenario == "medium")
    {
        uePos = Vector(50, 0, 0);   // near blockage
    }
    else if (scenario == "high")
    {
        uePos = Vector(80, 0, 0);   // deep NLoS
    }

    enbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0,0,0));
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(uePos);

    // Install buildings into nodes
    BuildingsHelper::Install(enbNodes);
    BuildingsHelper::Install(ueNodes);

    // -------- mmWave devices --------
    NetDeviceContainer enbDevs =
        mmwaveHelper->InstallEnbDevice(enbNodes);

    NetDeviceContainer ueDevs =
        mmwaveHelper->InstallUeDevice(ueNodes);

    mmwaveHelper->AttachToClosestEnb(ueDevs, enbDevs);

    // -------- UE IP --------
    Ipv4InterfaceContainer ueIpIface =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());

    ueStaticRouting->SetDefaultRoute(
        epcHelper->GetUeDefaultGatewayAddress(), 1);

    // -------- APPLICATION --------
    uint16_t port = 5000;

    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), port));

    ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    BulkSendHelper source("ns3::TcpSocketFactory",
                          InetSocketAddress(ueIpIface.GetAddress(0), port));

    source.SetAttribute("MaxBytes", UintegerValue(0));

    ApplicationContainer sourceApp = source.Install(remoteHost);
    sourceApp.Start(Seconds(1.0));
    sourceApp.Stop(Seconds(simTime));

    // -------- TRACE --------
    Config::ConnectWithoutContext(
        "/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow",
        MakeCallback(&CwndTracer));

    Config::ConnectWithoutContext(
        "/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/RTT",
        MakeCallback(&RttTracer));

    // -------- RUN --------
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // -------- Throughput --------
    Ptr<PacketSink> sink =
        DynamicCast<PacketSink>(sinkApp.Get(0));

    double throughput =
        (sink->GetTotalRx() * 8.0) / simTime / 1e6;

    std::cout << "TCP: " << tcpType
              << " Scenario: " << scenario
              << " Throughput(Mbps): " << throughput
              << std::endl;

    Simulator::Destroy();
    return 0;
}
