#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/applications-module.h"

#include <map>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("A3_Hex_21cells_ControlOnly");

static double   kSimTimeSec   = 40.0;
static uint32_t kSites        = 7;        // 7 sites × 3 sectors = 21 cells
static double   kIsd          = 500.0;    // meters
static double   kEnbTxDbm     = 30.0;     // dBm
static uint16_t kDlRb         = 25;       // 5 MHz
static uint16_t kUlRb         = 25;
static uint32_t kUePerEnb     = 1;        // Reduced to 8 UEs total for NetAnim clarity
static double   kSpeedKmph    = 30.0;     // Paper: 3 or 30 kmph
static uint16_t kTttMs        = 50;       // {50,200,400}
static double   kHystDb       = 1.0;      // {1,3,6}
static uint16_t kTmMs         = 50;       // accepted, not applied in 3.39
static uint16_t kTfMs         = 200;      // accepted, not applied in 3.39
static bool     kFastMode     = false;
static bool     kDemo         = false;    // Use paper's random movement
static bool     kNoAnim       = false;    // Enable NetAnim for visualization
static double   kPingPongWinS = 2.0;
static std::ofstream g_csv;
static double g_sampleInterval = 1.1;
static Ptr<Application> g_udpServerApp;
static double g_clientPps = 100.0;  // packets per second
static std::ofstream g_hoEvents;


// ---------- HO accounting ----------
struct LastHo { uint16_t prevCell=0, lastCell=0; double lastTime=-1e9; };
static std::map<uint64_t, LastHo> g_lastHo;
static uint64_t g_totalHo = 0, g_pingPong = 0;

static void SampleThroughput()
{
  auto srv = DynamicCast<UdpServer>(g_udpServerApp);
  if (!srv) return;

  static uint64_t prevRxPkts = 0;
  static double prevTime = 0.0;

  uint64_t totalRxPkts = srv->GetReceived();
  double now = Simulator::Now().GetSeconds();
  double interval = now - prevTime;

  // Sanity check
  if (interval <= 0.0) {
    Simulator::Schedule(Seconds(g_sampleInterval), &SampleThroughput);
    return;
  }

  uint64_t deltaPkts = totalRxPkts - prevRxPkts;
  double throughputMbps = (deltaPkts * 1200 * 8.0) / (interval * 1e6);  // 1200-byte payload

  // Estimate loss assuming expected pps = 100, interval ~0.2s
  double expectedPkts = g_sampleInterval * g_clientPps;
  double lossFrac = (expectedPkts > 0) ? std::max(0.0, 1.0 - double(deltaPkts) / expectedPkts) : 0.0;

  // Estimate jitter as variation in inter-arrival delay
  static double lastArrivalTime = -1.0;
  double jitter = 0.0;
  if (deltaPkts > 0 && lastArrivalTime > 0.0) {
    double avgPktDelay = interval / deltaPkts;
    jitter = std::abs(avgPktDelay - (prevTime - lastArrivalTime));
  }
  lastArrivalTime = now;

  g_csv << std::fixed << std::setprecision(3)
        << now << "," << std::setprecision(6)
        << throughputMbps << "," << lossFrac << "," << jitter * 1000 << ","  // ms
        << totalRxPkts << "," << deltaPkts << "\n";

  prevRxPkts = totalRxPkts;
  prevTime = now;

  Simulator::Schedule(Seconds(g_sampleInterval), &SampleThroughput);
}

static void OnHoStart(std::string, uint64_t imsi, uint16_t fromCell, uint16_t, uint16_t toCell)
{
  double now = Simulator::Now().GetSeconds();
  if (g_hoEvents.tellp() == 0) {
    g_hoEvents << "time_s,imsi,from_cell,to_cell\n";
  }
  g_hoEvents << now << "," << imsi << "," << fromCell << "," << toCell << "\n";

  std::cout << std::fixed << std::setprecision(3)
            << now << "s [HO_START] IMSI " << imsi << " " << fromCell << "->" << toCell << "\n";
}


static void OnHoEndOk(std::string, uint64_t imsi, uint16_t newCell, uint16_t)
{
  double now = Simulator::Now().GetSeconds();
  auto &s = g_lastHo[imsi];
  if (s.prevCell == newCell && (now - s.lastTime) <= kPingPongWinS) ++g_pingPong;
  s.prevCell = s.lastCell; s.lastCell = newCell; s.lastTime = now;
  ++g_totalHo;
  std::cout << std::fixed << std::setprecision(3)
            << now << "s [HO_END]   IMSI " << imsi << " now=" << newCell << "\n";
}

// Helper function to change UE direction (for random movement)
static void ChangeUeDirection(Ptr<ConstantVelocityMobilityModel> cv, double speed, Ptr<UniformRandomVariable> rvDir)
{
  double angle = rvDir->GetValue();
  double vx = speed * std::cos(angle);
  double vy = speed * std::sin(angle);
  cv->SetVelocity(Vector(vx, vy, 0));
}

// ---------- Geometry ----------
static std::vector<Vector> HexSiteCenters(uint32_t sites, double isd)
{
  std::vector<Vector> c; c.reserve(sites);
  const double R = isd/std::sqrt(3.0);
  c.push_back(Vector(0,0,25));
  c.push_back(Vector(+R, 0, 25));
  c.push_back(Vector(+R/2, +0.5*isd, 25));
  c.push_back(Vector(-R/2, +0.5*isd, 25));
  c.push_back(Vector(-R, 0, 25));          // leftmost ≈ index 4
  c.push_back(Vector(-R/2, -0.5*isd, 25));
  c.push_back(Vector(+R/2, -0.5*isd, 25));
  return c;
}

int main (int argc, char** argv)
{
  // CLI
  CommandLine cmd;
  cmd.AddValue("simTime",   "Simulation time (s)",          kSimTimeSec);
  cmd.AddValue("isd",       "Inter-site distance (m)",      kIsd);
  cmd.AddValue("uePerEnb",  "UEs per eNB sector",           kUePerEnb);
  cmd.AddValue("speedKmph", "UE speed in km/h",             kSpeedKmph);
  cmd.AddValue("ttt",       "A3 Time-to-Trigger (ms)",      kTttMs);
  cmd.AddValue("hyst",      "A3 Hysteresis (dB)",           kHystDb);
  cmd.AddValue("tm",        "UE meas. interval Tm (ms)",    kTmMs); // not applied in 3.39
  cmd.AddValue("tf",        "Sliding window Tf (ms)",       kTfMs); // not applied in 3.39
  cmd.AddValue("fast",      "Quick mode (reduce size/time)",kFastMode);
  cmd.AddValue("demo",      "Deterministic traverse",       kDemo);
  cmd.AddValue("noAnim",    "Disable NetAnim for speed",    kNoAnim);

  LogComponentEnable("A3_Hex_21cells_ControlOnly", LOG_LEVEL_INFO);
  LogComponentEnable("LteUeRrc", LOG_LEVEL_INFO);
  LogComponentEnable("LteEnbRrc", LOG_LEVEL_INFO);
  LogComponentEnable("A3RsrpHandoverAlgorithm", LOG_LEVEL_INFO);

  cmd.Parse(argc, argv);

  NS_LOG_INFO("Starting simulation...");
  NS_LOG_INFO("Parameters: Time=" << kSimTimeSec << "s, ISD=" << kIsd
             << "m, UEs/enb=" << kUePerEnb << ", Speed=" << kSpeedKmph
             << " km/h, Hysteresis=" << kHystDb << " dB, TTT=" << kTttMs << " ms");

  if (kFastMode) {
    kUePerEnb   = std::min<uint32_t>(kUePerEnb, 3u);
    kSimTimeSec = std::min<double>(kSimTimeSec, 40.0);
  }

  RngSeedManager::SetSeed(12345);
  RngSeedManager::SetRun(1);

  // EPC/LTE
  Ptr<PointToPointEpcHelper> epc = CreateObject<PointToPointEpcHelper>();
  Ptr<LteHelper> lte = CreateObject<LteHelper>();
  
  // Set antenna model BEFORE SetEpcHelper (required when using EPC)
  lte->SetEnbAntennaModelType("ns3::ParabolicAntennaModel");
  lte->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(70.0));
  lte->SetEnbAntennaModelAttribute("MaxAttenuation", DoubleValue(20.0));
  
  lte->SetEpcHelper(epc);

  // ANR + Ideal RRC so neighbors are discovered and A3 config goes out
  lte->SetAttribute("AnrEnabled",  BooleanValue(true));
  lte->SetAttribute("UseIdealRrc", BooleanValue(true));

  // Pathloss: L = 128.1 + 37.6 * log10(R_km) where R is in km
  // For LogDistance: PL(d) = PL0 + 10*n*log10(d/d0)
  // Setting d0=1m, PL0=128.1+37.6*log10(0.001) = 128.1-112.8 = 15.3 dB at 1m
  // But standard formula uses d0=1km reference, so we use d0=1000m, PL0=128.1, n=3.76
  lte->SetPathlossModelType(TypeId::LookupByName("ns3::LogDistancePropagationLossModel"));
  lte->SetPathlossModelAttribute("ReferenceDistance", DoubleValue(1000.0)); // 1 km
  lte->SetPathlossModelAttribute("ReferenceLoss",     DoubleValue(128.1));
  lte->SetPathlossModelAttribute("Exponent",          DoubleValue(3.76));
  
  // Fading: Typical Urban (ETU - Extended Typical Urban)
  lte->SetAttribute("FadingModel", StringValue("ns3::TraceFadingLossModel"));
  lte->SetFadingModelAttribute("TraceFilename", 
      StringValue("src/lte/model/fading-traces/fading_trace_ETU_3kmph.fad"));
  lte->SetFadingModelAttribute("TraceLength", TimeValue(Seconds(10.0)));
  lte->SetFadingModelAttribute("SamplesNum", UintegerValue(10000));
  lte->SetFadingModelAttribute("WindowSize", TimeValue(Seconds(0.5)));
  lte->SetFadingModelAttribute("RbNum", UintegerValue(25)); // 25 RBs for 5 MHz

  // PHY + bandwidth
  Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(kEnbTxDbm));
  lte->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(kDlRb));
  lte->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(kUlRb));

  // A3 handover params
  lte->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
  lte->SetHandoverAlgorithmAttribute("Hysteresis",    DoubleValue(kHystDb));
  lte->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(kTttMs)));

  // Nodes
  NS_LOG_INFO("Creating " << 3*kSites << " eNBs across " << kSites << " sites (hex grid layout)...");
  NodeContainer enbNodes; enbNodes.Create(3 * kSites);              // 21 eNBs
  NodeContainer ueNodes;  ueNodes.Create(3 * kSites * kUePerEnb);   // many UEs

  // Give PGW a mobility model to silence NetAnim warnings
  {
    Ptr<Node> pgw = epc->GetPgwNode();
    MobilityHelper pgwMob; pgwMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    pgwMob.Install(pgw);
  }

  // eNB positions with proper 3-sector configuration
  // Each site has 3 sectors with different antenna orientations: 0°, 120°, -120°
  std::vector<Vector> centers = HexSiteCenters(kSites, kIsd);
  MobilityHelper me;
  Ptr<ListPositionAllocator> enbAlloc = CreateObject<ListPositionAllocator>();
  for (uint32_t s=0; s<kSites; ++s) {
    for (uint32_t k=0; k<3; ++k) {
      enbAlloc->Add(centers[s]); // All sectors at site center
    }
  }
  me.SetPositionAllocator(enbAlloc);
  me.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  me.Install(enbNodes);

  // Install eNBs with proper sectorization
  // Set antenna orientation for each sector: 0°, 120°, -120°
  NetDeviceContainer enbDevs;
  for (uint32_t n=0; n<enbNodes.GetN(); ++n) {
    uint32_t sector = n % 3;
    double orientation = (sector == 0) ? 0.0 : (sector == 1) ? 120.0 : -120.0;
    // Set orientation before installing this device
    lte->SetEnbAntennaModelAttribute("Orientation", DoubleValue(orientation));
    enbDevs.Add(lte->InstallEnbDevice(enbNodes.Get(n)));
  }

  // UE mobility: Paper specifies "constant speed to random direction (changing direction every 5s)"
  // Distribute 10 UEs randomly in front of each eNB
  Ptr<UniformRandomVariable> rvX = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> rvY = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> rvDir = CreateObject<UniformRandomVariable>();
  rvX->SetAttribute("Min", DoubleValue(-200.0));
  rvX->SetAttribute("Max", DoubleValue(200.0));
  rvY->SetAttribute("Min", DoubleValue(-200.0));
  rvY->SetAttribute("Max", DoubleValue(200.0));
  rvDir->SetAttribute("Min", DoubleValue(0.0));
  rvDir->SetAttribute("Max", DoubleValue(2.0 * M_PI));
  
  double v = kSpeedKmph / 3.6; // m/s
  uint32_t ueIdx = 0;
  
  if (kDemo) {
    // Deterministic mode: all UEs move right
    MobilityHelper mu; 
    mu.SetMobilityModel("ns3::ConstantVelocityMobilityModel"); 
    mu.Install(ueNodes);
    double startX = centers[4].x - 800.0; // Start left of leftmost site
  for (uint32_t u=0; u<ueNodes.GetN(); ++u) {
      double y = (int(u % 9) - 4) * 4.0;
    Ptr<ConstantVelocityMobilityModel> cv = ueNodes.Get(u)->GetObject<ConstantVelocityMobilityModel>();
    cv->SetPosition(Vector(startX - (u % 10) * 5.0, y, 1.5));
    cv->SetVelocity(Vector(v, 0, 0));
    }
  } else {
    // Use RandomWalk2dMobilityModel - generates CourseChange events for NetAnim
    Vector centerSite = centers[0];
    double minX = centerSite.x - kIsd * 1.2;
    double maxX = centerSite.x + kIsd * 1.2;
    double minY = centerSite.y - kIsd * 1.2;
    double maxY = centerSite.y + kIsd * 1.2;
    
    MobilityHelper mu;
    mu.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                        "Mode", StringValue("Time"),
                        "Time", StringValue("5s"),
                        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(v) + "]"),
                        "Bounds", RectangleValue(Rectangle(minX, maxX, minY, maxY)));
    
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t e=0; e<enbNodes.GetN() && ueIdx<ueNodes.GetN(); ++e) {
      Vector enbPos = centers[e/3];
      for (uint32_t u=0; u<kUePerEnb && ueIdx<ueNodes.GetN(); ++u, ++ueIdx) {
        double offsetX = rvX->GetValue();
        double offsetY = rvY->GetValue();
        posAlloc->Add(Vector(enbPos.x + offsetX, enbPos.y + offsetY, 1.5));
      }
    }
    mu.SetPositionAllocator(posAlloc);
    mu.Install(ueNodes);
  }

  // IP + devices
  InternetStackHelper ip; ip.Install(ueNodes);
  NetDeviceContainer ueDevs = lte->InstallUeDevice(ueNodes);
  Ipv4InterfaceContainer ueIfaces = epc->AssignUeIpv4Address(ueDevs);

  // X2 first, then attach (good practice for ANR/X2 readiness)
  lte->AddX2Interface(enbNodes);

  uint16_t port = 1234;
  UdpServerHelper server(port);
  auto sApps = server.Install(ueNodes.Get(0));
  sApps.Start(Seconds(1.0));
  sApps.Stop(Seconds(kSimTimeSec));
  g_udpServerApp = sApps.Get(0);

  UdpClientHelper client(ueIfaces.GetAddress(0), port);
  client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
  client.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
  client.SetAttribute("PacketSize", UintegerValue(1200));
  auto cApps = client.Install(epc->GetPgwNode());
  cApps.Start(Seconds(2.0));
  cApps.Stop(Seconds(kSimTimeSec));

  // Attach UEs to nearest eNB (paper: UEs distributed in front of each eNB)
  // For proper handover behavior, attach each UE to its nearest eNB initially
  NS_LOG_INFO("Attaching UEs to nearest eNBs based on initial position...");
  for (uint32_t i=0; i<ueDevs.GetN(); ++i) {
    Ptr<MobilityModel> ueMob = ueNodes.Get(i)->GetObject<MobilityModel>();
    Vector uePos = ueMob->GetPosition();
    
    // Find nearest eNB
    double minDist = 1e9;
    uint32_t nearestEnb = 0;
    for (uint32_t e=0; e<enbNodes.GetN(); ++e) {
      Ptr<MobilityModel> enbMob = enbNodes.Get(e)->GetObject<MobilityModel>();
      Vector enbPos = enbMob->GetPosition();
      double dist = std::sqrt((uePos.x-enbPos.x)*(uePos.x-enbPos.x) + 
                              (uePos.y-enbPos.y)*(uePos.y-enbPos.y));
      if (dist < minDist) {
        minDist = dist;
        nearestEnb = e;
      }
    }
    lte->Attach(ueDevs.Get(i), enbDevs.Get(nearestEnb));
  }

  // HO events
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart", MakeCallback(&OnHoStart));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",  MakeCallback(&OnHoEndOk));

  // NetAnim - AnimationInterface automatically tracks CourseChange events
  if (!kNoAnim) {
    AnimationInterface anim("results/handover_demo.xml");
    anim.SetMobilityPollInterval(Seconds(0.05));
    anim.SetMaxPktsPerTraceFile(1000000);
    
    // Color and label eNBs by site and sector
  for (uint32_t i=0; i<enbNodes.GetN(); ++i) {
      uint32_t site = i / 3;
      uint32_t sector = i % 3;
      std::string desc = "Site" + std::to_string(site) + "_Sector" + std::to_string(sector) + "_eNB" + std::to_string(i);
      anim.UpdateNodeDescription(enbNodes.Get(i), desc);
      // Different colors for different sites
      uint8_t r = (site * 30) % 255;
      uint8_t g = 102;
      uint8_t b = 204;
      anim.UpdateNodeColor(enbNodes.Get(i), r, g, b);
      anim.UpdateNodeSize(enbNodes.Get(i), 12, 12);
  }
    
    // Color and label UEs
  for (uint32_t i=0; i<ueNodes.GetN(); ++i) {
      std::string desc = "UE" + std::to_string(i) + "_Speed" + std::to_string((int)kSpeedKmph) + "kmph";
      anim.UpdateNodeDescription(ueNodes.Get(i), desc);
      anim.UpdateNodeColor(ueNodes.Get(i), 255, 0, 0); // Red for UEs
      anim.UpdateNodeSize(ueNodes.Get(i), 8, 8);
    }
  }

  Simulator::Stop(Seconds(kSimTimeSec));
  g_csv.open("results/handover_timeseries.csv");
  g_csv << "time_s,throughput_mbps,loss_frac,jitter_ms,rx_pkts,delta_pkts\n";
  Simulator::Schedule(Seconds(g_sampleInterval), &SampleThroughput);
  g_hoEvents.open("ho_events.csv");

  Simulator::Run();

  // KPIs
  const double users = ueNodes.GetN();
  const double hoPerUserPerSec = (users>0 && kSimTimeSec>0) ? (double(g_totalHo)/users/kSimTimeSec) : 0.0;
  const double pingPongRate = (g_totalHo>0) ? (double(g_pingPong)/double(g_totalHo)) : 0.0;

  // Output in parseable format for scripts
  NS_LOG_INFO("Simulation complete. Computing KPIs...");
  NS_LOG_INFO("Total Handover Events: " << g_totalHo);
  NS_LOG_INFO("Ping-Pong Handover Events: " << g_pingPong);
  NS_LOG_INFO("HO per user per sec: " << hoPerUserPerSec);
  NS_LOG_INFO("Ping-pong rate: " << pingPongRate);

  std::cout << "[KPI] speed_kmph=" << kSpeedKmph
            << " TTT_ms=" << kTttMs
            << " Hyst_dB=" << kHystDb
            << " Tf_ms=" << kTfMs << " (accepted, not applied in 3.39)"
            << " Tm_ms=" << kTmMs << " (accepted, not applied in 3.39)"
            << " HO_total=" << g_totalHo
            << " HO_per_user_per_sec=" << hoPerUserPerSec
            << " pingpong_rate=" << pingPongRate << "\n";
  
  // CSV format for easy parsing
  std::cout << "CSV," << kSpeedKmph << "," << kTttMs << "," << kHystDb << "," 
            << kTfMs << "," << hoPerUserPerSec << "," << pingPongRate << "," 
            << g_totalHo << "," << g_pingPong << "\n";

  std::ofstream ho("ho_summary.csv", std::ios::app);
  if (ho.tellp() == 0) {
    ho << "speed_mps,hysteresis_db,ttt_ms,num_ues,sim_time_s,ho_total,ho_per_user_per_sec\n";
  }
  ho << kSpeedKmph / 3.6 << "," << kHystDb << "," << kTttMs << ","
     << ueNodes.GetN() << "," << kSimTimeSec << "," << g_totalHo << "," << hoPerUserPerSec << "\n";

  g_hoEvents.close();

  Simulator::Destroy();
  return 0;
}
