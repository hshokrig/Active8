/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008,2009 IITP RAS
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
 * Author: Kirill Andreev <andreev@iitp.ru>
 *
 *
 * By default this script creates m_xSize * m_ySize square grid topology with
 * IEEE802.11s stack installed at each node with peering management
 * and HWMP protocol.
 * The side of the square cell is defined by m_step parameter.
 * When topology is created, UDP ping is installed to opposite corners
 * by diagonals. packet size of the UDP ping and interval between two
 * successive packets is configurable.
 * 
 *  m_xSize * step
 *  |<--------->|
 *   step
 *  |<--->|
 *  * --- * --- * <---Ping sink  _
 *  | \   |   / |                ^
 *  |   \ | /   |                |
 *  * --- * --- * m_ySize * step |
 *  |   / | \   |                |
 *  | /   |   \ |                |
 *  * --- * --- *                _
 *  ^ Ping source
 *
 *  See also MeshTest::Configure to read more about configurable
 *  parameters.
 */


#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/netanim-module.h"

#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/gnuplot.h"
#include "ns3/random-variable-stream.h"



#include <iostream>
#include <sstream>
#include <fstream>

#include <string>
#include <cassert>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TestMeshScript");

class MeshTest
{
public:
  /// Init test
  MeshTest ();
  /// Configure test from command line arguments
  void Configure (int argc, char ** argv);
  /// Run test
  int Run ();
private:
  
  int       m_clients;
  int       m_routers;
  double    m_step;
  double    m_randomStart;
  double    m_totalTime;
  double    m_min;
  double    m_max;
  double    m_packetInterval;
  uint16_t  m_packetSize;
  double    m_rss;
  uint32_t  m_nIfaces;
  bool      m_chan;
  bool      m_pcap;

  std::string m_stack;
  std::string m_root;
  /// List of network nodes
  NodeContainer clients;
  NodeContainer routers;
  /// List of all mesh point devices
  NetDeviceContainer meshClients;
  NetDeviceContainer meshRouters;

  //Addresses of interfaces:
  Ipv4InterfaceContainer interfacesClients;
  Ipv4InterfaceContainer interfacesRouters;

  // MeshHelper. Report is not static methods
  MeshHelper mesh;


private:
  /// Create nodes and setup their mobility
  void CreateNodes ();
  /// Install internet m_stack on nodes
  void InstallInternetStack ();
  /// Install applications
  void InstallApplication ();
  /// Print mesh devices diagnostics
  void Report ();
};


MeshTest::MeshTest () :

  //m_xSize (3), //3
  //m_ySize (4), //3
  m_clients (4),
  m_routers (8),
  m_step (100.0), //100
  m_randomStart (0.1),
  m_totalTime (30), //100
  //m_packetInterval (1), //0.1
  m_min(0.1),
  m_max(5.0),
  m_packetInterval(0),
  m_packetSize (1024),
  m_rss(-80), // -dBm
  m_nIfaces (2),
  m_chan (true),
  m_pcap (false),
  m_stack ("ns3::Dot11sStack"),
  m_root ("ff:ff:ff:ff:ff:ff")
{
}

void
MeshTest::Configure (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.AddValue ("n_clients", "Number of clients in row a grid. [6]", m_clients);
  cmd.AddValue ("n_routers", "Number of routers in a grid. [6]", m_routers);
  cmd.AddValue ("step",   "Size of edge in our grid, meters. [100 m]", m_step);
  /*
   * As soon as starting node means that it sends a beacon,
   * simultaneous start is not good.
   */
  cmd.AddValue ("start",  "Maximum random start delay, seconds. [0.1 s]", m_randomStart);
  cmd.AddValue ("time",  "Simulation time, seconds [50 s]", m_totalTime);
  //cmd.AddValue ("packet-interval",  "Interval between packets in UDP ping, seconds [0.001 s]", m_packetInterval);
  cmd.AddValue ("mim-packet-interval",  "Minimum Interval between packets in UDP ping, seconds [0.001 s]", m_min);
  cmd.AddValue ("max-packet-interval",  "Maximum Interval between packets in UDP ping, seconds [0.001 s]", m_max);
  cmd.AddValue ("packet-size",  "Size of packets in UDP ping", m_packetSize);
  cmd.AddValue ("rss", "received signal strength", m_rss);
  cmd.AddValue ("interfaces", "Number of radio interfaces used by each mesh point. [1]", m_nIfaces);
  cmd.AddValue ("channels",   "Use different frequency channels for different interfaces. [0]", m_chan);
  cmd.AddValue ("pcap",   "Enable PCAP traces on interfaces. [0]", m_pcap);
  cmd.AddValue ("stack",  "Type of protocol stack. ns3::Dot11sStack by default", m_stack);
  cmd.AddValue ("root", "Mac address of root mesh point in HWMP", m_root);

  cmd.Parse (argc, argv);
  NS_LOG_DEBUG ("Grid:" << m_clients << "+" << m_routers);
  NS_LOG_DEBUG ("Simulation time: " << m_totalTime << " s");
}



void
MeshTest::CreateNodes ()
{ 
  /*
   * Create m_ySize*m_xSize stations to form a grid topology
   */
  clients.Create (m_clients);
  routers.Create (m_routers);
  //nodes.Create (m_clients + m_routers);
  std::cout<<"There are  "<< (m_clients + m_routers) << " in the network." << std::endl;


  // Configure YansWifiChannel
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  wifiPhy.Set ("RxGain", DoubleValue (0) ); 

  //YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless of the distance between the two stations, and the transmit power
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (m_rss));
  wifiPhy.SetChannel (wifiChannel.Create ());



  /*
   * Create mesh helper and set stack installer to it
   * Stack installer creates all needed protocols and install them to
   * mesh point device
   */
  mesh = MeshHelper::Default ();
  if (!Mac48Address (m_root.c_str ()).IsBroadcast ())
    {
      mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
    }
  else
    {
      //If root is not set, we do not use "Root" attribute, because it
      //is specified only for 11s
      mesh.SetStackInstaller (m_stack);
    }
  


  if (m_chan)
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
    }
  else
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
    }
  mesh.SetMacType ("RandomStart", TimeValue (Seconds (m_randomStart)));
  // Set number of interfaces - default is single-interface mesh point
  mesh.SetNumberOfInterfaces (m_nIfaces);
  // Install protocols and return container if MeshPointDevices
 
  meshClients = mesh.Install (wifiPhy, clients);
  meshRouters = mesh.Install (wifiPhy, routers);
  // Setup mobility - static grid topology
  
  MobilityHelper mobility_routers;
 
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));//Max=300
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));//Max=1500
  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  //mobility_routers.SetMobilityModel ("ns3::RandomWaypointMobilityModel","PositionAllocator", PointerValue (taPositionAlloc));
  mobility_routers.SetPositionAllocator (taPositionAlloc);
  mobility_routers.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle (-500, 500, -500, 500)));
  mobility_routers.Install (routers);
 
  MobilityHelper mobility_clients;
  mobility_clients.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (100.0),
                                 "MinY", DoubleValue (100.0),
                                 "DeltaX", DoubleValue (m_step),
                                 "DeltaY", DoubleValue (m_step),
                                 "GridWidth", UintegerValue (2),
                                 "LayoutType", StringValue ("RowFirst"));
  //mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_clients.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle (0, 300, 0, 300)));
  mobility_clients.Install (clients);




  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp-"));
}

  
void
MeshTest::InstallInternetStack ()
{
  InternetStackHelper internetStack;
  internetStack.Install (clients);
  internetStack.Install (routers);


  Ipv4AddressHelper address;
  NS_LOG_INFO ("Assign IP Addresses.");
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfacesClients = address.Assign (meshClients);
  address.SetBase ("10.1.10.0", "255.255.255.0");
  interfacesRouters = address.Assign (meshRouters);

}


void
MeshTest::InstallApplication ()
{

  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  x->SetAttribute ("Min", DoubleValue (m_min));
  x->SetAttribute ("Max", DoubleValue (m_max));
  // The values returned by a uniformly distributed random
  // variable should always be within the range
  //
  //     [min, max)  .
  //
  m_packetInterval = x->GetValue ();
  //int valueInt = x->GetInteger ();

  
  
  UdpEchoServerHelper echoServer (9);
  ApplicationContainer serverApps = echoServer.Install (clients.Get (3));
  serverApps.Start (Seconds (0.0));
  serverApps.Stop (Seconds (m_totalTime));

  UdpEchoClientHelper echoClient (interfacesClients.GetAddress (0), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t) (m_totalTime*(1/m_packetInterval))));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (m_packetInterval)));
  std::cout<<"Interval is: "<< m_packetInterval << " seconds" << std::endl;
  echoClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));  
  ApplicationContainer clientApps = echoClient.Install (clients.Get (0));



  clientApps.Start (Seconds (0.0));
  clientApps.Stop (Seconds (m_totalTime));
  
}


int
MeshTest::Run ()
{
  CreateNodes ();
  InstallInternetStack ();
  InstallApplication ();
  Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this);
  Simulator::Stop (Seconds (m_totalTime));


  AnimationInterface anim("mesh.xml");
  //---------------------------------------------------------------
  for (uint32_t i=0; i< clients.GetN(); ++i)
     {
        anim.UpdateNodeDescription (clients.Get (i), "");
        anim.UpdateNodeColor (clients.Get(i), 0, 250, 0);   
     }
  for (uint32_t i=0; i< routers.GetN(); ++i)
     {
        anim.UpdateNodeDescription (routers.Get (i), "");
        anim.UpdateNodeColor (routers.Get(i), 0, 0, 255);   
     }


  anim.EnableIpv4RouteTracking ("meshtrace.xml", Seconds(0), Seconds(5), Seconds(0.25));
  anim.EnableWifiMacCounters(Seconds(0), Seconds(m_totalTime));
  anim.EnableWifiPhyCounters(Seconds(0), Seconds(m_totalTime));

  
  Simulator::Stop (Seconds (m_totalTime));

  Simulator::Run ();

  //Flow Monitor
  /*FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

      uint32_t txPacketsum = 15;
      uint32_t rxPacketsum = 10;
      uint32_t DropPacketsum = 10;
      uint32_t LostPacketsum = 10;
      uint32_t rxBytessum = 15;
      uint32_t Delaysum = 0;

  std::ofstream ofs ("ResultGragh.plf", std::ofstream::out);


  for(std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end(); ++iter)

     {
         txPacketsum += iter->second.txPackets;
         rxPacketsum += iter->second.rxPackets;
         DropPacketsum += iter->second.packetsDropped.size();
         LostPacketsum += iter->second.lostPackets;
         Delaysum += iter->second.delaySum.GetSeconds();
         rxBytessum += iter->second.rxBytes;

         NS_LOG_UNCOND("Throughput: " <<iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/ 1024 << "mbps");
         NS_LOG_UNCOND("Tx Packets: " <<iter ->second.txPackets);
         NS_LOG_UNCOND("Rx Packets: " <<iter ->second.rxPackets);
         NS_LOG_UNCOND("Delay: " <<iter ->second.delaySum.GetSeconds());

         monitor->SerializeToXmlFile("lablab.flowmon1.xml", true, true);
     }

       NS_LOG_UNCOND("Average PDR: " <<((rxPacketsum * 100)/ txPacketsum) << " ");
       NS_LOG_UNCOND("Average jitter: " << ((LostPacketsum * 100)/ txPacketsum) << " ");
       NS_LOG_UNCOND("Average Throughput: " << ((rxBytessum * 8.0) / (m_totalTime)) / 1024 / 4 << "mbps");
       NS_LOG_UNCOND("Average Delay: " << (Delaysum / rxPacketsum) * 1000 << "ms" << "\n");


       ofs << "set terminal png" << std::endl;
       ofs << "set output ResultGraph.png" << std::endl;
       ofs << "set title" << std::endl;
       ofs << "set xlabel nodes" << std::endl;
       ofs << "set ylabel value" << std::endl;
       ofs << "plot" << "'-'" << "title" << "'packet_inter_arrival_time(ms)' with linespoints, '-' title 'jitter' with lines, '-' title 'Throughput' with lines, '-' title 'delay' with lines"<< std::endl;
       ofs << "1" <<0<< std::endl;
       ofs << (m_clients + m_routers) <<"" << Seconds(m_packetInterval)/(10000*10000) << std::endl;
       ofs << "e" << std::endl;
       ofs << "1" <<0 << std::endl;
       ofs << (m_clients + m_routers) <<"" << ((LostPacketsum * 100) / txPacketsum) << std::endl;
       ofs << "e" << std::endl;
       ofs << (m_clients + m_routers) <<"" << ((rxBytessum * 8.0) / (m_totalTime))/1024/4 << std::endl;
       ofs << "e" << std::endl;
       ofs << "1" <<0 << std::endl;
       ofs << (m_clients + m_routers) <<"" << (Delaysum / rxPacketsum)*1000 << std::endl;
       ofs << "e" << std::endl;
       ofs.close();

     //--------------------------------------------------------------------
*/
  Simulator::Destroy ();
  return 0;
}


void
MeshTest::Report ()
{
  unsigned n (0);
  for (NetDeviceContainer::Iterator i = meshClients.Begin (); i != meshClients.End (); ++i, ++n)
    {
      std::ostringstream os;
      os << "client-mp-report-" << n << ".txt";
      std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str () << "\n";
      std::ofstream of;
      of.open (os.str ().c_str ());
      if (!of.is_open ())
        {
          std::cerr << "Error: Can't open file " << os.str () << "\n";
          return;
        }
      mesh.Report (*i, of);
      of.close ();
    }


  unsigned m (0);
  for (NetDeviceContainer::Iterator i = meshRouters.Begin (); i != meshRouters.End (); ++i, ++m)
    {
      std::ostringstream os;
      os << "router-mp-report-" << m << ".txt";
      std::cerr << "Printing mesh point device #" << m << " diagnostics to " << os.str () << "\n";
      std::ofstream of;
      of.open (os.str ().c_str ());
      if (!of.is_open ())
        {
          std::cerr << "Error: Can't open file " << os.str () << "\n";
          return;
        }
      mesh.Report (*i, of);
      of.close ();
    }


}
int
main (int argc, char *argv[])
{
  MeshTest t; 
  t.Configure (argc, argv);
  return t.Run ();
}
