/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
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

// 
// This script configures two nodes on an 802.11b physical layer, with
// 802.11b NICs in adhoc mode, and by default, sends one packet of 1000 
// (application) bytes to the other node.  The physical layer is configured
// to receive at a fixed RSS (regardless of the distance and transmit
// power); therefore, changing position of the nodes has no effect. 
//
// There are a number of command-line options available to control
// the default behavior.  The list of available command-line options
// can be listed with the following command:
// ./waf --run "wifi-simple-adhoc --help"
//
// For instance, for this configuration, the physical layer will
// stop successfully receiving packets when rss drops below -97 dBm.
// To see this effect, try running:
//
// ./waf --run "wifi-simple-adhoc --rss=-97 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-98 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-99 --numPackets=20"
//
// Note that all ns-3 attributes (not just the ones exposed in the below
// script) can be changed at command line; see the documentation.
//
// This script can also be helpful to put the Wifi layer into verbose
// logging mode; this command will turn on all wifi logging:
// 
// ./waf --run "wifi-simple-adhoc --verbose=1"
//
// When you are done, you will notice two pcap trace files in your directory.
// If you have tcpdump installed, you can try this:
//
// tcpdump -r wifi-simple-adhoc-0-0.pcap -nn -tt
//

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/applications-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/dsdv-module.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhoc");


struct txrec {
  Time sts;
  uint64_t pktUid;
};

std::vector<txrec> tx_list;
uint32_t m_recvBytes=0;
Time totalDelay;
uint32_t throughput;
uint16_t numrecv=0;
uint16_t numsend=0;
Time FirstTx ;
Time LastRx;

void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> pk;
  uint64_t pk_uid; 
  while ((pk = socket->Recv ()))
    {
      
      //NS_LOG_UNCOND ("Received one packet!");      
      m_recvBytes += pk->GetSize();
      std::cout<< m_recvBytes << " bytes received." << std::endl;
      pk_uid = pk->GetUid();
      //std::cout<< "The uid is: "<< pk_uid << std::endl;
    }

   Time latency;   
   Time LastTx;
   
   for(uint16_t i = 0; i < tx_list.size(); i++ ) {
     if(tx_list.at(i).pktUid==pk_uid) { 
          latency =  Simulator::Now()-tx_list.at(i).sts;           
          //std::cout<<"The packet:"<< pk_uid << " latency is "<< latency.GetMicroSeconds() <<" us." <<std::endl;
          totalDelay += latency;
          //std::cout<<"Total delay is "<< totalDelay.GetMicroSeconds()<<" us."  <<std::endl; 
          FirstTx= tx_list.at(0).sts;         
          LastTx = tx_list.at(i).sts;
          //std::cout<<"First tx time is "<< FirstTx.GetMicroSeconds() <<" us."  <<std::endl;
          //std::cout<<"Last tx time is "<< LastTx.GetMicroSeconds()<<" us."  <<std::endl;

          numrecv++;
          std::cout<< numrecv <<" packets have been received." <<std::endl;
          //tx_list.erase(tx_list.begin()+i);
          break;
      }
    }//for
 
   std::cout<<"*************************Results*************************"  <<std::endl;  
   std::cout<<"Total delay is "<< totalDelay.GetMicroSeconds()<<" us."  <<std::endl;
   std::cout<<"Average delay is "<< (totalDelay.GetMicroSeconds()/numrecv) <<" us."  <<std::endl;
   LastRx = LastTx + latency; 
   std::cout<<"Total simulation time is "<< (LastRx.GetMicroSeconds() - FirstTx.GetMicroSeconds()) <<" us."  <<std::endl;
   throughput =  m_recvBytes * 8.0*1000000/( (LastRx.GetMicroSeconds() - FirstTx.GetMicroSeconds())*1024);
   std::cout<<"Throughput is "<< throughput <<" Kbps."  <<std::endl;
}
 

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {

     double     mean = 1024;
     double     bound = 0.0; 
     Ptr<ExponentialRandomVariable> x = CreateObject<ExponentialRandomVariable> ();
     x->SetAttribute ("Mean", DoubleValue (mean));
     x->SetAttribute ("Bound", DoubleValue (bound));
     uint32_t packetSize = (unsigned int) x->GetValue ();     

      Ptr<Packet> pk = Create<Packet>(packetSize);
      struct txrec t = {Simulator::Now(),pk->GetUid()};
      tx_list.push_back(t);
      socket->Send (pk);
      numsend++;
      Simulator::Schedule (pktInterval, &GenerateTraffic, 
                           socket,pktCount-1, pktInterval);
      std::cout<<"The Poisson arrival packet size is "<< packetSize << " bytes" << std::endl;
      std::cout<< numsend *2 <<" packets have been sent." <<std::endl;
      
    }
  else
    {
      socket->Close ();
      std::cout<<"Socket closed." << std::endl;
    }
  
}





int main (int argc, char *argv[])
{
  
  std::string phyMode ("DsssRate1Mbps");
  double     rss = -80;  // -dBm
  uint32_t   numPackets = 1000;
  int        arrivalRate = 5;
  double     interval; // seconds
  uint16_t   n_nodes = 12;


  uint32_t   m_nIfaces =1;
  bool       m_chan = true;
  double     m_randomStart =0.1;
  
  std::string m_stack ="ns3::Dot11sStack";
  std::string m_root ="ff:ff:ff:ff:ff:ff";

  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("arrivalRate", "arrivalRate each second", arrivalRate);
  cmd.Parse (argc, argv);

  

  NodeContainer nodes;
  nodes.Create (n_nodes);
  MeshHelper mesh;

  NetDeviceContainer meshDevices;/////////
  Ipv4InterfaceContainer interfaces;//////////

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
  wifiPhy.SetChannel (wifiChannel.Create ());

 
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
  meshDevices = mesh.Install (wifiPhy, nodes);


  MobilityHelper mobility;
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  
  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  //mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel","PositionAllocator", PointerValue (taPositionAlloc));
  mobility.SetPositionAllocator (taPositionAlloc);


  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle (0, 500, 0, 500)));



  mobility.Install (nodes);

  AsciiTraceHelper ascii;
  mobility.EnableAsciiAll (ascii.CreateFileStream ("wifimesh4-mobility.tr"));
  




  InternetStackHelper internetStack;
  internetStack.Install (nodes);
  Ipv4AddressHelper address;
  NS_LOG_INFO ("Assign IP Addresses.");
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign (meshDevices); 

  NS_LOG_INFO ("Create sockets.");
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");  
  Ptr<Socket> recvSink;
  Ptr<Socket> source;
  
    recvSink = Socket::CreateSocket (nodes.Get (nodes.GetN()-1), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    recvSink->Bind (local);
    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
  
    source = Socket::CreateSocket (nodes.Get (0), tid);
    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    source->SetAllowBroadcast (true);
    source->Connect (remote);


   



    Ptr<Socket> recvSink1;
    Ptr<Socket> source1;
  
    recvSink1 = Socket::CreateSocket (nodes.Get (nodes.GetN()-2), tid);
    InetSocketAddress local1 = InetSocketAddress (Ipv4Address::GetAny (), 80);
    recvSink1->Bind (local1);
    recvSink1->SetRecvCallback (MakeCallback (&ReceivePacket));
  
    source1 = Socket::CreateSocket (nodes.Get (1), tid);
    InetSocketAddress remote1 = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    source1->SetAllowBroadcast (true);
    source1->Connect (remote1);











   NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss );
   
    interval = 1./arrivalRate;
    std::cout<<"Interval is: "<< interval << " s."<< std::endl;
    Time interPacketInterval = Seconds (interval);
    Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  interPacketInterval, &GenerateTraffic, 
                                  source, numPackets, interPacketInterval);

    numPackets = numPackets - arrivalRate;


  
  Simulator::Stop (Seconds (50));

   


  AnimationInterface anim("wifimesh4.xml");
    //---------------------------------------------------------------
    anim.UpdateNodeDescription (nodes.Get (0), "");
    anim.UpdateNodeColor (nodes.Get(0), 255, 0, 0); //Source red

    anim.UpdateNodeDescription (nodes.Get (nodes.GetN()-1), "");
    anim.UpdateNodeColor (nodes.Get(nodes.GetN()-1), 0, 0, 255); //Receiver blue


    anim.UpdateNodeDescription (nodes.Get (1), "");
    anim.UpdateNodeColor (nodes.Get(1), 200, 100, 0); //Source red

    anim.UpdateNodeDescription (nodes.Get (nodes.GetN()-2), "");
    anim.UpdateNodeColor (nodes.Get(nodes.GetN()-2), 0, 100, 200); //Receiver blue




    
    for (uint32_t i=2; i< nodes.GetN()-2; ++i)
       {
          anim.UpdateNodeDescription (nodes.Get (i), "");
          anim.UpdateNodeColor (nodes.Get(i), 0, 255, 0);   
       }


    anim.EnableIpv4RouteTracking ("wifimesh4trace.xml", Seconds(0), Seconds(5), Seconds(0.25));
    anim.EnableWifiMacCounters(Seconds(0), Seconds(50));
    anim.EnableWifiPhyCounters(Seconds(0), Seconds(50));
  
    Simulator::Stop (Seconds (50));






  Simulator::Run ();
  Simulator::Destroy ();
}



