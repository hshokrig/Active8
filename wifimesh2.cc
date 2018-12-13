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
Time FirstTx ;


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
   std::cout<<"Total delay is "<< totalDelay.GetMicroSeconds()<<" us."  <<std::endl;
   std::cout<<"Average delay is "<< (totalDelay.GetMicroSeconds()/numrecv) <<" us."  <<std::endl;
   Time LastRx = LastTx + latency; 
   std::cout<<"Total simulation time is "<< (LastRx.GetMicroSeconds() - FirstTx.GetMicroSeconds()) <<" us."  <<std::endl;
   throughput =  m_recvBytes * 8.0*1000000/( (LastRx.GetMicroSeconds() - FirstTx.GetMicroSeconds())*1024);
   std::cout<<"Throughput is "<< throughput <<" Kbps."  <<std::endl;
}
 

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, 
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      Ptr<Packet> pk = Create<Packet>(pktSize);
      struct txrec t = {Simulator::Now(),pk->GetUid()};
      tx_list.push_back(t);
      socket->Send (pk);
      Simulator::Schedule (pktInterval, &GenerateTraffic, 
                           socket, pktSize,pktCount-1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
  
}

uint32_t PacketSize()
{
  
  double     mean = 1024;
  double     bound = 0.0; 
  Ptr<ExponentialRandomVariable> x = CreateObject<ExponentialRandomVariable> ();
     x->SetAttribute ("Mean", DoubleValue (mean));
     x->SetAttribute ("Bound", DoubleValue (bound));
     uint32_t packetSize = (unsigned int) x->GetValue ();
     std::cout<<"It is a Poisson arrival packet size is "<< packetSize << " bytes" << std::endl;
  
  return  packetSize;
}



int main (int argc, char *argv[])
{
  
  std::string phyMode ("DsssRate1Mbps");
  double     rss = -80;  // -dBm
  uint32_t   numPackets = 50;
  int        arrivalRate = 5;
  double     interval; // seconds
  uint16_t   n_nodes = 10;


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


  MobilityHelper mobilityAdhoc;
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "PositionAllocator", PointerValue (taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  mobilityAdhoc.Install (nodes);

  InternetStackHelper internetStack;
  internetStack.Install (nodes);
  Ipv4AddressHelper address;
  NS_LOG_INFO ("Assign IP Addresses.");
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign (meshDevices); 

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.01]"));
  onoff1.SetAttribute("DataRate", StringValue ("54Mbps"));
 
  
  Ptr<Socket> recvSink;
  Ptr<Socket> source;

  
    recvSink = Socket::CreateSocket (nodes.Get (9), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    recvSink->Bind (local);
    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
  
    source = Socket::CreateSocket (nodes.Get (0), tid);
    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    source->SetAllowBroadcast (true);
    source->Connect (remote);

   ApplicationContainer clientApps = onoff1.Install (nodes.Get (0));
   clientApps.Start (Seconds (0.0));
   clientApps.Stop (Seconds (10.0));

   NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss );
   
    interval = 1./arrivalRate;
     std::cout<<"Interval is: "<< interval << " s."<< std::endl;
     Time interPacketInterval = Seconds (interval);


if (numPackets>0){

     Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  interPacketInterval, &GenerateTraffic, 
                                  source, PacketSize(), numPackets, interPacketInterval);

     numPackets = numPackets - arrivalRate;

   

  }//if

  



  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

