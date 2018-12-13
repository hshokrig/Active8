/*
*                ----------------------------  ...................
*                |       Application        |    
*                ----------------------------  ...................
*                |   UDP/TCP  |   ICMPv6    |
*                ----------------------------  ...................
*                |          IPv6            |
*                ----------------------------
*                |                          |
*                |    Lr-WPAN Adaptation    |   ZigBee Network Layer
*                |          Layer           | 
*                |                          |
*                |                          |
*                ----------------------------  ...................
*                |       LR-WPAN MAC        |         Mesh
*                ----------------------------     IEEE 802.15.4
*                |       LR-WPAN PHY        |        CSMA/CA
*                ----------------------------  ...................
*/



#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-module.h"
#include "ns3/applications-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/aodv-module.h"



#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("802.15.4_SimpleAdhoc");


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
 

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktCount, Time pktInterval ,double mean, double pktsize)
{
  if (pktCount > 0)
    {

     std::cout<<"Current interval  is: "<< pktInterval.GetSeconds() << " s."<< std::endl;
     
     double     bound = 0.0; 
     Ptr<ExponentialRandomVariable> x = CreateObject<ExponentialRandomVariable> ();
     x->SetAttribute ("Mean", DoubleValue (mean));
     x->SetAttribute ("Bound", DoubleValue (bound));
     double rate = (unsigned int) x->GetValue ();
     double inter = 1./rate;
     pktInterval = Seconds (inter);
     

      Ptr<Packet> pk = Create<Packet>(pktsize);
      struct txrec t = {Simulator::Now(),pk->GetUid()};
      tx_list.push_back(t);
      socket->Send (pk);
      numsend++;
      Simulator::Schedule (pktInterval, &GenerateTraffic, 
                           socket,pktCount-1, pktInterval, mean, pktsize);
      
      std::cout<< numsend  <<" packets have been sent." <<std::endl;
      
    }
  else
    {
      socket->Close ();
      std::cout<<"Socket closed." << std::endl;
    }
  
}





int main (int argc, char *argv[])
{
  
  
  double     rss = -80;  // -dBm
  uint32_t   numPackets = 500;
  int        arrivalRate = 20;
  double     interval; // seconds
  uint16_t   n_nodes = 12;
  double     packetSize = 1024;


//  uint32_t   m_nIfaces =1;
//  bool       m_chan = true;
//  double     m_randomStart =0.1;
  double     n_mean = 20;
  std::string m_stack ="ns3::Dot11sStack";
  std::string m_root ="ff:ff:ff:ff:ff:ff";

  CommandLine cmd;
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("packetSize", "the size of packets generated", packetSize);
  cmd.AddValue ("n_mean", "mean value of arrivalRate each second", n_mean);
  cmd.Parse (argc, argv);

  

  NodeContainer nodes;
  nodes.Create (n_nodes);

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

 
  AodvHelper aodv;
  LrWpanHelper lrWpan;
  Ping6Helper ping6;


  NetDeviceContainer lrwpanDevices;/////////
  lrwpanDevices = lrWpan.Install (nodes);
  // Fake PAN association and short address assignment.
  lrWpan.AssociateToPan (lrwpanDevices, 0);

  InternetStackHelper internetStack;
  internetStack.SetRoutingHelper (aodv); // has effect on the next Install ()
  internetStack.Install (nodes);

  //MeshHelper mesh;
  //NetDeviceContainer meshDevices = mesh.Install (lrwpanDevices); 
  





  Ipv6AddressHelper address;
  address.SetBase (Ipv6Address ("2001:2::"), Ipv6Prefix (64));
  Ipv6InterfaceContainer interfaces;//////////
  interfaces = address.Assign (lrwpanDevices); 

  // check if adresses are assigned
  std::cout<< interfaces.GetAddress(0,1)<<std::endl;
  std::cout<< interfaces.GetAddress(nodes.GetN()-1,1)<<std::endl;


  ping6.SetLocal (interfaces.GetAddress (0, 1));
  ping6.SetRemote (interfaces.GetAddress (nodes.GetN()-1, 1));

  




/*  mesh = MeshHelper::Default ();
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
*/


  
  AsciiTraceHelper ascii;
  mobility.EnableAsciiAll (ascii.CreateFileStream ("zigbeemesh-mobility.tr"));
  lrWpan.EnableAsciiAll (ascii.CreateFileStream ("zigbeemesh.tr"));
  lrWpan.EnablePcapAll (std::string ("zigbeemesh"), true);
  
 

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

  ApplicationContainer apps = ping6.Install (nodes.Get (0));

  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (50.0));


   


   NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss );
   
    interval = 1./arrivalRate;
    Time interPacketInterval = Seconds (interval);
    Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  interPacketInterval, &GenerateTraffic, 
                                  source, numPackets, interPacketInterval, n_mean, packetSize);

    numPackets = numPackets - arrivalRate;


  Simulator::Stop (Seconds (50));

   


  AnimationInterface anim("zigbeemesh.xml");
    //---------------------------------------------------------------
    anim.UpdateNodeDescription (nodes.Get (0), "");
    anim.UpdateNodeColor (nodes.Get(0), 255, 0, 0); //Source red

    anim.UpdateNodeDescription (nodes.Get (nodes.GetN()-1), "");
    anim.UpdateNodeColor (nodes.Get(nodes.GetN()-1), 0, 0, 255); //Receiver blue



    
    for (uint32_t i=1; i< nodes.GetN()-1; ++i)
       {
          anim.UpdateNodeDescription (nodes.Get (i), "");
          anim.UpdateNodeColor (nodes.Get(i), 0, 255, 0);   
       }


    anim.EnableIpv4RouteTracking ("zigbeemesh.xml", Seconds(0), Seconds(5), Seconds(0.25));
    anim.EnableWifiMacCounters(Seconds(0), Seconds(50));
    anim.EnableWifiPhyCounters(Seconds(0), Seconds(50));
  
    Simulator::Stop (Seconds (50));






  Simulator::Run ();
  Simulator::Destroy ();
}



