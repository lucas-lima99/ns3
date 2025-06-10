/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
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
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "ns3/lora-channel.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/object-factory.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include <algorithm>
#include "ns3/lora-phy-helper.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"



namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("LoraChannel");

NS_OBJECT_ENSURE_REGISTERED (LoraChannel);

TypeId
LoraChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LoraChannel")
    .SetParent<Channel> ()
    .SetGroupName ("lorawan")
    .AddConstructor<LoraChannel> ()
    .AddAttribute ("PropagationLossModel",
                   "A pointer to the propagation loss model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&LoraChannel::m_loss),
                   MakePointerChecker<PropagationLossModel> ())
    .AddAttribute ("PropagationDelayModel",
                   "A pointer to the propagation delay model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&LoraChannel::m_delay),
                   MakePointerChecker<PropagationDelayModel> ())
    .AddTraceSource ("PacketSent",
                     "Trace source fired whenever a packet goes out on the channel",
                     MakeTraceSourceAccessor (&LoraChannel::m_packetSent),
                     "ns3::Packet::TracedCallback");
  return tid;
}

LoraChannel::LoraChannel ()
{
}

LoraChannel::~LoraChannel ()
{
  m_phyList.clear ();
}

LoraChannel::LoraChannel (Ptr<PropagationLossModel> loss,
                          Ptr<PropagationDelayModel> delay) :
  m_loss (loss),
  m_delay (delay)
{
}

void
LoraChannel::Add (Ptr<LoraPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);

  // Add the new phy to the vector
  m_phyList.push_back (phy);
}

void
LoraChannel::Remove (Ptr<LoraPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);

  // Remove the phy from the vector
  m_phyList.erase (find (m_phyList.begin (), m_phyList.end (), phy));
}

std::size_t
LoraChannel::GetNDevices (void) const
{
  return m_phyList.size ();
}

Ptr<NetDevice>
LoraChannel::GetDevice (std::size_t i) const
{
  return m_phyList[i]->GetDevice ()->GetObject<NetDevice> ();
}

void
LoraChannel::Send (Ptr< LoraPhy > sender, Ptr< Packet > packet,
                   double txPowerDbm, LoraTxParameters txParams,
                   Time duration, double frequencyMHz) const
{
  NS_LOG_FUNCTION (this << sender << packet << txPowerDbm << txParams <<
                   duration << frequencyMHz);

  // Get the mobility model of the sender
  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  
  Ptr<LoraChannel> senderChannel = sender->GetChannel();
  // std::cout << " Sender possui endereço: " << sender << " senderChannel: " << senderChannel << std::endl;
  // std::cout << " Sender possui channel de tamanho: " << senderChannel->GetNDevices() << std::endl;
  Ptr<Channel> channel = sender->GetChannel();
  // LoraPhyHelper phyHelper = LoraPhyHelper ();
  // phyHelper.SetChannel (channel);
  // phyHelper.SetChannel (senderChannel);
  // Ptr<Node> node = endDevices.Get(i);
  // Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
  // Ptr<LoraPhy> phy = loraNetDevice->GetPhy();


  NS_ASSERT (senderMobility != 0);     // Make sure it's available

  NS_LOG_INFO ("Starting cycle over all " << m_phyList.size () << " PHYs" << sender->GetChannel() << " channel");
  // std::cout <<" Starting cycle over all " << m_phyList.size () << " PHYs " << sender->GetChannel() << " channel" << std::endl;
  // for (i = )
  NS_LOG_INFO ("Sender mobility: " << senderMobility->GetPosition ());
  // std::cout << "(lora-channel) Sender mobility: " << senderMobility->GetPosition ();

  // uint32_t n = 0;
  // for (std::vector<Ptr<LoraPhy> > :: const_iterator m = m_phyList.begin (); m != m_phyList.end (); m++, n++)
  //   {
  //     Ptr<Channel> channel = m_phyList[n]->GetChannel();
  //     std::cout << "(lora-channel) channel conteúdo: " << channel << " size: " << m_phyList.size() << " n " << n << std::endl;
  //   }

  // Cycle over all registered PHYs
  uint32_t j = 0;
  std::vector<Ptr<LoraPhy> >::const_iterator i;
  for (i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
    {
      // std::cout <<" Entrou no for!" << std::endl;
      // std::cout << "Valor de i: " << *i  << " sender: " << sender << std::endl;
      // Do not deliver to the sender (*i is the current PHY)
    // Ptr<Channel> channel1 = m_phyList[j]->GetChannel();
    // std::cout << "(lora-channel) channel1: " << channel1 << " size: " << m_phyList.size() << " j " << j << std::endl;


      if (sender != (*i))
        {
          // Get the receiver's mobility model
          Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->
            GetObject<MobilityModel> ();
          // std::cout << " (lora-channel) i: " << *i << " receiverMobility: " << receiverMobility << std::endl;
          NS_LOG_INFO ("Receiver mobility: " <<
                       receiverMobility->GetPosition ());
          // std::cout << "(lora-channel) Valor de receiver mobility: " << receiverMobility->GetPosition() << std::endl;


          // Compute delay using the delay model
          Time delay = m_delay->GetDelay (senderMobility, receiverMobility);

          // Compute received power using the loss model
          double rxPowerDbm = GetRxPower (txPowerDbm, senderMobility,
                                          receiverMobility);

          NS_LOG_DEBUG ("Propagation: txPower=" << txPowerDbm <<
                        "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
                        "distance=" << senderMobility->GetDistanceFrom (receiverMobility) <<
                        "m, delay=" << delay);
          // std::cout << "Propagation: txPower=" << txPowerDbm <<
          //               "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
          //               "distance=" << senderMobility->GetDistanceFrom (receiverMobility) <<
          //               "m, delay=" << delay << std::endl;

          // Get the id of the destination PHY to correctly format the context
          Ptr<NetDevice> dstNetDevice = m_phyList[j]->GetDevice ();
          uint32_t dstNode = 0;
          if (dstNetDevice != 0)
            {
              NS_LOG_INFO ("Getting node index from NetDevice, since it exists");
              dstNode = dstNetDevice->GetNode ()->GetId ();
              NS_LOG_DEBUG ("dstNode = " << dstNode);

            }
          else
            {
              NS_LOG_INFO ("No net device connected to the PHY, using context 0");
            }
          // std::cout << "indice j " << j << std::endl;
          // Create the parameters object based on the calculations above
          LoraChannelParameters parameters;
          parameters.rxPowerDbm = rxPowerDbm;
          parameters.sf = txParams.sf;
          parameters.duration = duration;
          parameters.frequencyMHz = frequencyMHz;

          // Schedule the receive event
          NS_LOG_INFO ("Scheduling reception of the packet");
          // std::cout << "Entrando na recepção do pacote " << packet << "no contexto: " << dstNode << std::endl;

          Simulator::ScheduleWithContext (dstNode, delay, &LoraChannel::Receive,
                                          this, j, packet, parameters);

          // std::cout << "antes de m_packetSent" << std::endl;
          // Fire the trace source for sent packet
          m_packetSent (packet);
        }
    }
}

void
LoraChannel::Receive (uint32_t i, Ptr<Packet> packet,
                      LoraChannelParameters parameters) const
{
  NS_LOG_FUNCTION (this << i << packet << parameters);
  // std::cout << "this " << this << " i " << i << " packet " << packet << " parameters " << parameters << std::endl;

  // Call the appropriate PHY instance to let it begin reception
  m_phyList[i]->StartReceive (packet, parameters.rxPowerDbm, parameters.sf,
                              parameters.duration, parameters.frequencyMHz);
}

double
LoraChannel::GetRxPower (double txPowerDbm, Ptr<MobilityModel> senderMobility,
                         Ptr<MobilityModel> receiverMobility) const
{
  return m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility);
}

std::ostream &operator << (std::ostream &os, const LoraChannelParameters &params)
{
  os << "(rxPowerDbm: " << params.rxPowerDbm << ", SF: " << unsigned(params.sf) <<
    ", durationSec: " << params.duration.GetSeconds () <<
    ", frequencyMHz: " << params.frequencyMHz << ")";
  return os;
}
}
}
