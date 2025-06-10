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

#include "ns3/lora-mac-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/lora-net-device.h"
#include "ns3/log.h"

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("LoraMacHelper");

LoraMacHelper::LoraMacHelper ()
  : m_region (LoraMacHelper::EU)
{
}

void
LoraMacHelper::Set (std::string name, const AttributeValue &v)
{
  m_mac.Set (name, v);
}

void
LoraMacHelper::SetDeviceType (enum DeviceType dt)
{
  NS_LOG_FUNCTION (this << dt);
  switch (dt)
    {
    case GW:
      m_mac.SetTypeId ("ns3::GatewayLoraMac");
      break;
    case ED:
      m_mac.SetTypeId ("ns3::EndDeviceLoraMac");
      break;
    }
  m_deviceType = dt;
}

void
LoraMacHelper::SetAddressGenerator (Ptr<LoraDeviceAddressGenerator> addrGen)
{
  NS_LOG_FUNCTION (this);

  m_addrGen = addrGen;
}

void
LoraMacHelper::SetRegion (enum LoraMacHelper::Regions region)
{
  m_region = region;
}

Ptr<LoraMac>
LoraMacHelper::Create (Ptr<Node> node, Ptr<NetDevice> device) const
{
  Ptr<LoraMac> mac = m_mac.Create<LoraMac> ();
  mac->SetDevice (device);

  // If we are operating on an end device, add an address to it
  if (m_deviceType == ED && m_addrGen != 0)
    {
      mac->GetObject<EndDeviceLoraMac> ()->SetDeviceAddress
        (m_addrGen->NextAddress ());
    }

  // Add a basic list of channels based on the region where the device is
  // operating
  if (m_deviceType == ED)
    {
      Ptr<EndDeviceLoraMac> edMac = mac->GetObject<EndDeviceLoraMac> ();
      switch (m_region)
        {
        case LoraMacHelper::EU:
          {
            ConfigureForEuRegion (edMac);
            break;
          }
        default:
          {
            NS_LOG_ERROR ("This region isn't supported yet!");
            break;
          }
        }
    }
  else
    {
      Ptr<GatewayLoraMac> gwMac = mac->GetObject<GatewayLoraMac> ();
      switch (m_region)
        {
        case LoraMacHelper::EU:
          {
            ConfigureForEuRegion (gwMac);
            break;
          }
        default:
          {
            NS_LOG_ERROR ("This region isn't supported yet!");
            break;
          }
        }
    }
  return mac;
}

void
LoraMacHelper::ConfigureForEuRegion (Ptr<EndDeviceLoraMac> edMac) const
{
  NS_LOG_FUNCTION_NOARGS ();

  ApplyCommonEuConfigurations (edMac);

  /////////////////////////////////////////////////////
  // TxPower -> Transmission power in dBm conversion //
  /////////////////////////////////////////////////////
  edMac->SetTxDbmForTxPower (std::vector<double> {16, 14, 12, 10, 8, 6, 4, 2});

  ////////////////////////////////////////////////////////////
  // Matrix to know which DataRate the GW will respond with //
  ////////////////////////////////////////////////////////////
  LoraMac::ReplyDataRateMatrix matrix = {{{{0,0,0,0,0,0}},
                                          {{1,0,0,0,0,0}},
                                          {{2,1,0,0,0,0}},
                                          {{3,2,1,0,0,0}},
                                          {{4,3,2,1,0,0}},
                                          {{5,4,3,2,1,0}},
                                          {{6,5,4,3,2,1}},
                                          {{7,6,5,4,3,2}}}};
  edMac->SetReplyDataRateMatrix (matrix);

  /////////////////////
  // Preamble length //
  /////////////////////
  edMac->SetNPreambleSymbols (8);

  //////////////////////////////////////
  // Second receive window parameters //
  //////////////////////////////////////
  edMac->SetSecondReceiveWindowDataRate (0);
  edMac->SetSecondReceiveWindowFrequency (869.525);
}

void
LoraMacHelper::ConfigureForEuRegion (Ptr<GatewayLoraMac> gwMac) const
{
  NS_LOG_FUNCTION_NOARGS ();

  ///////////////////////////////
  // ReceivePath configuration //
  ///////////////////////////////
  Ptr<GatewayLoraPhy> gwPhy = gwMac->GetDevice ()->
    GetObject<LoraNetDevice> ()->GetPhy ()->GetObject<GatewayLoraPhy> ();

  ApplyCommonEuConfigurations (gwMac);

  if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
      NS_LOG_DEBUG ("Resetting reception paths");
      gwPhy->ResetReceptionPaths ();

      std::vector<double> frequencies;
      frequencies.push_back (868.1);
      frequencies.push_back (868.3);
      frequencies.push_back (868.5);

      std::vector<double>::iterator it = frequencies.begin ();

      int receptionPaths = 0;
      int maxReceptionPaths = 8;
      while (receptionPaths < maxReceptionPaths)
        {
          if (it == frequencies.end ())
            {
              it = frequencies.begin ();
            }
          gwPhy->GetObject<GatewayLoraPhy> ()->AddReceptionPath (*it);
          ++it;
          receptionPaths++;
        }
    }
}

void
LoraMacHelper::ApplyCommonEuConfigurations (Ptr<LoraMac> loraMac) const
{
  NS_LOG_FUNCTION_NOARGS ();

  //////////////
  // SubBands //
  //////////////

  LogicalLoraChannelHelper channelHelper;
  channelHelper.AddSubBand (868, 868.6, 0.01, 14);
  channelHelper.AddSubBand (868.7, 869.2, 0.001, 14);
  channelHelper.AddSubBand (869.4, 869.65, 0.1, 27);

  //////////////////////
  // Default channels //
  //////////////////////
  Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel> (868.1, 0, 5);
  Ptr<LogicalLoraChannel> lc2 = CreateObject<LogicalLoraChannel> (868.3, 0, 5);
  Ptr<LogicalLoraChannel> lc3 = CreateObject<LogicalLoraChannel> (868.5, 0, 5);
  channelHelper.AddChannel (lc1);
  channelHelper.AddChannel (lc2);
  channelHelper.AddChannel (lc3);

  loraMac->SetLogicalLoraChannelHelper (channelHelper);

  ///////////////////////////////////////////////
  // DataRate -> SF, DataRate -> Bandwidth     //
  // and DataRate -> MaxAppPayload conversions //
  ///////////////////////////////////////////////
  loraMac->SetSfForDataRate (std::vector<uint8_t> {12,11,10,9,8,7,7});
  loraMac->SetBandwidthForDataRate (std::vector<double>
                                    {125000,125000,125000,125000,125000,125000,250000});
  loraMac->SetMaxAppPayloadForDataRate (std::vector<uint32_t>
                                        {59,59,59,123,230,230,230,230});

}
void LoraMacHelper::SetSpreadingFactorsUp (NodeContainer endDevices, NodeContainer gateways, Ptr<LoraChannel> channel){
  int algoritmo=1;
  int txPowerdBm=14;
  SetSpreadingFactorsUp (endDevices,gateways,channel,algoritmo, txPowerdBm);
}

void LoraMacHelper::SetSpreadingFactorsUp(NodeContainer endDevices, NodeContainer gateways, Ptr<LoraChannel> channel,int algoritmo,
 int txPowerdBm){
  NS_LOG_FUNCTION_NOARGS ();

  
  std::vector<double> rxPowerAll(endDevices.GetN(),0);
  std::vector<int> order(endDevices.GetN(),0);
  int cont = 0,aux;
  double auxD;
  // LoraPhyHelper phyHelper = LoraPhyHelper ();
  // phyHelper.SetChannel (channel);
  // phyHelper.SetDeviceType (LoraPhyHelper::ED);
  // phyHelper.SetDeviceType (LoraPhyHelper::GW);



  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLoraMac> ();
      NS_ASSERT (mac != 0);
      // Ptr<LoraPhy> phy = loraNetDevice->GetPhy();

      // channel = phy->GetChannel();
      // Try computing the distance from each gateway and find the best one
      Ptr<Node> bestGateway = gateways.Get (0);
      Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel> ();

      // Assume devices transmit at txPowerdBm dBm
      double highestRxPower = channel->GetRxPower (txPowerdBm, position, bestGatewayPosition);

      for (NodeContainer::Iterator currentGw = gateways.Begin () + 1;
           currentGw != gateways.End (); ++currentGw)
        {
          // Compute the power received from the current gateway
          Ptr<Node> curr = *currentGw;
          Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel> ();
          double currentRxPower = channel->GetRxPower (txPowerdBm, position, currPosition);    // dBm

          if (currentRxPower > highestRxPower)
            {
              bestGateway = curr;
              bestGatewayPosition = curr->GetObject<MobilityModel> ();
              highestRxPower = currentRxPower;
            }
        }

      // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
      double rxPower = highestRxPower;
      rxPowerAll[cont]=rxPower;
      order[cont]=cont;
      cont++;
      Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
      // std::cout <<x->GetInteger(7,12)<<std::endl;
      mac->SetSf(x->GetInteger(7,12));
      // std::cout << "mac: " << mac << std::endl;

      // Get the ED sensitivity
      Ptr<EndDeviceLoraPhy> edPhy = loraNetDevice->GetPhy ()->GetObject<EndDeviceLoraPhy> ();
      const double *edSensitivity = edPhy->sensitivity;


      if (rxPower > *edSensitivity)
        {
          mac->SetDataRate (5);
        }
      else if (rxPower > *(edSensitivity + 1))
        {
          mac->SetDataRate (4);
        }
      else if (rxPower > *(edSensitivity + 2))
        {
          mac->SetDataRate (3);
        }
      else if (rxPower > *(edSensitivity + 3))
        {
          mac->SetDataRate (2);
        }
      else if (rxPower > *(edSensitivity + 4))
        {
          mac->SetDataRate (1);
        }
      else if (rxPower > *(edSensitivity + 5))
        {
          mac->SetDataRate (0);

        }
      else // Device is out of range. Assign SF12.
        {
          // NS_LOG_DEBUG ("Device out of range");
          mac->SetDataRate (0);
          // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);

        }
    }
    for(uint32_t  i=0 ; i < endDevices.GetN() ; i++){
      for(uint32_t   k=0 ; k < endDevices.GetN() ; k++){
        if(rxPowerAll[i] >rxPowerAll[k]){
          auxD=rxPowerAll[i];
          rxPowerAll[i]=rxPowerAll[k];
          rxPowerAll[k]=auxD;
          
          aux=order[i];
          order[i]=order[k];
          order[k]=aux;

        }

      }
    }


    for(uint32_t i=0 ; i < endDevices.GetN();i++){

      Ptr<Node> object = endDevices.Get(order[i]);
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLoraMac> ();
      NS_ASSERT (mac != 0);
      //std::cout << "valor de endDevices.GetN é: " << endDevices.GetN();
      // Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
      // std::cout << "phy: " << phy << std::endl;

      // channel = phy->GetChannel();
      // std::cout << "-----"  << std::endl;

      // std::cout << "channel: " << channel << std::endl;
      // std::cout<< "Tamanho do canal: " << channel->GetNDevices() << std::endl;

      Ptr<Node> bestGateway = gateways.Get (0);
      Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel> ();
      double highestRxPower = channel->GetRxPower (txPowerdBm, position, bestGatewayPosition);
      // std::cout << "highestRxPower : " << highestRxPower << std::endl;
      // Get the ED sensitivity
      Ptr<EndDeviceLoraPhy> edPhy = loraNetDevice->GetPhy ()->GetObject<EndDeviceLoraPhy> ();
      const double *edSensitivity = edPhy->sensitivity;

      // // (I)Fixed at SF 7
      // if(algoritmo==1){
      //   mac->SetSf(7);

      // }
      // (I) Arbitrarily divided - Capacity enhancement sensitivity based (a= {0.6,0.15,0.05,0.1,0.05,0.05})
      if(algoritmo==100){
        if(i<(6*endDevices.GetN())/10){
          mac->SetSf(7);
        }
        else if(i<(7.5*endDevices.GetN())/10){
          mac->SetSf(8);
        }
        else if(i<(8.0*endDevices.GetN())/10){
          mac->SetSf(9);
        }
        else if(i<(9.0*endDevices.GetN())/10){
          mac->SetSf(10);
        }
        else if(i<(9.5*endDevices.GetN())/10){
          mac->SetSf(11);
        }
        else{
          mac->SetSf(12);
      }

     }

      // (II) Fixed at SF 12
      // else if(algoritmo==2){
    	//   mac->SetSf(12);
      // }

     // (I) Arbitrarily divided - Capacity enhancement (a= {0.65,0.1,0.05,0.1,0.05,0.05})
      else if(algoritmo==1){
              if(i<(6.5*endDevices.GetN())/10){
                mac->SetSf(7);
              }
              else if(i<(7.5*endDevices.GetN())/10){
                mac->SetSf(8);
              }
              else if(i<(8*endDevices.GetN())/10){
                mac->SetSf(9);
              }
              else if(i<(9*endDevices.GetN())/10){
                mac->SetSf(10);
              }
              else if(i<(9.5*endDevices.GetN())/10){
                mac->SetSf(11);
              }
              else{
                mac->SetSf(12);
              }
            }

      // (III) Arbitrarily divided - Capacity enhancement (a= {0.7,0.1,0.1,0.05,0.04,0.01})
      else if(algoritmo==2){
              if(i<(7*endDevices.GetN())/10){
                mac->SetSf(7);
              }
              else if(i<(8*endDevices.GetN())/10){
                mac->SetSf(8);
              }
              else if(i<(9*endDevices.GetN())/10){
                mac->SetSf(9);
              }
              else if(i<(9.5*endDevices.GetN())/10){
                mac->SetSf(10);
              }
              else if(i<(9.9*endDevices.GetN())/10){
                mac->SetSf(11);
              }
              else{
                mac->SetSf(12);
              }
            }
      // (IV) Arbitrarily divided - Capacity enhancement (a= {0.6,0.2,0.05,0.05,0.05,0.05})
      else if(algoritmo==3){
        if(i<(6*endDevices.GetN())/10){
          mac->SetSf(7);
        }
        else if(i<(8*endDevices.GetN())/10){
          mac->SetSf(8);
        }
        else if(i<(8.5*endDevices.GetN())/10){
          mac->SetSf(9);
        }
        else if(i<(9*endDevices.GetN())/10){
          mac->SetSf(10);
        }
        else if(i<(9.5*endDevices.GetN())/10){
          mac->SetSf(11);
        }
        else{
          mac->SetSf(12);
        }
      }

      // (V) Sensitivity based
      else if(algoritmo==4){
        mac->SetSf(mac->GetSfFromDataRate(mac->GetDataRate()));
      }

      //(VII) Sensitivity based arbitrarily divided - Coverage enhancement (a={0.05,0.05,0.15,0.05,0.2,0.5})
      else if(algoritmo==600){
        if(i<(0.5*endDevices.GetN())/10 && rxPowerAll[order[i]] > *edSensitivity ){
          mac->SetSf(7);
        }
        else if(i<(1.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 1)){
          mac->SetSf(8);
        }
        else if(i<(2.5*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 2)){
          mac->SetSf(9);
        }
        else if(i<(3.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 3)){
          mac->SetSf(10);
        }
        else if(i<(5.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 4)){
          mac->SetSf(11);
        }
        else{
          mac->SetSf(12);
        }
      }

      //(VII) Equally divided
      else if(algoritmo==5){
        if(i<(1*endDevices.GetN())/6){
          mac->SetSf(7);
        }
        else if(i<(2*endDevices.GetN())/6){
          mac->SetSf(8);
        }
        else if(i<(3*endDevices.GetN())/6){
          mac->SetSf(9);
        }
        else if(i<(4*endDevices.GetN())/6){
          mac->SetSf(10);
        }
        else if(i<(5*endDevices.GetN())/6){
          mac->SetSf(11);
        }
        else{
          mac->SetSf(12);
        }
      }

      // (VIII) Arbitrarily divided - Capacity enhancement (a= {0.05,0.1,0.2,0.2,0.25,0.2})
            else if(algoritmo==6){
              if(i<(0.5*endDevices.GetN())/10){
                mac->SetSf(7);
              }
              else if(i<(1.5*endDevices.GetN())/10){
                mac->SetSf(8);
              }
              else if(i<(3.5*endDevices.GetN())/10){
                mac->SetSf(9);
              }
              else if(i<(5.5*endDevices.GetN())/10){
                mac->SetSf(10);
              }
              else if(i<(8*endDevices.GetN())/10){
                mac->SetSf(11);
              }
              else{
                mac->SetSf(12);
              }
            }
      // (IX) Arbitrarily divided - Capacity enhancement (a= {0.05,0.1,0.1,0.1,0.25,0.40})
      else if(algoritmo==7){
              if(i<(0.5*endDevices.GetN())/10){
                mac->SetSf(7);
              }
              else if(i<(1.5*endDevices.GetN())/10){
                mac->SetSf(8);
              }
              else if(i<(2.5*endDevices.GetN())/10){
                mac->SetSf(9);
              }
              else if(i<(3.5*endDevices.GetN())/10){
                mac->SetSf(10);
              }
              else if(i<(6.0*endDevices.GetN())/10){
                mac->SetSf(11);
              }
              else{
                mac->SetSf(12);
              }
            }

      // (X) Random assignment
      else if(algoritmo==10){
        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
        mac->SetSf(x->GetInteger(7,12));
      }

//      else if(algoritmo==10){
//    	// Perform sensitivity algorithm
//        mac->SetSf(mac->GetSfFromDataRate(mac->GetDataRate()));
//        // Perform realocation
//        if(i<(targetRealocation*endDevices.GetN()/100) && mac->GetSf() > 7 ){
//          mac->SetSf(mac->GetSf()-1);
//        }
//      }
      // // (X) Coverage based on energy consumption and sensitivity  (a={0.05,0.05,0.1,0.2,0.5,0.1})
      //     else if(algoritmo==10){
      //         if(i<(0.5*endDevices.GetN())/10 && rxPowerAll[order[i]] > *edSensitivity ){
      //           mac->SetSf(7);
      //         }
      //         else if(i<(1.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 1)){
      //           mac->SetSf(8);
      //         }
      //         else if(i<(2.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 2)){
      //           mac->SetSf(9);
      //         }
      //         else if(i<(4.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 3)){
      //           mac->SetSf(10);
      //         }
      //         else if(i<(9.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 4)){
      //           mac->SetSf(11);
      //         }
      //         else{
      //           mac->SetSf(12);
      //         }
      //       }

      // (XI) Coverage based on energy consumption
           else if(algoritmo==11){
         	  if(i<(0.5*endDevices.GetN())/10){
         		  mac->SetSf(7);
                   }
               else if(i<(1.0*endDevices.GetN())/10){
             	  mac->SetSf(8);
                   }
               else if(i<(2.0*endDevices.GetN())/10){
             	  mac->SetSf(9);
                   }
               else if(i<(4.0*endDevices.GetN())/10){
             	  mac->SetSf(10);
                   }
               else if(i<(9.0*endDevices.GetN())/10){
             	  mac->SetSf(11);
                   }
               else{
             	  mac->SetSf(12);
                   }
           }

      // (XII) Sensitivity based equal split  (a={0.16,0.16,0.16,0.16,0.16,0.16})
          else if(algoritmo==12){
              if(i<(1.0*endDevices.GetN())/6 && rxPowerAll[order[i]] > *edSensitivity ){
                mac->SetSf(7);
              }
              else if(i<(2.0*endDevices.GetN())/6  && rxPowerAll[order[i]] > *(edSensitivity + 1)){
                mac->SetSf(8);
              }
              else if(i<(3.0*endDevices.GetN())/6  && rxPowerAll[order[i]] > *(edSensitivity + 2)){
                mac->SetSf(9);
              }
              else if(i<(4.0*endDevices.GetN())/6  && rxPowerAll[order[i]] > *(edSensitivity + 3)){
                mac->SetSf(10);
              }
              else if(i<(5.0*endDevices.GetN())/6  && rxPowerAll[order[i]] > *(edSensitivity + 4)){
                mac->SetSf(11);
              }
              else{
                mac->SetSf(12);
              }
            }

      // (XIII) Sensitivity based equal split  (a={0.20,0.20,0.20,0.20,0.10,0.10})
         else if(algoritmo==13){
        	 if(i<(2.0*endDevices.GetN())/10 && rxPowerAll[order[i]] > *edSensitivity ){
        		 mac->SetSf(7);
                    }
        	 else if(i<(4.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 1)){
        		 mac->SetSf(8);
                    }
             else if(i<(6.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 2)){
                 mac->SetSf(9);
                    }
             else if(i<(8.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 3)){
            	 mac->SetSf(10);
                    }
             else if(i<(59.0*endDevices.GetN())/10  && rxPowerAll[order[i]] > *(edSensitivity + 4)){
            	 mac->SetSf(11);
                    }
             else{
            	 mac->SetSf(12);
                     }
                  }

            // (XIV) Arbitrarily divided - Capacity enhancement (a= {0.65,0.1,0.05,0.1,0.05,0.05})
            else if(algoritmo==14){
              if(i<(6.5*endDevices.GetN())/10){
                mac->SetSf(7);
              }
              else if(i<(7.5*endDevices.GetN())/10){
                mac->SetSf(8);
              }
              else if(i<(8*endDevices.GetN())/10){
                mac->SetSf(9);
              }
              else if(i<(9*endDevices.GetN())/10){
                mac->SetSf(10);
              }
              else if(i<(9.5*endDevices.GetN())/10){
                mac->SetSf(11);
              }
              else{
                mac->SetSf(12);
              }
            }
            // (XV) Arbitrarily divided - Capacity enhancement (a= {0.7,0.1,0.1,0.05,0.04,0.01})
            else if(algoritmo==15){
              if(i<(7*endDevices.GetN())/10){
                mac->SetSf(7);
              }
              else if(i<(8*endDevices.GetN())/10){
                mac->SetSf(8);
              }
              else if(i<(9*endDevices.GetN())/10){
                mac->SetSf(9);
              }
              else if(i<(9.5*endDevices.GetN())/10){
                mac->SetSf(10);
              }
              else if(i<(9.9*endDevices.GetN())/10){
                mac->SetSf(11);
              }
              else{
                mac->SetSf(12);
              }
            }
            

            // (II) Fixed at SF 7
            else if(algoritmo==27){
          	  mac->SetSf(7);
            }
            else if(algoritmo==28){
            	mac->SetSf(8);
                      }
            else if(algoritmo==29){
            	mac->SetSf(9);
                      }
            else if(algoritmo==30){
            	mac->SetSf(10);
                      }
            else if(algoritmo==31){
            	mac->SetSf(11);
                      }
            else if(algoritmo==32){
            	mac->SetSf(12);
                      }
      else{
      }


/*

      // Get the Gw sensitivity
      Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
      Ptr<LoraNetDevice> gatewayLoraNetDevice = gatewayNetDevice->GetObject<LoraNetDevice> ();
      Ptr<GatewayLoraPhy> gatewayPhy = gatewayLoraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();
      const double *gwSensitivity = gatewayPhy->sensitivity;

      if(rxPower > *gwSensitivity)
        {
          mac->SetDataRate (5);
          sfQuantity[0] = sfQuantity[0] + 1;

        }
      else if (rxPower > *(gwSensitivity+1))
        {
          mac->SetDataRate (4);
          sfQuantity[1] = sfQuantity[1] + 1;

        }
      else if (rxPower > *(gwSensitivity+2))
        {
          mac->SetDataRate (3);
          sfQuantity[2] = sfQuantity[2] + 1;

        }
      else if (rxPower > *(gwSensitivity+3))
        {
          mac->SetDataRate (2);
          sfQuantity[3] = sfQuantity[3] + 1;
        }
      else if (rxPower > *(gwSensitivity+4))
        {
          mac->SetDataRate (1);
          sfQuantity[4] = sfQuantity[4] + 1;
        }
      else if (rxPower > *(gwSensitivity+5))
        {
          mac->SetDataRate (0);
          sfQuantity[5] = sfQuantity[5] + 1;

        }
      else // Device is out of range. Assign SF12.
        {
          mac->SetDataRate (0);
          sfQuantity[6] = sfQuantity[6] + 1;

        }
        */
// end loop on nodes

  

    }
  } //  end function

}
} //end class

