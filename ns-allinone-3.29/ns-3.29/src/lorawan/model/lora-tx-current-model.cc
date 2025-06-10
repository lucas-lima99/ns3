/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Romagnolo Stefano <romagnolostefano93@gmail.com>
 */

#include "lora-tx-current-model.h"
#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "lora-utils.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include <cmath>

namespace ns3 {
namespace lorawan {




NS_LOG_COMPONENT_DEFINE ("LoraTxCurrentModel");

NS_OBJECT_ENSURE_REGISTERED (LoraTxCurrentModel);

TypeId
LoraTxCurrentModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LoraTxCurrentModel")
    .SetParent<Object> ()
    .SetGroupName ("Lora")
  ;
  return tid;
}

LoraTxCurrentModel::LoraTxCurrentModel ()
{
}

LoraTxCurrentModel::~LoraTxCurrentModel ()
{
}

// Similarly to the wifi case
NS_OBJECT_ENSURE_REGISTERED (LinearLoraTxCurrentModel);

TypeId
LinearLoraTxCurrentModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LinearLoraTxCurrentModel")
    .SetParent<LoraTxCurrentModel> ()
    .SetGroupName ("Lora")
    .AddConstructor<LinearLoraTxCurrentModel> ()
    .AddAttribute ("Eta", "The efficiency of the power amplifier.",
                   DoubleValue (0.10),
                   MakeDoubleAccessor (&LinearLoraTxCurrentModel::SetEta,
                                       &LinearLoraTxCurrentModel::GetEta),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Voltage", "The supply voltage (in Volts).",
                   DoubleValue (3.3),
                   MakeDoubleAccessor (&LinearLoraTxCurrentModel::SetVoltage,
                                       &LinearLoraTxCurrentModel::GetVoltage),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("StandbyCurrent", "The current in the STANDBY state (in Watts).",
                   DoubleValue (0.0014),      // idle mode = 1.4mA
                   MakeDoubleAccessor (&LinearLoraTxCurrentModel::SetStandbyCurrent,
                                       &LinearLoraTxCurrentModel::GetStandbyCurrent),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

LinearLoraTxCurrentModel::LinearLoraTxCurrentModel ()
{
  NS_LOG_FUNCTION (this);
}

LinearLoraTxCurrentModel::~LinearLoraTxCurrentModel ()
{
  NS_LOG_FUNCTION (this);
}

void
LinearLoraTxCurrentModel::SetEta (double eta)
{
  NS_LOG_FUNCTION (this << eta);
  m_eta = eta;
}

void
LinearLoraTxCurrentModel::SetVoltage (double voltage)
{
  NS_LOG_FUNCTION (this << voltage);
  m_voltage = voltage;
}

void
LinearLoraTxCurrentModel::SetStandbyCurrent (double idleCurrent)
{
  NS_LOG_FUNCTION (this << idleCurrent);
  m_idleCurrent = idleCurrent;
}

double
LinearLoraTxCurrentModel::GetEta (void) const
{
  return m_eta;
}

double
LinearLoraTxCurrentModel::GetVoltage (void) const
{
  return m_voltage;
}

double
LinearLoraTxCurrentModel::GetStandbyCurrent (void) const
{
  return m_idleCurrent;
}

double
LinearLoraTxCurrentModel::CalcTxCurrent (double txPowerDbm) const
{
  NS_LOG_FUNCTION (this << txPowerDbm);
  return DbmToW (txPowerDbm) / (m_voltage * m_eta) + m_idleCurrent;
}


NS_OBJECT_ENSURE_REGISTERED (ConstantLoraTxCurrentModel);

TypeId
ConstantLoraTxCurrentModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ConstantLoraTxCurrentModel")
    .SetParent<LoraTxCurrentModel> ()
    .SetGroupName ("Lora")
    .AddConstructor<ConstantLoraTxCurrentModel> ()
    .AddAttribute ("TxCurrent",
                   "The radio Tx current in Ampere.",
                   DoubleValue (0.028),        // transmit at 0dBm = 28mA
                   MakeDoubleAccessor (&ConstantLoraTxCurrentModel::SetTxCurrent,
                                       &ConstantLoraTxCurrentModel::GetTxCurrent),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

ConstantLoraTxCurrentModel::ConstantLoraTxCurrentModel ()
{
  NS_LOG_FUNCTION (this);
}

ConstantLoraTxCurrentModel::~ConstantLoraTxCurrentModel ()
{
  NS_LOG_FUNCTION (this);
}

void
ConstantLoraTxCurrentModel::SetTxCurrent (double txCurrent)
{
  NS_LOG_FUNCTION (this << txCurrent);
  m_txCurrent = txCurrent;
}

double
ConstantLoraTxCurrentModel::GetTxCurrent (void) const
{
  return m_txCurrent;
}

double
ConstantLoraTxCurrentModel::CalcTxCurrent (double txPowerDbm) const
{
  NS_LOG_FUNCTION (this << txPowerDbm);
  return m_txCurrent;
}

NS_OBJECT_ENSURE_REGISTERED (SX1272LoRaWANCurrentModel);

const double SX1272LoRaWANCurrentModel::m_txPowerUsePaBoost[] = {0.032018, 0.033157, 0.034224, 0.035168, 0.036302,
0.037481, 0.038711, 0.040310, 0.042289, 0.044276, 0.046755, 0.050334, 0.054216, 0.061582, 0.068982, 0.077138, 0.0, 0.0, 0.105454}; // 2dBm to 17dBm, 1dB steps. Then high power mode is 20dBm.

const double SX1272LoRaWANCurrentModel::m_txPowerUseRfo[] = {0.0};

const double SX1272LoRaWANCurrentModel::m_rxWithLnaBoost[] = {0.010803, 0.011607}; //125kHz, 250kHz
const double SX1272LoRaWANCurrentModel::m_rxNoLnaBoost[]   = {0.009877, 0.010694}; //125kHz, 250kHz

TypeId
SX1272LoRaWANCurrentModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SX1272LoRaWANCurrentModel")
    .SetParent<LoraTxCurrentModel> ()
    .SetGroupName ("Lora")
    .AddConstructor<SX1272LoRaWANCurrentModel> ()
    .AddAttribute ("Voltage", "The supply voltage (in Volts).",
                   DoubleValue (3.3),
                   MakeDoubleAccessor (&SX1272LoRaWANCurrentModel::SetVoltage,
                                       &SX1272LoRaWANCurrentModel::GetVoltage),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("IdleCurrent", "The current in the standby state.",
                   DoubleValue (0.001664),
                   MakeDoubleAccessor (&SX1272LoRaWANCurrentModel::SetIdleCurrent,
                                       &SX1272LoRaWANCurrentModel::GetIdleCurrent),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SleepCurrent", "The current in the SLEEP state.",
                   DoubleValue (0.000001),
                   MakeDoubleAccessor (&SX1272LoRaWANCurrentModel::SetSleepCurrent,
                                       &SX1272LoRaWANCurrentModel::GetSleepCurrent),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxCurrent", "The current in the TX state.",
                   DoubleValue (0.054216),
                   MakeDoubleAccessor (&SX1272LoRaWANCurrentModel::SetTxCurrentDirectly,
                                       &SX1272LoRaWANCurrentModel::GetTxCurrent),
                   MakeDoubleChecker<double> ())
	.AddAttribute ("TxPowerToTxCurrent", "The power who generate the current in the TX state.",
				   DoubleValue (14),
				   MakeDoubleAccessor (&SX1272LoRaWANCurrentModel::SetTxCurrent,
				    					&SX1272LoRaWANCurrentModel::GetTxCurrent),
				  MakeDoubleChecker<double> ())
    .AddAttribute ("RxCurrent", "The current in the RX state.",
                   DoubleValue (0.010803),
                   MakeDoubleAccessor (&SX1272LoRaWANCurrentModel::SetRxCurrentDirectly,
                                       &SX1272LoRaWANCurrentModel::GetRxCurrent),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("UsePaBoost", "Choice of use of PaBoost pin or RFO pin.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&SX1272LoRaWANCurrentModel::SetPaBoost,
                                        &SX1272LoRaWANCurrentModel::GetPaBoost),
                   MakeBooleanChecker ())
    .AddAttribute ("UseLnaBoost", "Choice of use of LnaBoost mode.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&SX1272LoRaWANCurrentModel::SetLnaBoost,
                                        &SX1272LoRaWANCurrentModel::GetLnaBoost),
                   MakeBooleanChecker ())
  ;
  return tid;
}

SX1272LoRaWANCurrentModel::SX1272LoRaWANCurrentModel ()
{
  NS_LOG_FUNCTION (this);
}

SX1272LoRaWANCurrentModel::~SX1272LoRaWANCurrentModel()
{
}



//TODO: link this in helper

double
SX1272LoRaWANCurrentModel::CalcTxCurrent (double txPowerDbm) const
{
  NS_LOG_FUNCTION (this << txPowerDbm);

  int ind = (int) txPowerDbm - 2;
  // ind = ind - 2;

  // m_txCurrent =  m_txPowerUsePaBoost[ind];

  double newcurrent = m_txPowerUsePaBoost[ind];

  return newcurrent;
}

void
SX1272LoRaWANCurrentModel::SetRxCurrent(double bandwidth)
{
  if(m_useLnaBoost)
  {
    if(bandwidth == 125000)
    {
      m_rxCurrent = m_rxWithLnaBoost[0];
    }
    else if(bandwidth == 250000)
    {
      m_rxCurrent = m_rxWithLnaBoost[1];
    }
    else
    {
      std::cout << bandwidth << std::endl;
      NS_FATAL_ERROR ("SX1272LoRaWANCurrentModel:current values for bandwidth chosen not available");
    }
  }
  else
  {
    if(bandwidth == 125000)
    {
      m_rxCurrent = m_rxNoLnaBoost[0];
    }
    else if(bandwidth == 250000)
    {
      m_rxCurrent = m_rxNoLnaBoost[1];
    }
    else
    {
      NS_FATAL_ERROR ("SX1272LoRaWANCurrentModel:current values for bandwidth chosen not available");
    }
  }
}

void
SX1272LoRaWANCurrentModel::SetRxCurrentDirectly(double rx_current)
{
  m_rxCurrent = rx_current;
}

double
SX1272LoRaWANCurrentModel::GetRxCurrent (void) const
{
  NS_LOG_FUNCTION (this);
  return m_rxCurrent;
}

void
SX1272LoRaWANCurrentModel::SetPaBoost (bool paBoost)
{
  NS_LOG_FUNCTION (this << paBoost);
  m_usePaBoost = paBoost;
}

bool
SX1272LoRaWANCurrentModel::GetPaBoost (void) const
{
  NS_LOG_FUNCTION (this);
  return m_usePaBoost;
}

void
SX1272LoRaWANCurrentModel::SetLnaBoost (bool lnaBoost)
{
  NS_LOG_FUNCTION (this << lnaBoost);
  m_useLnaBoost = lnaBoost;
}

bool
SX1272LoRaWANCurrentModel::GetLnaBoost (void) const
{
  NS_LOG_FUNCTION (this);
  return m_useLnaBoost;
}

void
SX1272LoRaWANCurrentModel::SetTxCurrent (double txPowerDbm)
{
  NS_LOG_FUNCTION (this << txPowerDbm);
  m_usePaBoost = true;
  if(m_usePaBoost)
  {
    if(txPowerDbm < 2)
    {
      NS_LOG_LOGIC (this << " Chosen dBm of " << txPowerDbm << " is less than the SX1272 min of 2dBm, using 2dBm instead.");
      txPowerDbm = 2;

    }
    else if(txPowerDbm == 18 || txPowerDbm == 19)
    {
      NS_FATAL_ERROR ("SX1272LoRaWANCurrentModel:18dBm and 19dBm are not available on the SX1272.");
    }
    else if(txPowerDbm > 20)
    {
      NS_LOG_LOGIC (this << " Chosen dBm of " << txPowerDbm << " is higher than the SX1272 max of 20dBm, using 20dBm instead.");
      txPowerDbm = 20;
    }

    int ind = (int) txPowerDbm;
    ind -= 2;

    m_txCurrent =  m_txPowerUsePaBoost[ind];

  }
  else
  {
    //TODO
    NS_FATAL_ERROR ("SX1272LoRaWANCurrentModel:values for RFO not yet defined");
  }
}

void
SX1272LoRaWANCurrentModel::SetTxCurrentDirectly(double tx_current)
{
  m_txCurrent = tx_current;
}

void
SX1272LoRaWANCurrentModel::SetTxPowerToTxCurrent(double txPowerdBm)
{
  m_txPowerdBm = txPowerdBm;
}

double
SX1272LoRaWANCurrentModel::GetTxPowerToTxCurrent(void) const
{
  NS_LOG_FUNCTION(this);
  return m_txPowerdBm;
}

double
SX1272LoRaWANCurrentModel::GetTxCurrent (void) const
{
  NS_LOG_FUNCTION (this);
  return m_txCurrent;
}

void
SX1272LoRaWANCurrentModel::SetVoltage (double voltage)
{
  NS_LOG_FUNCTION (this << voltage);
  m_voltage = voltage;
}

double
SX1272LoRaWANCurrentModel::GetVoltage (void) const
{
  NS_LOG_FUNCTION (this);
  return m_voltage;
}

void
SX1272LoRaWANCurrentModel::SetIdleCurrent (double idleCurrent)
{
  NS_LOG_FUNCTION (this << idleCurrent);
  m_idleCurrent = idleCurrent;
}

double
SX1272LoRaWANCurrentModel::GetIdleCurrent (void) const
{
  NS_LOG_FUNCTION (this);
  return m_idleCurrent;
}


void
SX1272LoRaWANCurrentModel::SetSleepCurrent (double sleepCurrent)
{
  NS_LOG_FUNCTION (this << sleepCurrent);
  m_sleepCurrent = sleepCurrent;
}


double
SX1272LoRaWANCurrentModel::GetSleepCurrent (void) const
{
  NS_LOG_FUNCTION (this);
  return m_sleepCurrent;
}


}
} // namespace ns3
