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
 * Authors: Romagnolo Stefano <romagnolostefano93@gmail.com>
 *          Davide Magrin <magrinda@dei.unipd.it>
 */

#ifndef LORA_TX_CURRENT_MODEL_H
#define LORA_TX_CURRENT_MODEL_H

#include "ns3/object.h"

namespace ns3 {
namespace lorawan {

/**
 * \ingroup energy
 *
 * \brief Model the transmit current as a function of the transmit power and
 * mode.
 */
class LoraTxCurrentModel : public Object
{
public:
  static TypeId GetTypeId (void);

  LoraTxCurrentModel ();
  virtual ~LoraTxCurrentModel ();

  /**
   * Get the current for transmission at this power.
   *
   * \param txPowerDbm The nominal tx power in dBm
   * \returns The transmit current (in Ampere)
   */
  virtual double CalcTxCurrent (double txPowerDbm) const = 0;
};

/**
 * A linear model of the transmission current for a LoRa device, based on the
 * WiFi model.
 */
class LinearLoraTxCurrentModel : public LoraTxCurrentModel
{
public:
  static TypeId GetTypeId (void);

  LinearLoraTxCurrentModel ();
  virtual ~LinearLoraTxCurrentModel ();

  /**
   * \param eta (dimension-less)
   *
   * Set the power amplifier efficiency.
   */
  void SetEta (double eta);

  /**
   * \param voltage (Volts)
   *
   * Set the supply voltage.
   */
  void SetVoltage (double voltage);

  /**
   * \param idleCurrent (Ampere)
   *
   * Set the current in the STANDBY state.
   */
  void SetStandbyCurrent (double idleCurrent);

  /**
   * \return the power amplifier efficiency.
   */
  double GetEta (void) const;

  /**
   * \return the supply voltage.
   */
  double GetVoltage (void) const;

  /**
   * \return the current in the STANDBY state.
   */
  double GetStandbyCurrent (void) const;

  double CalcTxCurrent (double txPowerDbm) const;

private:
  double m_eta;     //!< ETA
  double m_voltage;     //!< Voltage
  double m_idleCurrent;     //!< Standby current
};

class ConstantLoraTxCurrentModel : public LoraTxCurrentModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  ConstantLoraTxCurrentModel ();
  virtual ~ConstantLoraTxCurrentModel ();

  /**
   * \param txCurrent (Ampere)
   *
   * Set the current in the TX state.
   */
  void SetTxCurrent (double txCurrent);

  /**
   * \return the current in the TX state.
   */
  double GetTxCurrent (void) const;

  double CalcTxCurrent (double txPowerDbm) const;


private:
  double m_txCurrent;
};

class  SX1272LoRaWANCurrentModel : public LoraTxCurrentModel
{
public:
  static TypeId GetTypeId (void);

  SX1272LoRaWANCurrentModel ();
  virtual ~SX1272LoRaWANCurrentModel ();


  /**
   * \param txPowerDbm (dBm)
   *
   * Set the current in the TX state of the model, based on the Tx power.
   */
  void SetTxCurrent (double txPowerDbm);

  /**
   * \param tx_current (Ampere)
   *
   * Set the current in the TX state of the model directly.
   */
  void SetTxCurrentDirectly(double tx_current);

  /**
   * \return the current in the TX state, which is dependent on the current TX Power and choice of PA circuitry.
   */
  double GetTxCurrent (void) const;

  double CalcTxCurrent (double txPowerDbm) const;
  /**
   * \param bandwidth (Hz)
   *
   *  Set the bandwidth in the RX state, which is dependent on the bandwidth of the channel.
   */
  void SetRxCurrent (double bandwidth);

  /**
   * \param rx_current (Ampere)
   *
   * Set the current in the RX state of the model directly.
   */
  void SetRxCurrentDirectly(double rx_current);

  /**
   * \return the current in the RX state, which is dependent on the bandwidth of the channel and use of LnaBoost.
   */
  double GetRxCurrent () const;


  double GetVoltage (void) const;

    /**
     * \param voltage (Volts)
     *
     * Set the supply voltage.
     */
  void SetVoltage (double voltage);

    /**
     * \param idleCurrent (Ampere)
     *
     * Set the current in the IDLE state.
     */
  void SetIdleCurrent (double idleCurrent);

    /**
     * \return the current in the IDLE state.
     */
  double GetIdleCurrent (void) const;


    /**
     * \param sleepCurrent (Ampere)
     *
     * Set the current in the SLEEP state.
     */
  void SetSleepCurrent (double sleepCurrent);

    /**
     * \return the current in the SLEEP state.
     */
  double GetSleepCurrent (void) const;

  void SetTxPowerToTxCurrent (double txPowerdBm);

  double GetTxPowerToTxCurrent (void) const;

  private:

    double m_voltage;
    double m_idleCurrent;
    double m_sleepCurrent;


  /**
   * \param sleepCurrent (Ampere)
   *
   * Set PaBoost .
   */
  void SetPaBoost (bool paBoost);

  /**
   * \return the current in the RX state, which is dependent on the bandwidth of the channel and use of LnaBoost.
   */
  bool GetPaBoost (void) const;


  /**
   * \param sleepCurrent (Ampere)
   *
   * Set the current in the SLEEP state.
   */
  void SetLnaBoost (bool lnaBoost);

  /**
   * \return the current in the RX state, which is dependent on the bandwidth of the channel and use of LnaBoost.
   */
  bool GetLnaBoost (void) const;



private:

  const static double m_txPowerUsePaBoost[];
  const static double m_txPowerUseRfo[];

  const static double m_rxWithLnaBoost[];
  const static double m_rxNoLnaBoost[];

  double m_txCurrent; // current value of TX current consumption

  double m_rxCurrent; // current value of RX current consumption

  bool m_usePaBoost; // choice of whether to use PaBoost mode or not

  bool m_useLnaBoost; // choice of whether to use LnaBoost mode or not
  double m_txPowerdBm; // txPower who converts to txCurrent


};

} // namespace ns3

}
#endif /* LORA_TX_CURRENT_MODEL_H */
