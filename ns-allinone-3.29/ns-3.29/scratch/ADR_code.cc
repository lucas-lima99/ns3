/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lora-mac.h"
#include "ns3/gateway-lora-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include <algorithm>
#include <ctime>
#include <time.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <stack>
#include "ns3/random-variable-stream.h"



using namespace ns3;
using namespace lorawan;


NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Test if the file is empty
bool is_empty(std::ifstream& pFile)
{
	return pFile.peek() == std::ifstream::traits_type::eof();
}

// To be used in tic toc time counter
clock_t startTimer;
time_t beginTimer;
//
// Implementation of tic, i.e., start time counter
void
tic()
{
	beginTimer = time(&beginTimer);
	struct tm * timeinfo;
	timeinfo = localtime(&beginTimer);
	std::cout << "simulation start at: " << asctime(timeinfo) << std::endl;
}
// implementation of toc, i.e., stop time counter
double
toc()
{
	time_t finishTimer = time(&finishTimer);
	double simTime = difftime(finishTimer, beginTimer) / 60.0;
	struct tm * timeinfo;
	timeinfo = localtime(&finishTimer);
	std::cout << "simulation finished at: " << asctime(timeinfo) << std::endl;
	//
	std::cout << "Time elapsed: " << simTime << " minutes" << std::endl;
	//
	return simTime;
}

int main (int argc, char *argv[])
{
	tic();
	// Network settings
	int nDevices = 3000;
	int nGateways = 1;
	double radius = 7500;
	double simulationTime = 600;
	int seed=1;
	uint32_t runSeed=1;
	int algoritmo=7;

	// Channel model
	bool realisticChannelModel = false;
	int appPeriodSeconds = 600;
	//int periodsToSimulate = 1;
	int transientPeriods = 0;
	std::string outputDir = "./";
	std::string filename = "DadosLora";

	// Output control
	bool print = false;
	std::string chFilename;

	bool fixedSeed = false;


	CommandLine cmd;
	cmd.AddValue ("nDevices",
			"Number of end devices to include in the simulation",
			nDevices);
	cmd.AddValue ("radius",
			"The radius of the area to simulate",
			radius);
	cmd.AddValue ("simulationTime",
			"The time for which to simulate",
			simulationTime);
	cmd.AddValue ("appPeriodSeconds",
			"The period in seconds to be used by periodically transmitting applications",
			appPeriodSeconds);
	cmd.AddValue ("print",
			"Whether or not to print various informations",
			print);
	cmd.AddValue ("runSeed",
			"Set runseed",
			runSeed);
	cmd.AddValue ("outputDir",
				"Output directory",
				outputDir);
	cmd.AddValue ("fixedSeed",
					"Enable fixed seed and runSeed",
					fixedSeed);
	cmd.AddValue ("filename",
					"Output file name",
					filename);
	cmd.AddValue ("algoritmo",
			"Algoritmo de alocaçao de SF a ser utilizado",
			algoritmo);//1-60%80%85%90%95%   2-20%  3-Fixo 7  4-Sensibilidade 5-dividido sensibilidade 6-random
	cmd.Parse (argc, argv);

	// Set up logging
	LogComponentEnable ("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
	// LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraMac", LOG_LEVEL_ALL);
	//LogComponentEnable("EndDeviceLoraMac", LOG_LEVEL_WARN);
	// LogComponentEnable("GatewayLoraMac", LOG_LEVEL_ALL);
	// LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraMacHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraMacHeader", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

	if (fixedSeed){
		RngSeedManager::SetSeed(seed);
		RngSeedManager::SetRun(runSeed);
	}

	//Getting seed and runSeed for checking and displaying purposes
	seed = RngSeedManager::GetSeed();
	runSeed = RngSeedManager::GetRun();

	chFilename = outputDir+"/"+ filename+".txt";
	std::ifstream file(chFilename.c_str());
	bool firstTime = is_empty(file);
	file.close();
	std::ofstream myfile;
	myfile.open(chFilename.c_str(), std::ofstream::app);
	if ( firstTime )
	{
		// file is empty: write down the header
		myfile <<
				"RunSeed," <<
				"Seed," <<
				"Algoritmo," <<
				"Radius," <<
				"nDevices," <<
				"appPeriodSeconds," <<
				"simulationTime," <<
				"PHYTotal," <<
				"PHYSuccessful," <<
				"PHYInterfered," <<
				"PHYNoMoreReceivers," <<
				"PHYUnderSensitivity," <<
				"PHYLostBecauseTX," <<
				"SimulationDuration";
	}

	myfile << std::endl<<
			runSeed << "," <<
			seed << "," <<
			algoritmo << "," <<
			radius<< "," <<
			nDevices << "," <<
			appPeriodSeconds << "," <<
			simulationTime << ",";
	myfile.close ();

	// Display current simulation information
	std::cout <<
			"Running with nDevices = " << nDevices <<
			", radius = " << radius << " m" <<
			", simulation Time =" << simulationTime << " s" <<
			", appPeriodSeconds = " << appPeriodSeconds << " s" <<
			", Algoritmo ADR = " << algoritmo <<
			", seed = " << seed <<
			", run stream = " << runSeed <<
			", outputDir = " << outputDir << std::endl;

	/***********
	 *  Setup  *
	 ***********/

	// Create the time value from the period
	Time appPeriod = Seconds (appPeriodSeconds);

	// Mobility
	MobilityHelper mobility;
	mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
			"rho", DoubleValue (radius),
			"X", DoubleValue (0.0),
			"Y", DoubleValue (0.0));
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

	/************************
	 *  Create the channel  *
	 ************************/

	// Create the lora channel object
	Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
	loss->SetPathLossExponent (3.76);
	loss->SetReference (1, 7.7);

	if (realisticChannelModel)
	{
		// Create the correlated shadowing component
		Ptr<CorrelatedShadowingPropagationLossModel> shadowing = CreateObject<CorrelatedShadowingPropagationLossModel> ();

		// Aggregate shadowing to the logdistance loss
		loss->SetNext (shadowing);

		// Add the effect to the channel propagation loss
		Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

		shadowing->SetNext (buildingLoss);
	}

	Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

	Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

	/************************
	 *  Create the helpers  *
	 ************************/

	// Create the LoraPhyHelper
	LoraPhyHelper phyHelper = LoraPhyHelper ();
	phyHelper.SetChannel (channel);

	// Create the LoraMacHelper
	LoraMacHelper macHelper = LoraMacHelper ();

	// Create the LoraHelper
	LoraHelper helper = LoraHelper ();
	helper.EnablePacketTracking (chFilename); // Output filename
	// helper.EnableSimulationTimePrinting ();

	//Create the NetworkServerHelper
	NetworkServerHelper nsHelper = NetworkServerHelper ();

	//Create the ForwarderHelper
	ForwarderHelper forHelper = ForwarderHelper ();

	/************************
	 *  Create End Devices  *
	 ************************/

	// Create a set of nodes
	NodeContainer endDevices;
	endDevices.Create (nDevices);

	// Assign a mobility model to each node
	mobility.Install (endDevices);

	// Make it so that nodes are at a certain height > 0
	for (NodeContainer::Iterator j = endDevices.Begin ();
			j != endDevices.End (); ++j)
	{
		Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
		Vector position = mobility->GetPosition ();
		position.z = 1.2;
		mobility->SetPosition (position);
	}

	// Create the LoraNetDevices of the end devices
	uint8_t nwkId = 54;
	uint32_t nwkAddr = 1864;
	Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);

	// Create the LoraNetDevices of the end devices
	macHelper.SetAddressGenerator (addrGen);
	phyHelper.SetDeviceType (LoraPhyHelper::ED);
	macHelper.SetDeviceType (LoraMacHelper::ED);
	helper.Install (phyHelper, macHelper, endDevices);

	// Now end devices are connected to the channel

	// Connect trace sources
	for (NodeContainer::Iterator j = endDevices.Begin ();
			j != endDevices.End (); ++j)
	{
		Ptr<Node> node = *j;
		Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
		Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
	}

	/*********************
	 *  Create Gateways  *
	 *********************/

	// Create the gateway nodes (allocate them uniformely on the disc)
	NodeContainer gateways;
	gateways.Create (nGateways);

	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
	// Make it so that nodes are at a certain height > 0
	allocator->Add (Vector (0.0, 0.0, 15.0));
	mobility.SetPositionAllocator (allocator);
	mobility.Install (gateways);


	// Create a netdevice for each gateway
	phyHelper.SetDeviceType (LoraPhyHelper::GW);
	macHelper.SetDeviceType (LoraMacHelper::GW);
	helper.Install (phyHelper, macHelper, gateways);

	/**********************
	 *  Handle buildings  *
	 **********************/

	double xLength = 130;
	double deltaX = 32;
	double yLength = 64;
	double deltaY = 17;
	int gridWidth = 2 * radius / (xLength + deltaX);
	int gridHeight = 2 * radius / (yLength + deltaY);
	if (realisticChannelModel == false)
	{
		gridWidth = 0;
		gridHeight = 0;
	}
	Ptr<GridBuildingAllocator> gridBuildingAllocator;
	gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
	gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
	gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
	gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
	gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
	gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
	gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
	gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
	gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
	gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
	gridBuildingAllocator->SetAttribute ("MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
	gridBuildingAllocator->SetAttribute ("MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
	BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

	BuildingsHelper::Install (endDevices);
	BuildingsHelper::Install (gateways);
	BuildingsHelper::MakeMobilityModelConsistent ();

	// Print the buildings
	if (print)
	{
		std::ofstream myfile;
		myfile.open ("buildings.txt");
		std::vector<Ptr<Building> >::const_iterator it;
		int j = 1;
		for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j)
		{
			Box boundaries = (*it)->GetBoundaries ();
			myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
		}
		myfile.close ();

	}

	/**********************************************
	 *  Set up the end device's spreading factor  *
	 **********************************************/
	int txPowerdBm = 2;
	macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel,algoritmo, txPowerdBm);


	NS_LOG_DEBUG ("Completed configuration");

	/*********************************************
	 *  Install applications on the end devices  *
	 *********************************************/

	Time appStopTime = Seconds (simulationTime);
	PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
	appHelper.SetPeriod (Seconds (appPeriodSeconds));
	appHelper.SetPacketSize (23);
	//Ptr <RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> ("Min", DoubleValue (0), "Max", DoubleValue (10));
	ApplicationContainer appContainer = appHelper.Install (endDevices);

	appContainer.Start (Seconds (0));
	appContainer.Stop (appStopTime);

	/*
  ApplicationContainer appContainer2 = appHelper.Install (endDevices);
  ApplicationContainer appContainer3 = appHelper.Install (endDevices);

  appContainer2.Start (Seconds (0));
  appContainer2.Stop (appStopTime);
  appContainer3.Start (Seconds (0));
  appContainer3.Stop (appStopTime);
	 */
	/**************************
	 *  Create Network Server  *
	 ***************************/

	// Create the NS node
	NodeContainer networkServer;
	networkServer.Create (1);

	// Create a NS for the network
	nsHelper.SetEndDevices (endDevices);
	nsHelper.SetGateways (gateways);
	nsHelper.Install (networkServer);

	//Create a forwarder for each gateway
	forHelper.Install (gateways);

	/**********************
	 * Print output files *
	 *********************/
	if (print)
	{
		helper.PrintEndDevices (endDevices, gateways,
				"endDevices.dat");
	}

	////////////////
	// Simulation //
	////////////////

	Simulator::Stop (appStopTime + Hours (1));

	NS_LOG_INFO ("Running simulation...");
	Simulator::Run ();

	Simulator::Destroy ();

	///////////////////////////
	// Print results to file //
	///////////////////////////
	NS_LOG_INFO ("Computing performance metrics...");
	helper.PrintPerformance (transientPeriods * appPeriod, appStopTime);

	double simDutation = toc();
	// Display total time to perform simulation
	//std::cout << "SimDuration: " << simDutation << std::endl;
	myfile.open (chFilename.c_str(), std::ofstream::app);
	myfile << simDutation;
	myfile.close ();


	return 0;
}
