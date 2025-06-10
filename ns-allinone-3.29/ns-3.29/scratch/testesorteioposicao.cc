/*
 * This script simulates a simple network to explain how the Lora energy model
 * works.
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
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/end-device-lora-mac.h"
#include "ns3/file-helper.h"
#include "ns3/names.h"
#include <algorithm>
#include <ctime>
#include <time.h>
#include <math.h>
#include "ns3/lora-tx-current-model.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/propagation-loss-model.h"
#include <fstream>
#include <iostream>
#include <string>
#include <stack>
#include "ns3/double.h"
#include <cstdlib>
#include <map>
#include <iomanip>
#include <random>


using namespace ns3;
using namespace lorawan;


///////////////////MAB////////////////
#include "LoRaBandit_v2.h"
#include "LoRaBandit_v2.cpp"

int NBandits = 9; //number of actions -- bandits to test
LoRaBandit LoRaMAB(NBandits);
double start_Q_time = 2.5; 
int MabInterval = 600;

//lista de todos os pacotes recebidos no intervalo de tempo de atuação do MAB
std::vector<std::tuple<Ptr<Packet const>, uint32_t, double, uint8_t>> Rxlist;
// Guardam quantidade de pacotes recebidos e potencia de cada intervalo de tempo marcado por time.
//lista de todos os pacotes transmitidos pelos EDs no intervalo de tempo de atuação do MAB
std::vector<std::tuple<Ptr<Packet const>>> TxlistED;
std::vector<std::tuple<Ptr<Packet const>>> RxlistED;

//função de callback, sempre que um pacote é transmitido por uma vaca, o pacote é adicionado a lista TxlistED.
void getTxlistED (Ptr<Packet const> packet, uint32_t systemId){
	TxlistED.push_back(std::make_tuple(packet));
	LoRaMAB.SetPHYtot(1);
  // std::cout << "Pacote transmitido registrado no tempo: " << Simulator::Now().GetSeconds() << "s" << std::endl;

}

void getRxlistED(Ptr<const Packet> packet, uint32_t systemId){
	RxlistED.push_back(std::make_tuple(packet));
	LoRaMAB.SetPHYsucc(1);

}
void processLine(const std::string& line, std::vector<std::string>& values) {
    size_t start = line.find('[');
    size_t end = line.find(']');
    if (start != std::string::npos && end != std::string::npos) {
        std::string data = line.substr(start + 1, end - start - 1);
        std::istringstream iss(data);
        std::string value;
        while (std::getline(iss, value, ' ')) {
            if (!value.empty()) {
                values.push_back(value);
            }
        }
    }
}


void callPrintPerformance(LoraHelper *helper, Time start, Time stop) {
    helper->PrintPerformance(start, stop);
}

///////////////////////////////

NS_LOG_COMPONENT_DEFINE ("LoraEnergyModelExample");


// Test if the file is empty
bool is_empty(std::ifstream& pFile)
{
	return pFile.peek() == std::ifstream::traits_type::eof();
}

// To be used in tic toc time counter
clock_t startTimer;
time_t beginTimer;
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

void SaveMABmetrics(const std::string& chFilenameMAB, std::string moment) {
    const double* Q = LoRaMAB.GetQ();
    const double* N = LoRaMAB.GetN();
    const int size = LoRaMAB.GetSize();

    std::ofstream MABfile(chFilenameMAB.c_str(), std::ofstream::app);

    MABfile << moment << ": " << std::endl;
    MABfile << "Q: [ ";
    for (int i = 0; i < size; ++i) {
        MABfile << std::fixed << std::setprecision(6) << Q[i] << " ";
    }
    MABfile << "]" << std::endl;

    MABfile << "N: [ ";
    for (int i = 0; i < size; ++i) {
        MABfile << N[i] << " ";
    }
    MABfile << "]" << std::endl;

    MABfile.close();
}


void LoRaBanditRun(NodeContainer endDevices, NodeContainer gateways, Ptr<LoraChannel> channel, int txPowerdBm, LoraMacHelper macHelper, bool visada){
	//std::cout << "LoRaBandit Running..." << std::endl;
    if (visada == 1) {
        std::cout << "------ LOS ------" << std::endl;
    } else {
        std::cout << "------ NLOS -----" << std::endl;
        }
    
	LoRaMAB.Run();
	int Action = LoRaMAB.GetAction();
  std::cout << "Algoritmo: " << Action << " Tx: " << txPowerdBm << std::endl;
  // std::cout << "Channel: " << channel <<  std::endl;


	macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel, (Action), txPowerdBm);

}


void ChangeStateToNLOS(NodeContainer devices, Ptr<LoraChannel> channel, NodeContainer gateways, LoraPhyHelper phyHelper,
 LoraMacHelper macHelper, LoraHelper helper)
{

    phyHelper.SetChannel (channel);

    // macHelper.SetSpreadingFactorsUp (devices, gateways, channel, algoritmo, txPowerdBm);

    std::cout << "trocou para NLOS" << std::endl;

    for (u_int32_t i = 0; i < devices.GetN(); ++i)
    {
        Ptr<Node> node = devices.Get(i);
        // NetDevice
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();

        // phy->SetChannel(channelNLOS); // Canal A candidato
        phy->SetChannel(channel); // Canal A candidato
        // channel->Add(phy);
        // Ptr<LoraChannel> testechannel = phy->GetChannel();
        // std::cout << "VALOR DE phy: " << phy << std::endl;
        // std::cout << "VALOR DE channel: " << channel << std::endl;
        // std::cout << "VALOR DE channel dentro do phy: " << testechannel<< std::endl;


    }

    


    helper.Install (phyHelper, macHelper, gateways);
    NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, devices);

    std::cout << "VALOR DE channelGetNDevices: " << channel->GetNDevices() << std::endl;
          uint32_t dstNode = 0;
    int j = 0;
    for(int i = 0; i < channel->GetNDevices(); i++, j++){
        Ptr<NetDevice> channellist = channel->GetDevice(i);
        std::cout << "VALOR DE channellist: " << channellist << std::endl;
        Ptr<NetDevice> dstNetDevice = channel->GetDevice (j);
        dstNode = dstNetDevice->GetNode ()->GetId ();
        std::cout << "dstNode: " << dstNode << std::endl;


    }
    Ptr<NetDevice> dstNetDevice = channel->GetDevice (0);
    dstNode = dstNetDevice->GetNode ()->GetId ();
    std::cout << "dstNode: " << dstNode << std::endl;
   
    // LoRaMAB.resetBandits();
}

void ChangeStatus(NodeContainer devices, Ptr<LoraChannel> channelLOS, Ptr<LoraChannel> channelNLOS, NodeContainer gateways,
 double proporcaoNLOS){

 int nDevices = devices.GetN();
 int endNLOS = proporcaoNLOS * nDevices;
 int endLOS = nDevices - endNLOS;

for (u_int32_t i = 0; i < devices.GetN(); ++i)
    {
        Ptr<Node> node = devices.Get(i);
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();

        if (i < endNLOS)
        {
            // channelNLOS->Add(phy);
            phy->SetChannel(channelNLOS); // Canal A candidato
            // phyHelper.SetChannel(channelNLOS);
            // std::cout << "NLOS: " << i << "canal: " << phy->GetChannel() << std::endl;
            channelLOS->Add(phy);
            Ptr<MobilityModel> mobilidade = phy->GetMobility();
          // std::cout << " mobilty do dispositivo " << i << " : " << mobilidade << std::endl;

        // validacaoNLOS++;           

        }
        else
        {
            // channelLOS->Add(phy);
            phy->SetChannel(channelLOS); // Canal B
            // std::cout << "LOS: " << i << "canal: " << phy->GetChannel() << std::endl;
            // Ptr<MobilityModel> mobilidade = phy->GetMobility();
            // std::cout << " mobilty do dispositivo " << i << " : " << mobilidade << std::endl;

            // phyHelper.SetChannel(channelLOS);
            channelLOS->Add(phy);
            // validacaoLOS++;

        }

  // Ptr<Channel> testechannel = loraNetDevice->GetChannel();
  // std::cout << "channel LOS: " << channelLOS << std::endl;
  // std::cout << "channel NLOS: " << channelNLOS << std::endl;

  // std::cout << "testechannel: " << testechannel << std::endl;
    }
  std::cout << "dispositivos mudaram de canal! " << std::endl;
}

void ChangePosition(NodeContainer devices, NodeContainer gateways, int radius, MobilityHelper mobilityhelper){
  

   for (NodeContainer::Iterator it = devices.Begin(); it != devices.End(); ++it) {
	        Ptr<Node> node = *it;
	        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
          // mobility->SetPosition(Vector(0,0,0));
          mobilityhelper.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
          "rho", DoubleValue (radius),
          "X", DoubleValue (0.0),
          "Y", DoubleValue (0.0));
          mobilityhelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
          mobilityhelper.Install(node);
          // mobility->SetPosition((Vector(12.0,10.0,0.0)));

	        // Vector pos = mobility->GetPosition();
	        // std::cout << node->GetId()<< "," << pos.x << "," << pos.y << "," << pos.z  << std::endl;
}

  std::cout << " Dispositivos mudaram de posição! " << std::endl;

}

int main (int argc, char *argv[])
{
	tic();
  // Set up logging
//   LogComponentEnable ("LoraEnergyModelExample", LOG_LEVEL_ALL);
//  LogComponentEnable ("LoraRadioEnergyModel", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraChannel", LOG_LEVEL_ALL); //candidato
  // LogComponentEnable ("LoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_ALL);
//   LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("EndDeviceLoraMac", LOG_LEVEL_ALL);
//   LogComponentEnable ("GatewayLoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraPhyHelper", LOG_LEVEL_ALL);
//   LogComponentEnable ("LoraMacHelper", LOG_LEVEL_ALL);
//   LogComponentEnable ("OneShotSenderHelper", LOG_LEVEL_ALL);
//   LogComponentEnable ("OneShotSender", LOG_LEVEL_ALL);
//   LogComponentEnable ("LoraMacHeader", LOG_LEVEL_ALL);
//   LogComponentEnable ("LoraFrameHeader", LOG_LEVEL_ALL);
  // LogComponentEnableAll (LOG_PREFIX_FUNC);
  // LogComponentEnableAll (LOG_PREFIX_NODE);
  // LogComponentEnableAll (LOG_PREFIX_TIME);
  // LogComponentEnableAll (LOG_PREFIX_FUNC);
  // LogComponentEnableAll (LOG_PREFIX_NODE);
  // LogComponentEnableAll (LOG_PREFIX_TIME);

  int radius = 2000;
  int nDevices = 2;
  int algoritmo = 6;
  double batteryEnergyFinal;
  double batteryEnergyInit = 10000;
  double batteryVoltage = 3.3;
  double appPeriodsSeconds = 1800;
  bool realisticChannelModel = false;
  int seed=1;
  uint32_t runSeed=1;
  // bool print = false;
  int packetsize = 23;
  double N = 3.78;
  double sigma = 0;
  int txPowerdBm = 12;
  int hours = 1;
  double distanceReference = 41;
  double lossReference = 79;
  double simulationTime;

  double N_LOS = 2.64;
  double lossReferenceLOS = 79;
  double distanceReferenceLOS = 41;

  double N_NLOS = 3.03;
  double lossReferenceNLOS = 81;
  double distanceReferenceNLOS = 27;

	//Getting seed and runSeed for checking and displaying purposes
	seed = RngSeedManager::GetSeed();
	runSeed = RngSeedManager::GetRun();

  std::string outputDir = "./";
  std::string filename = "DadosBattery";
  std::string chFilename;
  std::string chfilenamepositions;
  std::string fileMAB = "DadosMAB100LOS";
  std::string chFilenameMAB;      //File tp store MAB data
  chFilenameMAB = outputDir+"/"+ fileMAB +".txt";
  // std::string bestMABsimulation = "BestMAB";
  // std::string chFilenameBestMAB;      //File tp store the best MAB data
  // chFilenameBestMAB = outputDir+"/"+ bestMABsimulation +".txt";


	CommandLine cmd;
	cmd.AddValue ("nDevices",
			"Number of end devices to include in the simulation",
			nDevices);
	cmd.AddValue ("radius",
			"The radius of the area to simulate",
			radius);
	cmd.AddValue ("appPeriodsSeconds",
			"The period in seconds to be used by periodically transmitting applications",
			appPeriodsSeconds);
	cmd.AddValue ("outputDir",
				"Output directory",
				outputDir);
	cmd.AddValue ("filename",
					"Output file name",
					filename);
	cmd.AddValue ("algoritmo",
			"Algoritmo de alocaçao de SF a ser utilizado",
			algoritmo);
	cmd.AddValue ("packetsize",
				"Tamanho do pacote em bytes",
				packetsize);
	cmd.AddValue ("runSeed",
				"Set runseed",
				runSeed);
	cmd.AddValue ("expoenteN",
				  "expoente N da regressão para modelo los ou nlos",
				  N);
	cmd.AddValue ("distanceReference",
				 " distancia de referencia para path loss log distance",
				  distanceReference);
	cmd.AddValue ("lossReference",
				  "P0 para path loss log distance",
				  lossReference);
	cmd.AddValue ("sigma",
				  "Desvio padrão do PL em dB",
				  sigma);
	cmd.AddValue ("txPowerdBm",
				  "Potência de transmissão em dBm",
				  txPowerdBm);
	cmd.AddValue ("hours",
				  "Tempo de simulação em horas",
				  hours);
	cmd.Parse (argc, argv);



// Displaying the seed and runSeed being used in the simulation
	std::cout << "Seed: " << seed << ", RunSeed: " << runSeed << std::endl;

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
				"packetSize," <<
				"N," <<
				"d," <<
				"P_0," <<
				"sigma," <<
  			"Algoritmo," <<
  			"Radius," <<
  			"nDevices," <<
  			"appPeriodsSeconds," <<
				"batteryVoltage," <<
				"batteryEnergyInit," <<
				"hours," <<
				"TxPowerdBm," <<
				"PHYTotal," <<
				"PHYSuccessful," <<
				"PHYInterfered," <<
				"PHYNoMoreReceivers," <<
				"PHYUnderSensitivity," <<
				"PHYLostBecauseTX," <<
				"batteryEnergyFinal";
  	}

  	myfile << std::endl <<
  			runSeed << "," <<
			seed << "," <<
  			packetsize << "," <<
			N << "," <<
			distanceReference << "," <<
			lossReference << "," <<
			sigma << "," <<
  			algoritmo << "," <<
  			radius<< "," <<
  			nDevices << "," <<
  			appPeriodsSeconds << "," <<
  			batteryVoltage << "," <<
			batteryEnergyInit << "," <<
			hours << "," <<
  			txPowerdBm << ",";

  	myfile.close ();

  /************************
  *  Create the channel  *
  ************************/

  Ptr<NormalRandomVariable> gaussianVar = CreateObject<NormalRandomVariable> ();
  gaussianVar->SetAttribute ("Mean", DoubleValue (0.0));
  gaussianVar->SetAttribute ("Variance", DoubleValue (sigma * sigma));
  double gaussianValue = gaussianVar->GetValue();

  // Create the lora channel object

  Ptr<LogDistanceGaussianDistributionPropagationLossModel> LOS = CreateObject<LogDistanceGaussianDistributionPropagationLossModel> ();
  LOS->SetPathLossExponent (N_LOS);
  LOS->SetReference (distanceReferenceLOS, lossReferenceLOS);
  LOS->SetGaussianVariable(gaussianValue);

  Ptr<LogDistanceGaussianDistributionPropagationLossModel> NLOS = CreateObject<LogDistanceGaussianDistributionPropagationLossModel> ();
  NLOS->SetPathLossExponent (N_NLOS);
  NLOS->SetReference (distanceReferenceNLOS, lossReferenceNLOS);
  NLOS->SetGaussianVariable(gaussianValue);

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  // Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
  // std::cout << "Primeira chamada: algoritmo: " << algoritmo << "channel: " <<channel << std::endl;


  Ptr<LoraChannel> channelLOS = CreateObject<LoraChannel> (LOS, delay);
  // std::cout << "channel LOS : " <<channelLOS << std::endl;


  Ptr<LoraChannel> channelNLOS = CreateObject<LoraChannel> (NLOS, delay);
  // std::cout << "channel nLOS : " <<channelNLOS << std::endl;

// std::cout << "LOS: " << channelLOS << std::endl;
// std::cout << "NLOS: " << channelNLOS << std::endl;


  /************************
  *  Create the helpers  *
  ************************/


  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
  			"rho", DoubleValue (radius),
  			"X", DoubleValue (0.0),
  			"Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  // phyHelper.SetChannel (channel);
  // phyHelper.SetChannel (channelLOS);
  phyHelper.SetChannel (channelNLOS);

  // Create the LoraMacHelper
  LoraMacHelper macHelper = LoraMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (chFilename); // Output filename

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();


  /*******************
  *  Create End Devices  *
  *******************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  //  double proporcaoNLOS = 0.8;
  // int endNLOS = proporcaoNLOS * nDevices;
  // int endLOS = nDevices - endNLOS;
  // int validacaoNLOS = 0, validacaoLOS = 0;

 
  // Assign a mobility model to the node
  mobility.Install (endDevices);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LoraMacHelper::ED);

  NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);
 
    // }
  /******************
   *Create Gateways*
   *****************/

  NodeContainer gateways;
  gateways.Create (1);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  allocator->Add (Vector (0.0, 0.0, 15.0));

  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LoraMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

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

	/********************
	 *  Handle buildings  *
	 ********************/

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

	// // Print the buildings
	// if (print)
	// {
	// 	std::ofstream myfile;
	// 	myfile.open ("buildings.txt");
	// 	std::vector<Ptr<Building> >::const_iterator it;
	// 	int j = 1;
	// 	for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j)
	// 	{
	// 		Box boundaries = (*it)->GetBoundaries ();
	// 		myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
	// 	}
	// 	myfile.close ();
	// }

  double proporcaoNLOS = 0.77; // Todo LOS
  int endNLOS = proporcaoNLOS * nDevices;

for (u_int32_t i = 0; i < endDevices.GetN(); ++i)
    {
        Ptr<Node> node = endDevices.Get(i);
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();

        if (i < endNLOS)
        {
            phy->SetChannel(channelNLOS); // Canal A candidato
            channelLOS->Add(phy);
            // Ptr<MobilityModel> mobilidade = phy->GetMobility();
          // std::cout << " mobilty do dispositivo " << i << " : " << mobilidade << std::endl;
        }
        else
        {
            phy->SetChannel(channelLOS); // Canal B
            // Ptr<MobilityModel> mobilidade = phy->GetMobility();
            // std::cout << " mobilty do dispositivo " << i << " : " << mobilidade << std::endl;
            channelLOS->Add(phy);
        }
    }

  Ptr<Node> nodeGateway = gateways.Get(0);
  Ptr<LoraNetDevice> loraNetDeviceGateway = nodeGateway->GetDevice(0)->GetObject<LoraNetDevice>();
  Ptr<LoraPhy> phyGateway = loraNetDeviceGateway->GetPhy();
  Ptr<MobilityModel> mobilidadegateway = phyGateway->GetMobility();
  // std::cout << " mobilty do gateway " << mobilidadegateway << std::endl;

  channelLOS->Add(phyGateway);
  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channelNLOS, algoritmo, txPowerdBm);
  for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
      Ptr<Node> node = endDevices.Get(i);

      for (uint32_t j = 0; j < node->GetNDevices(); ++j) {
          Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(j));

          if (loraNetDevice) {
              Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLoraMac>();
              if (mac) {
                  mac->SetTransmissionPower(txPowerdBm);

              } else {
            	  std::cerr << "Erro: MAC nao encontrado para o dispositivo " << i << "\n";
                  }
          }
      }
  }


  /*****************************************
   *  Install applications on the end devices  *
   *****************************************/

  PeriodicSenderHelper periodicSenderHelper;
  periodicSenderHelper.SetPeriod (Seconds (appPeriodsSeconds));
  periodicSenderHelper.SetPacketSize (packetsize);

  periodicSenderHelper.Install (endDevices);

  /********************
   * Install Energy Model *
   *******************/

  BasicEnergySourceHelper basicSourceHelper;
  LoraRadioEnergyModelHelper radioEnergyHelper;

  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (batteryEnergyInit)); // Energy in J
  basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (batteryVoltage)); //Voltage in V

  radioEnergyHelper.SetTxCurrentModel ("ns3::SX1272LoRaWANCurrentModel",
		  	  	  	  	  	  	  	   "TxPowerToTxCurrent", DoubleValue(txPowerdBm),
								       "UsePaBoost", BooleanValue(true));
  // install source on EDs' nodes
  EnergySourceContainer sources = basicSourceHelper.Install (endDevices);

  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install
      (endDevicesNetDevices, sources);
  std::string local;
    // int j = 0;

  simulationTime = hours*60*60;
  // double metade = simulationTime/2;

//sempre que um pacote é recebido, ele é adicionado à lista Rx
	for (NodeContainer::Iterator j = gateways.Begin ();
				j != gateways.End (); ++j)
		{
			Ptr<Node> node = *j;

			
			Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
			Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
			phy->TraceConnectWithoutContext ("ReceivedPacket",
												MakeCallback
													(&getRxlistED));	
		}

	//sempre que um pacote é transmitido, ele é adicionado à lista Tx
	for (NodeContainer::Iterator j = endDevices.Begin ();
			j != endDevices.End (); ++j)
	{
		Ptr<Node> node = *j;
		Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
		Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
		phy->TraceConnectWithoutContext ("StartSending",
                                               MakeCallback
                                                 (&getTxlistED));
	}
    std::cout<< "Início da simulação " << std::endl;

// for (NodeContainer::Iterator it = endDevices.Begin(); it != endDevices.End(); ++it) {
// 	        Ptr<Node> node = *it;
// 	        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
// 	        Vector pos = mobility->GetPosition();
//           std::cout << node->GetId()<< "," << pos.x << "," << pos.y << "," << pos.z << "," << algoritmo << std::endl;
// }


  /**************
  * Simulation *
  **************/

  Simulator::Stop (Seconds (simulationTime) + Seconds (5));
  bool visada;
// LOS/NLOS no tempo
// for(int i=600; i < simulationTime/2; i += MabInterval){
//         // Simulator::Schedule (Seconds(i), &ReallocateChannels, endDevices, channelNLOS, channelLOS);
//    		Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelLOS, txPowerdBm, macHelper, visada=1);
//    		// Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelLOS, txPowerdBm, macHelper, visada=1);
//         // Simulator::Schedule (Seconds(i), &SaveMABmetrics, chFilenameMAB, local);
//         // j++;
// 	}

//   // Simulator::Schedule(Seconds(1 + simulationTime / 2.0), &ChangeStateToNLOS, endDevices, channelNLOS, gateways, phyHelper, macHelper, helper);
  Simulator::Schedule(Seconds(simulationTime), &callPrintPerformance, &helper, Seconds(0), Seconds(simulationTime));
  // Simulator::Schedule(Seconds(simulationTime/4), &ChangePosition, endDevices, gateways, radius, mobility);
  // Simulator::Schedule(Seconds(simulationTime/8), &ChangePosition, endDevices, gateways, radius, mobility);
  // Simulator::Schedule(Seconds(simulationTime/2), &ChangePosition, endDevices, gateways, radius, mobility);

  // Simulator::Schedule(Seconds(simulationTime/4), &ChangeStatus, endDevices,channelLOS, channelNLOS, gateways, proporcaoNLOS);
  // Simulator::Schedule(Seconds(simulationTime/8), &ChangeStatus, endDevices,channelLOS, channelNLOS, gateways, proporcaoNLOS);
  // Simulator::Schedule(Seconds(simulationTime/2), &ChangeStatus, endDevices,channelLOS, channelNLOS, gateways, proporcaoNLOS);


  int aux = 0;

  for(int i=600, m=1; i <= simulationTime; i += MabInterval, m+=1){

      
   		Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelNLOS, txPowerdBm, macHelper, visada=0);

        // Simulator::Schedule (Seconds(i), &ReallocateChannels, endDevices, channelNLOS, channelLOS);
        // Simulator::Schedule(Seconds(i), &ChangePosition, endDevices, gateways, radius, mobility);
  // Simulator::Schedule(Seconds(i), &ChangePosition, endDevices, gateways, radius, mobility);
  // Simulator::Schedule(Seconds(i), &ChangePosition, endDevices, gateways, radius, mobility);
        if (m == 2){
        
          Simulator::Schedule(Seconds(i), &ChangePosition, endDevices, gateways, radius, mobility);
          Simulator::Schedule(Seconds(i), &ChangeStatus, endDevices,channelLOS, channelNLOS, gateways, proporcaoNLOS);
          m=0;
        }
        // Simulator::Schedule(Seconds(i), &ChangeStatus, endDevices,channelLOS, channelNLOS, gateways, proporcaoNLOS);
  // Simulator::Schedule(Seconds(i), &ChangeStatus, endDevices,channelLOS, channelNLOS, gateways, proporcaoNLOS);
  // Simulator::Schedule(Seconds(i), &ChangeStatus, endDevices,channelLOS, channelNLOS, gateways, proporcaoNLOS);
   		// Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelNLOS, txPowerdBm, macHelper, visada=0);
   		// Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelLOS, txPowerdBm, macHelper, visada=1);
        Simulator::Schedule (Seconds(i), &SaveMABmetrics, chFilenameMAB, local);
        // j++;
        aux = i;
        // std::cout << "Trocou de posição em " << m << " segundos " << "mab acionado em " << i << " segundos"<<std::endl;
	}
  

  // for(int i=600; i <= simulationTime; i += MabInterval){
  //       // Simulator::Schedule (Seconds(i), &ReallocateChannels, endDevices, channelNLOS, channelLOS);
  //  		Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelNLOS, txPowerdBm, macHelper, visada=0);
  //  		// Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelLOS, txPowerdBm, macHelper, visada=1);
  //       // Simulator::Schedule (Seconds(i), &SaveMABmetrics, chFilenameMAB, local);
  //       // j++;
  //       aux = i;
	// }
  // Simulator::Schedule (Seconds(metade), &callprintPerformance, helper, inicio, (metade), chFilename);
  std::cout<< "valor de aux: " << aux << std::endl;
  // Simulator::Schedule (Seconds(aux), &SaveMABmetrics, chFilenameMAB, local);

  // Simulator::Schedule (Seconds(metade), &ChangeState, endDevices, channelNLOS, gateways, phyHelper, macHelper, helper);
  // Simulator::Schedule (Seconds(metade), &ChangeStatus, algoritmo, txPowerdBm, endDevices, channelNLOS, gateways, macHelper);
    // Simulator::Schedule(Seconds(simulationTime), &callPrintPerformance, &helper, Seconds(simulationTime/2.0), Seconds(simulationTime));
  // Simulator::Schedule (Seconds(simulationTime), &callprintPerformance, helper, metade, simulationTime, chFilename);
  


//TODO ver isso

// for(int i=simulationTime/2; i < simulationTime; i += MabInterval){
//         // Simulator::Schedule (Seconds(i), &ReallocateChannels, endDevices, channelNLOS, channelLOS);
//    		Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelNLOS, txPowerdBm, macHelper, visada=0);
//    		// Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelLOS, txPowerdBm, macHelper, visada=1);
//         Simulator::Schedule (Seconds(i), &SaveMABmetrics, chFilenameMAB, local);
//         j++;
// 	}

  // Simulator::Schedule (Seconds(simulationTime/2), &callprintPerformance, helper, (simulationTime/2), chFilename);

  Simulator::Run ();
  double energy = 0;
  for(int i=0; i<nDevices; i++){
	  energy += sources.Get(i)->GetRemainingEnergy();
  }
  batteryEnergyFinal = energy/nDevices;
  Simulator::Destroy ();
  // helper.PrintPerformance (Seconds(0), Hours(hours));

  toc();
  myfile.open (chFilename.c_str(), std::ofstream::app);
  myfile << batteryEnergyFinal;
  myfile.close ();


// std::ifstream inFile("DadosMAB100LOS.txt");
//     if (!inFile) {
//         std::cerr << "Erro ao abrir o arquivo DadosMab.txt!" << std::endl;
//         return 1;
//     }

//     std::string line;
//     std::vector<std::string> lines;

//     // Ler todas as linhas do arquivo
//     while (std::getline(inFile, line)) {
//         if (!line.empty()) {
//             lines.push_back(line);
//         }
//     }

//     // Verificar se o arquivo tem pelo menos 2 linhas
//     if (lines.size() < 2) {
//         std::cerr << "Erro: O arquivo não possui pelo menos duas linhas!" << std::endl;
//         return 1;
//     }

//     // Penúltima e última linha
//     std::string qLine = lines[lines.size() - 2];
//     std::string nLine = lines[lines.size() - 1];

//     // Vetores para armazenar valores de Q e N
//     std::vector<std::string> Q_values;
//     std::vector<std::string> N_values;

//     // Processar as linhas para extrair os valores
//     processLine(qLine, Q_values);
//     processLine(nLine, N_values);

//     // Verificar se os tamanhos dos vetores de Q e N são iguais
//     if (Q_values.size() != N_values.size()) {
//         std::cerr << "Erro: O número de elementos em Q e N não são iguais!" << std::endl;
//         return 1;
//     }

//     // Abrir o arquivo de saída no modo append (adicionar conteúdo ao final)
//     std::ofstream outFile("final.txt", std::ios::app);
//     if (!outFile) {
//         std::cerr << "Erro ao abrir o arquivo final.txt!" << std::endl;
//         return 1;
//     }

//     // Escrever cabeçalhos apenas se o arquivo estiver vazio
//     std::ifstream checkFile("finalteste.txt");
//     if (checkFile.peek() == std::ifstream::traits_type::eof()) {
//         outFile << "visada,seed,radius";
//         for (size_t i = 0; i < Q_values.size(); ++i) {
//             outFile << ",Q" << (i + 1);
//         }
//         for (size_t i = 0; i < N_values.size(); ++i) {
//             outFile << ",N" << (i + 1);
//         }
//         outFile << "\n";
//     }

//     // Escrever valores de seed, radius, Q e N
//     outFile << "final," << runSeed << "," << radius;
//     for (const auto& q : Q_values) {
//         outFile << "," << q;
//     }
//     for (const auto& n : N_values) {
//         outFile << "," << n;
//     }
//     outFile << "\n";

//     // Fechar os arquivos
//     inFile.close();
//     outFile.close();

//     std::cout << "Dados com seed e radius adicionados em 'final.txt' com sucesso!" << std::endl;


  return 0;
}
