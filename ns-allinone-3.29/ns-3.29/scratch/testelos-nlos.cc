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

# define M_PIl          3.141592653589793238462643383279502884L /* pi */

using namespace ns3;
using namespace lorawan;

/////////////////////////////////////MAB///////////////////////////////////////
#include "LoRaBandit_v2.h"
#include "LoRaBandit_v2.cpp"

int NBandits = 9; //number of actions -- bandits to test
LoRaBandit LoRaMAB(NBandits);
double start_Q_time = 2.5; // Start algorithm 
int MabInterval = 30;
int ChangePositionsInterval = 1800; //30 minutes for 1,5 Km
bool MABActive;

//lista de todos os pacotes recebidos no intervalo de tempo de atuação do MAB
std::vector<std::tuple<Ptr<Packet const>, uint32_t, double, uint8_t>> Rxlist;
// Guardam quantidade de pacotes recebidos e potencia de cada intervalo de tempo marcado por time.
std::vector<std::tuple<int, Time>> Metricscows;
//lista de todos os pacotes transmitidos pelos EDs no intervalo de tempo de atuação do MAB
std::vector<std::tuple<Ptr<Packet const>>> TxlistED;
std::vector<std::tuple<Ptr<Packet const>>> RxlistED;

//função de callback, sempre que um pacote é transmitido por uma vaca, o pacote é adicionado a lista TxlistED.
void getTxlistED (Ptr<Packet const> packet, uint32_t systemId){
	TxlistED.push_back(std::make_tuple(packet));
	LoRaMAB.SetPHYtot(1);
}

void getRxlistED(Ptr<const Packet> packet, uint32_t systemId){
	RxlistED.push_back(std::make_tuple(packet));
	LoRaMAB.SetPHYsucc(1);
}


///////////////////////////////////////////////////////////////////////////

NS_LOG_COMPONENT_DEFINE ("LoraEnergyModelExample");



// Test if the file is empty
bool is_empty(std::ifstream& pFile)
{
	return pFile.peek() == std::ifstream::traits_type::eof();
}


void PrintPositions(NodeContainer nodes, std::string filename, int algoritmo)

{
	std::ifstream inFile(filename.c_str());
	bool fileIsEmpty = is_empty(inFile);
	inFile.close();

	std::ofstream outFile;

	if (fileIsEmpty){
		outFile.open(filename.c_str(), std::ios::out); //abre em modo de escrita
		outFile << "Nó,PosX,PosY,PosZ,Algoritmo" << std::endl;

	}
	outFile.open(filename.c_str(), std::ios::app); // abre em modo de anexação
	for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
	        Ptr<Node> node = *it;
	        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	        Vector pos = mobility->GetPosition();
	//        outFile << "Node " << node->GetId() << ": x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
	        outFile << node->GetId()<< "," << pos.x << "," << pos.y << "," << pos.z << "," << algoritmo << std::endl;
	}
//	outFile << "--" << std::endl;
	outFile.close();
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
        std::cout << "-------- LOS --------" << std::endl;
    } else {
        std::cout << "------ NLOS ---------" << std::endl;
        }
    
	LoRaMAB.Run();
	int Action = LoRaMAB.GetAction();
	macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel, (Action), txPowerdBm);
}

void ReallocateChannels(NodeContainer devices, Ptr<LoraChannel> channelA, Ptr<LoraChannel> channelB)
{
    // Gerar proporção aleatória para o canal A
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0.6, 1.0);
    double proportionA = dist(gen);

    // Determinar o número de dispositivos em cada canal
    u_int32_t numDevicesA = static_cast<int>(proportionA * devices.GetN());
    u_int32_t numDevicesB = devices.GetN() - numDevicesA;

    // Criar uma lista de IDs dos dispositivos
    std::vector<int> deviceIds(devices.GetN());
    std::iota(deviceIds.begin(), deviceIds.end(), 0);

    // Embaralhar aleatoriamente os IDs
    std::shuffle(deviceIds.begin(), deviceIds.end(), gen);

    // Atribuir dispositivos aos canais
    for (u_int32_t i = 0; i < devices.GetN(); ++i)
    {
        Ptr<Node> node = devices.Get(deviceIds[i]);
        std::cout << "VALOR DE NODE: " << node << std::endl;

        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
        std::cout << "VALOR DE phy: " << phy << std::endl;


        if (i < numDevicesA)
        {
            phy->SetChannel(channelA); // Canal A
        std::cout << "NLOS: " << std::endl;
            
        std::cout << "VALOR DE canal: " << phy->GetChannel() << std::endl;

        }
        else
        {
            phy->SetChannel(channelB); // Canal B
        }
    }

    // Imprimir proporções
    std::cout << "Proporção NLOS: " << proportionA << " (" << numDevicesA << " dispositivos em NLOS)" << std::endl;
    std::cout << "Proporção LOS: " << (1 - proportionA) << " (" << numDevicesB << " dispositivos em LOS)" << std::endl;
}


int main (int argc, char *argv[])
{
	tic();
  // Set up logging
//   LogComponentEnable ("LoraEnergyModelExample", LOG_LEVEL_ALL);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);

  int radius = 2000;
  int nDevices = 100;
  int algoritmo = 6;
  double batteryEnergyFinal;
  double batteryEnergyInit = 10000;
  double batteryVoltage = 3.3;
  double appPeriodsSeconds = 600;
  bool realisticChannelModel = false;
  int seed=1;
  uint32_t runSeed=1;
  bool fixedSeed = false;
  bool print = false;
  int packetsize = 23;
  double N = 3.78;
  double sigma = 0;
  int txPowerdBm = 12;
  int hours = 2;
  double distanceReference = 41;
  double lossReference = 79;
  double simulationTime;

  double N_LOS = 2.64;
  double lossReferenceLOS = 79;
  double distanceReferenceLOS = 41;

  double N_NLOS = 3.12;
  double lossReferenceNLOS = 80;
  double distanceReferenceNLOS = 23;


	if (fixedSeed){
		RngSeedManager::SetSeed(seed);
		RngSeedManager::SetRun(runSeed);
	}

	//Getting seed and runSeed for checking and displaying purposes
	seed = RngSeedManager::GetSeed();
	runSeed = RngSeedManager::GetRun();

  std::string outputDir = "./";
  std::string filename = "DadosBattery";
  std::string chFilename;
  std::string chfilenamepositions;
  std::string outputDirpositions = "./";
  std::string fileMAB = "DadosMAB";
  std::string chFilenamegeral;    //File to store the node metric      
  std::string chFilenameMAB;      //File tp store MAB data
  chFilenamegeral = outputDir+"/"+ filename +".txt";
  chFilenameMAB = outputDir+"/"+ fileMAB +".txt";


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

  NS_LOG_INFO ("Creating the channel...");


  Ptr<NormalRandomVariable> gaussianVar = CreateObject<NormalRandomVariable> ();
  gaussianVar->SetAttribute ("Mean", DoubleValue (0.0));
  gaussianVar->SetAttribute ("Variance", DoubleValue (sigma * sigma));
  double gaussianValue = gaussianVar->GetValue();

  // Create the lora channel object

  Ptr<LogDistanceGaussianDistributionPropagationLossModel> loss = CreateObject<LogDistanceGaussianDistributionPropagationLossModel> ();
  loss->SetPathLossExponent (N);
  loss->SetReference (distanceReference, lossReference);
  loss->SetGaussianVariable(gaussianValue);
  NS_LOG_INFO("Gaussian na classe  " << loss->GetGaussianVariable() << " gaussiano");

//TODO Ajeitar LOS/NLOS

  Ptr<LogDistanceGaussianDistributionPropagationLossModel> LOS = CreateObject<LogDistanceGaussianDistributionPropagationLossModel> ();
  LOS->SetPathLossExponent (N_LOS);
  LOS->SetReference (distanceReferenceLOS, lossReferenceLOS);
  LOS->SetGaussianVariable(gaussianValue);
  NS_LOG_INFO("Gaussian na classe  " << LOS->GetGaussianVariable() << " gaussiano");

  Ptr<LogDistanceGaussianDistributionPropagationLossModel> NLOS = CreateObject<LogDistanceGaussianDistributionPropagationLossModel> ();
  NLOS->SetPathLossExponent (N_NLOS);
  NLOS->SetReference (distanceReferenceNLOS, lossReferenceNLOS);
  NLOS->SetGaussianVariable(gaussianValue);
  NS_LOG_INFO("Gaussian na classe  " << LOS->GetGaussianVariable() << " gaussiano");

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  Ptr<LoraChannel> channelLOS = CreateObject<LoraChannel> (LOS, delay);

  Ptr<LoraChannel> channelNLOS = CreateObject<LoraChannel> (NLOS, delay);


//TODO Ajeitar LOS/NLOS

  // Ptr<LoraChannel> channelLOS = CreateObject<LoraChannel> (LOS, delay);
  // Ptr<LoraChannel> channelNLOS = CreateObject<LoraChannel> (NLOS, delay);
  // 

  /************************
  *  Create the helpers  *
  ************************/

  NS_LOG_INFO ("Setting up helpers...");

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
  			"rho", DoubleValue (radius),
  			"X", DoubleValue (0.0),
  			"Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

 
//TODO Ajeitar LOS/NLOS

 /*
 LoraPhyHelper phyHelperLOS = LoraPhyHelper ();
  phyHelperLOS.SetChannel (channelLOS); 

  LoraPhyHelper phyHelperNLOS = LoraPhyHelper ();
  phyHelperNLOS.SetChannel (channel);
 */
  // Create the LoraMacHelper
  LoraMacHelper macHelper = LoraMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (chFilename); // Output filename

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();


  /************************
  *  Create End Devices  *
  ************************/

//   NS_LOG_INFO ("Creating the end device...");

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to the node
  mobility.Install (endDevices);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LoraMacHelper::ED);

//TODO Ajeitar LOS/NLOS

/*
  phyHelperLOS.SetDeviceType (LoraPhyHelper::ED);
  phyHelperNLOS.SetDeviceType (LoraPhyHelper::ED);


*/

  NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);

//TODO Ajeitar LOS/NLOS

/*
  NetDeviceContainer endDevicesNetDevicesLOS = helper.Install (phyHelperLOS, macHelper, endDevices);
  NetDeviceContainer endDevicesNetDevicesNLOS = helper.Install (phyHelperNLOS, macHelper, endDevices);


*/
  /*********************
   *  Create Gateways  *
   *********************/

  NS_LOG_INFO ("Creating the gateway...");
  NodeContainer gateways;
  gateways.Create (1);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();//ADRcode
  	// Make it so that nodes are at a certain height > 0 //ADRcode
  allocator->Add (Vector (0.0, 0.0, 15.0)); //ADRcode

  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LoraMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

//TODO ADICIONAR LOSNLOS
  /*
  phyHelperLOS.SetDeviceType (LoraPhyHelper::GW);
  phyHelperNLOS.SetDeviceType (LoraPhyHelper::GW);
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


  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel, algoritmo, txPowerdBm);

 //TODO LOSNLOS
  /*
  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channelLOS, algoritmo, txPowerdBm);
  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channelNLOS, algoritmo, txPowerdBm);
  

  */


  for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
      Ptr<Node> node = endDevices.Get(i);
      for (uint32_t j = 0; j < node->GetNDevices(); ++j) {
          Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(j));

          if (loraNetDevice) {
              Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLoraMac>();
              if (mac) {
                  mac->SetTransmissionPower(txPowerdBm);
                //   std::cout << "Potencia de transmissao para o dispositivo" << i << ": " << txPowerdBm << " dBm\n";

              } else {
            	  std::cerr << "Erro: MAC nao encontrado para o dispositivo " << i << "\n";
                  }
          }
      }
  }
  std::cout << "Potencia de transmissao: " << txPowerdBm << " dBm\n algoritmo: " << algoritmo << " raio: " << radius << "\n";




  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  PeriodicSenderHelper periodicSenderHelper;
  periodicSenderHelper.SetPeriod (Seconds (appPeriodsSeconds));
    periodicSenderHelper.SetPacketSize (packetsize);

  periodicSenderHelper.Install (endDevices);

  /************************
   * Install Energy Model *
   ************************/

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
    int j = 0;

  simulationTime = hours*60*60;
  uint32_t nDevicesNLOS = nDevices * 0.8; //TODO 
   for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        Ptr<Node> node = endDevices.Get(i);
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();

        if (i < nDevicesNLOS)
        {
            phyHelper.SetChannel(channelNLOS);
            std::cout << "Dispositivo " << i << " no canal NLOS " << "\n";

        }
        else
        {
            // phy->SetChannel(channelLOS);
            phyHelper.SetChannel(channelLOS);
            std::cout << "Dispositivo " << i << " no canal LOS " << "\n";

        }
    }

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
  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop (Hours (hours));

bool visada;

//TODO ADICIONAR LOS/NLOS
for(int i=10; i < simulationTime; i += MabInterval){
        // Simulator::Schedule (Seconds(i), &ReallocateChannels, endDevices, channelNLOS, channelLOS);
   		Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelNLOS, txPowerdBm, macHelper, visada=0);
   		Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevices, gateways, channelLOS, txPowerdBm, macHelper, visada=1);
        Simulator::Schedule (Seconds(i), &SaveMABmetrics, chFilenameMAB, local);
        j++;
	}
  Simulator::Run ();

  double energy = 0;
  for(int i=0; i<nDevices; i++){
	  energy += sources.Get(i)->GetRemainingEnergy();
//	  NS_LOG_INFO("energia restante do dispositivo " << i << " igual a " << sources.Get(i)->GetRemainingEnergy());

  }
  batteryEnergyFinal = energy/nDevices;

  Simulator::Destroy ();

  NS_LOG_INFO ("Computing performance metrics...");

  helper.PrintPerformance (Seconds(0), Hours(hours));

  toc();
  myfile.open (chFilename.c_str(), std::ofstream::app);
  myfile << batteryEnergyFinal;
  myfile.close ();

  return 0;
}
