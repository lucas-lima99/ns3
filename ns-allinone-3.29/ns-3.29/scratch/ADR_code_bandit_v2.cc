#include <vector>
#include <tuple>
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
#include <iomanip> // Para manipulação de saída formatada




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
//lista de todos os pacotes transmitidos pelas vacas no intervalo de tempo de atuação do MAB
std::vector<std::tuple<Ptr<Packet const>>> Txlistcows;
std::vector<std::tuple<Ptr<Packet const>>> Rxlistcows;

//-------------------------------VARIÁVEIS GLOBAIS DE ENTRADA ------------------------------------------------------
	int hoursduration = 4; // duração de confinamento das vacas em horas;
    int confinTimer = 1*60*60;

	int nEDcows =   1128; //  5076 10011 (has to be multiple of 8) número de vacas
	int nGateways = 1; //número de gateways
    int apv = 5;
	int radiusHotspot = 15;    // raio do hostpost das vacas.
	int nHotspot= 8; // 36  71 número de hostpots
	int radius = 9000; //metros //raio da área de simulação
    int radiusStep = 1500;
	int appPeriodSecondscows = 1200; // periodo da aplicação das vacas
	//tempo de simulação

	//LoRaWan Parameters
	int PacketSize = 20;
	double PathLoss = 3.42;
	
	int seed=1;
	uint32_t runSeed=1;
	int algoritmo=6;
	double targetRealocation = 5;
	bool realisticChannelModel = false;
	
	int periodsToSimulate = 1;
	int transientPeriods = 0;
	double desvpadrao;

	std::string SimMode = "confinado";

	std::string outputDir = "./";
	std::string chfilenamepositions;
    std::string outputDirpositions = "./";
	std::string filename = "DadosLora";
    std::string fileMAB = "DadosMAB";
	std::string chFilenamegeral;    //File to store the node metric      
    std::string chFilenameMAB;      //File tp store MAB data


//---------------------------------VARIÁVEIS GLOBAIS PARA UTILIZAÇÃO DO CÓDIGO -----------------------------//

	//transformação das variáveis de tempo para segundos.
    int nDevices = nEDcows;
	int simulationTime= hoursduration*60*60;

	bool fixedSeed = false;
	Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();

	bool firstTime;



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

	std::cout << "Time elapsed: " << simTime << " minutes" << std::endl;

	return simTime;

}

void PrintChosenAction(int Action){
	switch(Action){
		case 1:
			std::cout << "I - Fixo em SF 7" << std::endl;
			break;
		case 2:
			std::cout << "II - Fixo em SF 12" << std::endl;
			break;
		case 3:
			std::cout << "III - Igualmente dividido" << std::endl;
			break;
		case 4:
			std::cout << "IV - Arbitrariamente dividido | Aumento de capacidade" << std::endl;
			break;
		case 5:
			std::cout << "V - Arbitrariamente dividido | Ampliação da cobertura" << std::endl;
			break;
		case 6:
			std::cout << "VI - Baseado na Sensibilidade" << std::endl;
			break;
		case 7:
			std::cout << "VII - Arbitrariamente Dividido Baseado na Sensibilidade | Aumento de capacidade" << std::endl;
			break;
		case 8:
			std::cout << "VIII - Arbitrariamente Dividido Baseado na Sensibilidade | Aumento de cobertura" << std::endl;
			break;
        case 9:
			std::cout << "XI - Distribuído aleatoriamente" << std::endl;
            break;
        case 10:
            std::cout << "Sensitivity algorithm with reallocation" << std::endl;
            break;
		case 11:
			std::cout << "desconcentrar muito estando perto" << std::endl;
			break;
		case 12:
			std::cout << "desconcentrar estando perto" << std::endl;
			break;
		case 13:
			std::cout << "desconcentrar estando longe" << std::endl;
			break;
		default:
			std::cout << "desconcentrar muito estando longe" << std::endl;
	}
}

void callprintpositions (NodeContainer endDevices, NodeContainer gateways, std::string chfilenamepositions, LoraHelper helper){
 	helper.PrintEndDevices (endDevices, gateways, chfilenamepositions);
 }

 NodeContainer create_gateways(int nGateways, MobilityHelper mobility){
    // Create the gateway nodes (allocate them uniformely on the disc)
	NodeContainer gateways;
	gateways.Create (nGateways);

	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
	// Make it so that nodes are at a certain height > 0
	allocator->Add (Vector (0.0, 0.0, 15.0));
	mobility.SetPositionAllocator (allocator);
	mobility.Install (gateways);

    return gateways;
}

void create_EDs(int radiusSede, int nvpHotspot, NodeContainer& endDevicescows, NodeContainer*& endDevicescowsh,MobilityHelper& mobilitycows, MobilityHelper*& mobilitycowsh){
    // create a mobility model for the nodes when extensive mode
    mobilitycowsh = new MobilityHelper[nHotspot];
    endDevicescowsh = new NodeContainer [nHotspot];

    double angle = 2*M_PIl/nHotspot;
	Ptr<UniformRandomVariable> randomRadius = CreateObject<UniformRandomVariable>();

    for (int j = 0; j < nHotspot; j++)
    {   
        endDevicescowsh[j].Create(nvpHotspot);
         
        int Radiusaux = randomRadius->GetInteger(radiusSede, radius);

        mobilitycowsh[j].SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
            "rho", DoubleValue (radiusHotspot),
            "X", DoubleValue (Radiusaux*cos(angle*j)),
            "Y", DoubleValue (Radiusaux*sin(angle*j)));
        mobilitycowsh[j].SetMobilityModel ("ns3::ConstantPositionMobilityModel");

        mobilitycowsh[j].Install(endDevicescowsh[j]);
        endDevicescows.Add(endDevicescowsh[j]);
    }

    // create a mobility model for the nodes when intensive mode
    mobilitycows.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
            "rho", DoubleValue (radiusSede),
            "X", DoubleValue (0.0),
            "Y", DoubleValue (0.0));
    mobilitycows.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
}

//Muda os dispositivos da sede para o pasto
 void changemobilitycowstoh(int nHostpot, MobilityHelper * mobilitycowsh, NodeContainer * endDevicescowsh){
    std::cout << "==========================================================================" << std::endl;
    std::cout << "                      O Gado saiu para o pasto" << std::endl;
    std::cout << "==========================================================================" << std::endl;
    
    for (int j=0; j<nHostpot; j++)
    {
        mobilitycowsh[j].Install (endDevicescowsh[j]);
    }
    //Reseta o vetor Q e N
    LoRaMAB.resetBandits();
}

//faz a mudança dos End devices do pasto para a sede
void changemobilitycowstos (MobilityHelper mobilitycows,  NodeContainer endDevicescows){
    std::cout << "==========================================================================" << std::endl;
    std::cout << "                          O Gado está na sede" << std::endl;
    std::cout << "==========================================================================" << std::endl;
	mobilitycows.Install (endDevicescows);
    //Reseta o vetor Q e N
    LoRaMAB.resetBandits();
}

//Funcao para mover os dispositivos para novas posicoes no cenario extensivo
void SetMobilityPositions(int nHotspot, int radiusSede, MobilityHelper *&mobilitycowsh, NodeContainer *&endDevicescowsh, NodeContainer &endDevicescows) {
    //int newRadio = radiusStep;
    double angle = 2 * M_PI / nHotspot;
	static bool isFirstCall = true;

    Ptr<UniformRandomVariable> randomRadius = CreateObject<UniformRandomVariable>();

	// Criar um conjunto para armazenar os valores únicos gerados
    //std::set<std::pair<int, int>> uniqueValues;

    for (int j = 0; j < nHotspot; j++) {

        int Radiusaux = (radiusStep <= radius) ? (randomRadius->GetValue(radiusSede, radiusStep)) : (randomRadius->GetValue(radiusSede, radius));

        mobilitycowsh[j].SetPositionAllocator("ns3::UniformDiscPositionAllocator",
            "rho", DoubleValue(radiusHotspot),
            "X", DoubleValue(Radiusaux * cos(angle * j)),
            "Y", DoubleValue(Radiusaux * sin(angle * j)));
        mobilitycowsh[j].SetMobilityModel("ns3::ConstantPositionMobilityModel");

        mobilitycowsh[j].Install(endDevicescowsh[j]);
        endDevicescows.Add(endDevicescowsh[j]);
    }
    if (isFirstCall) {
        LoRaMAB.resetBandits();
        isFirstCall = false;
    }
    radiusStep += 1500;
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

//função de callback, sempre que um pacote é transmitido por uma vaca, o pacote é adicionado a lista Txlistcows.
void getTxlistcows (Ptr<Packet const> packet, uint32_t systemId){
	Txlistcows.push_back(std::make_tuple(packet));
	LoRaMAB.SetPHYtot(1);
}

void getRxlistcows(Ptr<const Packet> packet, uint32_t systemId){
	Rxlistcows.push_back(std::make_tuple(packet));
	LoRaMAB.SetPHYsucc(1);
}

void LoRaBanditRun(NodeContainer endDevicescows, NodeContainer gateways, Ptr<LoraChannel> channel,double targetRealocation, LoraMacHelper macHelper){ //preciso criar um contexto para buscar as variaveis (endDevicescows, gateways, channel)
	//std::cout << "LoRaBandit Running..." << std::endl;
	LoRaMAB.Run();
	int Action = LoRaMAB.GetAction();
	//PrintChosenAction(Action+1);
	macHelper.SetSpreadingFactorsUp (endDevicescows, gateways, channel, (Action), targetRealocation);
}


int main (int argc, char *argv[])
{
	tic();
	// Network settings

	CommandLine cmd;
	
	cmd.AddValue ("radius",
			"Raio da fazenda",
			radius);
	cmd.AddValue ("radiusHotspot",
			"total number of cows to include in the simulation",
			radiusHotspot);
	cmd.AddValue ("nEDcows",
			"total number of cows to include in the simulation",
			nEDcows);
	cmd.AddValue ("appPeriodSecondscows",
			"The period in seconds to be used by periodically transmitting cows applications",
			appPeriodSecondscows);
	cmd.AddValue ("simulationTime",
			"Duration (in hours) of the confinement",
			simulationTime);
	cmd.AddValue ("PacketSize",
			"Payload of the packets",
			PacketSize);
	cmd.AddValue ("runSeed",
			"Set runseed",
			runSeed);
	cmd.AddValue ("outputDir",
			"Output directory",
			outputDir);
	cmd.AddValue ("fixedSeed",
			"Enable fixed seed and runSeed",
			fixedSeed);
	cmd.AddValue ("targetRealocation",
			"Porcentagem de realocação de usuários do SF [%]",
			targetRealocation);
	cmd.AddValue ("filename",
			"Output file name",
			filename);
	cmd.AddValue ("algoritmo",
			"Algoritmo de alocaçao de SF a ser utilizado",
			algoritmo);
	cmd.AddValue ("SimMode",
			"Algoritmo de alocaçao de SF a ser utilizado",
			SimMode);
	
	//
	cmd.Parse (argc, argv);
    
	//SimMode = SimMode;
	std::cout << SimMode << std::endl;

	cmd.Parse (argc, argv);
    int A_hotspot = M_PIl * pow(radiusHotspot, 2); //radiusHotspot^2 ; 
	int nvpHotspot = floor(A_hotspot/apv);
	nHotspot = ceil(nEDcows/nvpHotspot);
	int nDevices = nEDcows;

	//Area Sede
	int A_tot = apv * nEDcows; 
	int radiusSede = ceil(sqrt(A_tot/M_PIl));
	
	if (fixedSeed){
		RngSeedManager::SetSeed(seed);
		RngSeedManager::SetRun(runSeed);
	}

	//Getting seed and runSeed for checking and displaying purposes
	seed = RngSeedManager::GetSeed();
	runSeed = RngSeedManager::GetRun();

	chFilenamegeral = outputDir+"/"+ filename +".txt";
    chFilenameMAB = outputDir+"/"+ fileMAB +".txt";

	std::ifstream file(chFilenamegeral.c_str());
    //std::ifstream fileMAB(chFilenamegeral.c_str());
	firstTime = is_empty(file);
	file.close();
	std::ofstream myfile;
	myfile.open(chFilenamegeral.c_str(), std::ofstream::app);


	if ( firstTime )
	{
		// file is empty: write down the header
		myfile <<
				"RunSeed," <<
				"Seed," <<
				"Algoritmo," <<
				"Raio," <<
				"RaioHotspot," <<
				"nHotspots," <<
				"nDevices," <<
				"simulationTime," <<
				"PHYTotal," <<
				"PHYSuccessful," <<
				"PHYInterfered," <<
				"PHYNoMoreReceivers," <<
				"PHYUnderSensitivity," <<
				"PHYLostBecauseTX," <<
				"SimulationDuration," <<
				"targetRealocation";
	}
    
    myfile << std::endl <<
			runSeed << "," <<
			seed << "," <<
			algoritmo << "," <<
			radius << "," <<
			radiusHotspot << "," <<
			nHotspot<< "," <<
			nDevices << "," <<
			simulationTime << ",";
	myfile.close ();

	
	std::cout <<
			"Running with nDevices = " << nDevices <<
			", radius = " << radius << " m" <<
			", simulation Time =" << simulationTime << " s" <<
			", Algoritmo ADR = " << algoritmo <<
			", n° Hotspots = " << nHotspot <<
            ", targetRealocation = " << targetRealocation <<
			", seed = " << seed << std::endl;
	
	 
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
	loss->SetPathLossExponent (PathLoss); //https://biblioteca.sbrt.org.br/articles/2874
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
	helper.EnablePacketTracking (chFilenamegeral); 
	
	//Create the NetworkServerHelper
	NetworkServerHelper nsHelper = NetworkServerHelper ();

	//Create the ForwarderHelper
	ForwarderHelper forHelper = ForwarderHelper ();

	/*********************
	 *  Create Gateways  *
	 *********************/

	// Create the gateway nodes (allocate them uniformely on the disc)
	NodeContainer gateways = create_gateways(nGateways, mobility);

	// Create a netdevice for each gateway
	phyHelper.SetDeviceType (LoraPhyHelper::GW);
	macHelper.SetDeviceType (LoraMacHelper::GW);
	helper.Install (phyHelper, macHelper, gateways);

	/************************
	 *  Create End Devices  *
	 ************************/
    NodeContainer endDevicescows;
    NodeContainer *endDevicescowsh;
    MobilityHelper mobilitycows;
    MobilityHelper *mobilitycowsh;

    //Criando os EDs e setando valores default de posicao e mobilidade
    create_EDs(radiusSede, nvpHotspot, endDevicescows, endDevicescowsh, mobilitycows, mobilitycowsh);

    // Make it so that nodes are at a certain height > 0
    for (NodeContainer::Iterator j = endDevicescows.Begin (); j != endDevicescows.End (); ++j)
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

	macHelper.SetAddressGenerator (addrGen);
	phyHelper.SetDeviceType (LoraPhyHelper::ED);
	macHelper.SetDeviceType (LoraMacHelper::ED);
	helper.Install (phyHelper, macHelper, endDevicescows);
	
	// Now end devices are connected to the channel

	macHelper.SetSpreadingFactorsUp (endDevicescows, gateways, channel, algoritmo, targetRealocation);

	/*********************************************
	 *  Install applications on the end devices  *
	 *********************************************/


    // -------------aplicação de proteção contra animais -----------------------------------------------
    Time appStopTime = Seconds (simulationTime);
	
	// ---------------------aplicação vacas -------------------------------------------

	PeriodicSenderHelper appHelpercows = PeriodicSenderHelper ();
	appHelpercows.SetPeriod (Seconds (appPeriodSecondscows));
	appHelpercows.SetPacketSize (PacketSize);
	ApplicationContainer appContainercows = appHelpercows.Install (endDevicescows);
	appContainercows.Start (Seconds (0));
	appContainercows.Stop (appStopTime);

	/**************************
	 *  Create Network Server  *
	 ***************************/

	// Create the NS node
	NodeContainer networkServer;
	networkServer.Create (1);

	// Create a NS for the network
	nsHelper.SetEndDevices (endDevicescows);
	nsHelper.SetGateways (gateways);
	nsHelper.Install (networkServer);

	//Create a forwarder for each gateway
	forHelper.Install (gateways);

	//sempre que um pacote é recebido, ele é adicionado à lista Rx
	for (NodeContainer::Iterator j = gateways.Begin ();
				j != gateways.End (); ++j)
		{
			Ptr<Node> node = *j;
			
			Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
			Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
			phy->TraceConnectWithoutContext ("ReceivedPacket",
												MakeCallback
													(&getRxlistcows));	
		}

	//sempre que um pacote é transmitido, ele é adicionado à lista Tx
	for (NodeContainer::Iterator j = endDevicescows.Begin ();
			j != endDevicescows.End (); ++j)
	{
		Ptr<Node> node = *j;
		
		Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
		Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
		phy->TraceConnectWithoutContext ("StartSending",
                                               MakeCallback
                                                 (&getTxlistcows));
		
	}
	
//=============================================================================================================================================
                    //DEFININDO O ESCOPO DA SIMULACAO
//=============================================================================================================================================
    Simulator::Schedule (Seconds(0), &changemobilitycowstos, mobilitycows, endDevicescows);
    //Simulator::Schedule (Seconds(1), &callprintpositions, endDevicescows, gateways, chfilenamepositions + "_Confin", helper);


	for(int i=confinTimer; i < simulationTime; i += ChangePositionsInterval){
        Simulator::Schedule (Seconds(i), &SetMobilityPositions, nHotspot, radiusSede, mobilitycowsh, endDevicescowsh, endDevicescows);
	    //Simulator::Schedule (Seconds(i+1), &callprintpositions, endDevicescows, gateways, chfilenamepositions+ "_Hotspots_" + std::to_string(i+1), helper);

    }

    for(int i=10; i < simulationTime; i += MabInterval){
   		Simulator::Schedule (Seconds(i), &LoRaBanditRun, endDevicescows, gateways, channel, targetRealocation, macHelper);
	}


//====================================================================================================================================================
                                            //SAVING MAB METRICS
//====================================================================================================================================================
    //Simulator::Schedule (Seconds(confinTimer-1), &SaveMABmetrics, chFilenameMAB, "confinamento");
    std::string local;
    int j = 0;

    for(int i=confinTimer-1; i < simulationTime; i += ChangePositionsInterval){
        if (i==confinTimer-1){
            local = "Confinamento ";
        } else{
            local = "Pasto " + std::to_string(j);
        }
	    Simulator::Schedule (Seconds(i), &SaveMABmetrics, chFilenameMAB, local);
        j++;
    }
//=============================================================================================================================================
//=============================================================================================================================================


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

	//int Rx = std::get<0>(Rxlistcows);

    double simDuration = toc();

    helper.PrintPerformance(Seconds(0), Seconds(simulationTime));

	myfile.open(chFilenamegeral.c_str(), std::ofstream::app);
	myfile << simDuration << ",";
	myfile << targetRealocation;
	myfile.close ();

    delete[] endDevicescowsh;
    delete[] mobilitycowsh;
	 
	return 0;
}