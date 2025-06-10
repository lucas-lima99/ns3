#include <iostream>
#include "LoRaBandit_v2.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/lora-phy.h"
#include <vector>
#include "ns3/basic-energy-source.h"


//Construtor
//Inicializando parâmetros
LoRaBandit::LoRaBandit(int Na) : Q(Na, 0.0), N(Na, 0.0), size(Na), node(0), PDR(0.0), avgSF(0), resphySucc(0), resphyTotal(0), epsilon(0.95), actions(Na, 0.0), trackAction(0), fileName("") {std::cout << Q[0] << std::endl;}


LoRaBandit::~LoRaBandit() {}

void LoRaBandit::Print() {
    std::cout << "Q: [ ";
    for (int i = 0; i < size; i++) {
        std::cout << Q[i] << " ";
    }
    std::cout << "]" << std::endl;
    std::cout << "N: [ ";
    for (int i = 0; i < size; i++) {
        std::cout << N[i] << " ";
    }
    std::cout << "]" << std::endl << "=================================================" << std::endl;
}

double *LoRaBandit::GetQ() {
    return Q.data();
}

double *LoRaBandit::GetN() {
    return N.data();
}

double LoRaBandit::GetEnergy() {
    return energy;
}

void LoRaBandit::SetEnergy(double data) {
    energy = data;
}

void LoRaBandit::resetBandits() {
    std::fill(Q.begin(), Q.end(), 0.0);
    std::fill(N.begin(), N.end(), 0.0);
    epsilon = 0.9;
}

void LoRaBandit::SetSF(uint32_t data) {
    avgSF += data;
}

int LoRaBandit::GetSF() {
    return avgSF;
}

void LoRaBandit::SetPHYsucc(uint32_t data) {
    resphySucc += data;
}

void LoRaBandit::SetPHYtot(uint32_t data) {
    resphyTotal += data;
}
int LoRaBandit::GetPHYtot() {
    return resphyTotal;
}
int LoRaBandit::GetPHYsucc() {
    return resphySucc;
}
void LoRaBandit::ResetPDR() {
    resphySucc = 0;
    resphyTotal = 0;
}

int LoRaBandit::GetMax() { 
    //index of the max element in the Q array 
    return std::distance(Q.begin(), std::max_element(Q.begin(), Q.end()));
}

int LoRaBandit::GetAction() {
    return trackAction+1;
}

void LoRaBandit::SetCurrentSF(double data1, double data2) {
    currentsf7 = data1;
    currentsf12 = data2;
}

std::pair<double, double> LoRaBandit::GetCurrentSF() {
    return {currentsf7, currentsf12};
}
void LoRaBandit::Run() {
    float alpha = 0.75;
    float beta = 1 - alpha;
    // double currentsf12 = 0.102611;
    // double currentsf7 = 0.005959;
    std::pair<double, double> currentsfs = GetCurrentSF();
    double currentsf7 = currentsfs.first;
    double currentsf12 = currentsfs.second;
    // std::cout << "current sf7 " << currentsf7 << " current sf12 " << currentsf12 << std::endl;

    double current = GetEnergy();
    double current_normalizada = (current - currentsf7) / (currentsf12 - currentsf7);
    double PDR = (resphyTotal != 0) ? static_cast<double>(resphySucc) / static_cast<double>(resphyTotal) : 0.0;
    double QoSE = (alpha*PDR) + (beta*(1 - current_normalizada));

    std::ofstream myfile(fileName, std::ios_base::app);
    myfile << actions[trackAction] << " " << QoSE <<" ; "; // anterior
    
    N[trackAction] += 1;
    Q[trackAction] = Q[trackAction] + (QoSE - Q[trackAction])/N[trackAction];
    
    Print();
    
    double e = static_cast<double>(std::rand()) / RAND_MAX;
    int A;
    //e-greedy
    if (e < epsilon) {
        A = std::rand() % size;
        epsilon /= 1.01;       //Reduz epsilon em 4%
    } else {
        A = GetMax();
    }

    
    
    myfile << actions[A] << std::endl;
    myfile.close();
    trackAction = A;

    // Ptr<LoraPhy> loraphy = CreateObject<LoraPhy>();
    // Time airTime = loraphy->GetOnAirTime(packet, txParams);

    std::cout << "total de pacotes enviados: "<< GetPHYtot() << " Pacotes recebidos: " << GetPHYsucc() << " PDR momentânea: " << PDR  << " QoSE momentâneo " << QoSE << std::endl;
    ResetPDR();
    // PDR = 0.0;
    // old_energy = energy;
}