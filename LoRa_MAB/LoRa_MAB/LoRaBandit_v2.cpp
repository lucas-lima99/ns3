#include <iostream>
#include "LoRaBandit_v2.h"
#include <vector>

//Construtor
//Inicializando parâmetros
LoRaBandit::LoRaBandit(int Na) : Q(Na, 0.0), N(Na, 0.0), size(Na), node(0), PDR(0.0), avgSF(0), resphySucc(0), resphyTotal(0), epsilon(0.45), actions(Na, 0.0), trackAction(0), fileName("") {std::cout << Q[0] << std::endl;}
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

void LoRaBandit::resetBandits() {
    std::fill(Q.begin(), Q.end(), 0.0);
    std::fill(N.begin(), N.end(), 0.0);
    epsilon = 0.4;
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

void LoRaBandit::Run() {
    //double PDR = static_cast<double>(resphySucc) / static_cast<double>(resphyTotal);
    double PDR = (resphyTotal != 0) ? static_cast<double>(resphySucc) / static_cast<double>(resphyTotal) : 0.0;
    std::ofstream myfile(fileName, std::ios_base::app);
    myfile << actions[trackAction] << " " << PDR <<" ; ";
    
    N[trackAction] += 1;
    Q[trackAction] = Q[trackAction] + (PDR - Q[trackAction])/N[trackAction];
    
    Print();
    
    double e = static_cast<double>(std::rand()) / RAND_MAX;
    int A;
    //e-greedy
    if (e < epsilon) {
        A = std::rand() % size;
        epsilon /= 1.02;       //Reduz epsilon em 2%
    } else {
        A = GetMax();
    }
    
    myfile << actions[A] << std::endl;
    myfile.close();
    trackAction = A;
    ResetPDR();
    PDR = 0.0;
}