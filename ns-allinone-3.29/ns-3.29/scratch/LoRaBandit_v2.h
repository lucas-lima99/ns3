#ifndef LORABANDIT_H
#define LORABANDIT_H

#include <iostream>
#include <string>
#include <iterator>
#include <algorithm> 
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string.h>
#include <vector>


class LoRaBandit {
private:
    std::vector<double> Q;
    std::vector<double> N;
    int size;
    int node;
    double PDR;
    int avgSF;
    int resphySucc;
    int resphyTotal;
    double epsilon;
    std::vector<float> actions;
    int trackAction;
    std::string fileName;
    double energy;
    double currentsf7;
    double currentsf12;


public:
    LoRaBandit(int Na);
    ~LoRaBandit();
    void Run();
    void Print();
    void resetBandits();
    void SetPHYsucc(uint32_t data);
    void SetPHYtot(uint32_t data);
    void SetEnergy(double data);
    double GetEnergy();
    void SetCurrentSF(double data1, double data2);
    std::pair<double,double> GetCurrentSF();
    int GetPHYtot();
    int GetPHYsucc();
    void ResetPDR();
    void SetSF(uint32_t data);
    int GetSF();
    double *GetQ();
    double *GetN();
    int GetMax();
    int GetAction();
    int GetSize() const {return size;};
};

#endif // LORA_BANDIT_H