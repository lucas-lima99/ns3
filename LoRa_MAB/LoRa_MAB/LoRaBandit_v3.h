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
#include <vector>
#include <deque>
#include <numeric>

class LoRaBandit {
private:
    std::vector<std::vector<uint8_t>> Q;                     // Valor esperado de cada braço
    //std::vector<std::deque<double>> rewards;  // Recompensas dentro da janela para cada braço
    std::vector<std::vector<std::deque<uint8_t>>> rewards;
    int sizeA;                                 // Número de braços
    int sizeP;
    int windowSize;                           // Tamanho da janela deslizante
    double epsilon;                           // Taxa de exploração
    std::vector<float> actions;               // Braços disponíveis
    int trackActionA;                          // Ação selecionada
    int trackActionP;
    std::string fileName;
    int resphySucc;
    int resphyTotal;

public:
    LoRaBandit(int NaA, int NaP, int W);
    ~LoRaBandit();
    void Run();
    void Print();
    void resetBandits();
    void SetPHYsucc(uint32_t data);
    void SetPHYtot(uint32_t data);
    void ResetPDR();
    //int GetMax();
    //int GetAction();
    std::pair<int, int> GetMax();
    std::pair<int, int> GetAction(); 
};

#endif //LORABANDIT_H