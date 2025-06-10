#include "LoRaBandit_v3.h"

// Construtor: inicializa as variáveis
LoRaBandit::LoRaBandit(int NaA, int NaP, int W) 
    : Q(NaA, std::vector<uint8_t>(NaP, 0)), 
    rewards(NaA, std::vector<std::deque<uint8_t>>(NaP)), 
    sizeA(NaA),
    sizeP(NaP),
    windowSize(W), 
    epsilon(0.45), 
    //actions(Na, 0.0), 
    trackActionA(0),
    trackActionP(0), 
    resphySucc(0), 
    resphyTotal(0) {
}

LoRaBandit::~LoRaBandit() {}

// Imprimir a matriz de recompensas
void LoRaBandit::Print() {
    std::cout << "Q Matrix:" << std::endl;
    for (int i = 0; i < sizeA; i++) {
        for (int j = 0; j < sizeP; j++) {
            std::cout << Q[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "=================================================" << std::endl;
}

void LoRaBandit::resetBandits() {
    for (int i = 0; i < sizeA; i++) {
        for (int j = 0; j < sizeP; j++) {
            while (!rewards[i][j].empty()) {
                rewards[i][j].pop_front();
            }
            Q[i][j] = 0.0;
        }
    }
    epsilon = 0.45;
}

// Obter a melhor ação (máxima recompensa)
std::pair<int, int> LoRaBandit::GetMax() {
    double maxVal = -1.0;
    int bestX = 0, bestY = 0;
    for (int i = 0; i < sizeA; i++) {
        for (int j = 0; j < sizeP; j++) {
            if (Q[i][j] > maxVal) {
                maxVal = Q[i][j];
                bestX = i;
                bestY = j;
            }
        }
    }
    return {bestX, bestY};
}

// Obter a ação atual
std::pair<int, int> LoRaBandit::GetAction() {
    return {trackActionA, trackActionP};
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

/*
int LoRaBandit::GetMax() { 
    return std::distance(Q.begin(), std::max_element(Q.begin(), Q.end()));
}

int LoRaBandit::GetAction() {
    return trackAction + 1;
}
*/

void LoRaBandit::Run() {
    double PDR = (resphyTotal != 0) ? static_cast<double>(resphySucc) / static_cast<double>(resphyTotal) : 0.0;
    std::ofstream myfile(fileName, std::ios_base::app);
    //myfile << actions[trackAction] << " " << PDR << " ; ";

    // Atualiza a janela deslizante
    auto& window = rewards[trackActionA][trackActionP];
    window.push_back(PDR);
    if (window.size() > windowSize) {
        window.pop_front(); // Remove a recompensa mais antiga
    }

    // Calcula a média da janela deslizante
    double sum = 0.0;
    for (const auto& reward : window) {
        sum += reward;
    }
    Q[trackActionA][trackActionP] = sum / window.size();

    Print();
    
    // Seleção da próxima ação usando e-greedy
    double e = static_cast<double>(std::rand()) / RAND_MAX;
    int nextX, nextY;
    if (e < epsilon) {
        nextX = std::rand() % sizeA;
        nextY = std::rand() % sizeP;
        epsilon /= 1.02; // Reduz epsilon em 2%
    } else {
        std::tie(nextX, nextY) = GetMax();
    }

    // Registro da ação escolhida
    trackActionA = nextX;
    trackActionP = nextY;

    myfile << "Selected Action: (" << nextX << ", " << nextY << ") | PDR: " << PDR << std::endl;

    myfile.close();
    ResetPDR();
}