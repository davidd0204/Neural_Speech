#include <Arduino.h>

#define SAMPLE_RATE 44000 // Fréquence d'échantillonnage en Hz
#define BUFFER_SIZE 41 // Taille du buffer égale au nombre de coefficients du filtre
#define OUTPUT_SIZE 1 // Taille de la sortie égale à 1 pour un échantillon filtré à chaque fois

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 44000 Hz

fixed point precision: 16 bits

* 0 Hz - 15000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 16000 Hz - 22000 Hz
  gain = 0
  desired attenuation = -30 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM 35

static int filter_taps[FILTER_TAP_NUM] = {
  -698,
  1599,
  2716,
  -76,
  -332,
  952,
  -735,
  -46,
  945,
  -1212,
  440,
  1011,
  -2065,
  1524,
  1068,
  -4949,
  8437,
  22930,
  8437,
  -4949,
  1068,
  1524,
  -2065,
  1011,
  440,
  -1212,
  945,
  -46,
  -735,
  952,
  -332,
  -76,
  2716,
  1599,
  -698
};

uint16_t inputSignal[BUFFER_SIZE] = {0};
uint16_t filteredSignal[OUTPUT_SIZE];
volatile int bufferIndex = 0;

void setupADC() {
    PMC->PMC_PCER1 |= PMC_PCER1_PID37; // Active le périphérique ADC
    ADC->ADC_MR = ADC_MR_PRESCAL(0) // Définit le diviseur de fréquence à 255
                | ADC_MR_STARTUP_SUT64 // Définit le temps de démarrage à 64 périodes d'ADC_CLK
                | ADC_MR_TRACKTIM(15) // Définit le temps de suivi à 15 périodes d'ADC_CLK
                | ADC_MR_SETTLING_AST3;// Définit le temps de stabilisation à 17 périodes d'ADC_CLK
    ADC->ADC_CHER = ADC_CHER_CH7; // Active le canal 7 (A0)
    
    // Configure Timer Counter 0 Channel 0 (TC0) pour samplingFrequency
    PMC->PMC_PCER0 |= PMC_PCER0_PID27; // Active le périphérique TC0
    TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
    // Définit la source d'horloge à TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
    // Active le déclenchement de comparaison RC
    // Définit la valeur RC pour une fréquence de 44 kHz
    TC0->TC_CHANNEL[0].TC_RC = 21; // 32 kHz -> ( MCK / 128 / 32000 ) -> 21
    
    // Active l'interruption de comparaison RC
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    
    // Active l'interruption TC0_IRQn dans le NVIC
    NVIC_EnableIRQ(TC0_IRQn);
    
    // Activation du compteur
    TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void setupDAC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID38; // Active le périphérique DAC
  // Configure le DAC en mode normal
  DACC->DACC_MR = DACC_MR_TRGEN_DIS // Désactive le déclencheur externe
                  | DACC_MR_USER_SEL_CHANNEL1 // Sélectionne le canal 1
                  | DACC_MR_WORD_HALF // Largeur de mot de 16 bits (0 - 4095)
                  | DACC_MR_REFRESH(1) // Temps de rafraîchissement (dans les cycles de l'horloge du périphérique)
                  | DACC_MR_STARTUP_8 // Temps de démarrage (8 * 6 cycles)
                  | DACC_MR_MAXS; // Utilise le contrôleur DMA pour les transferts DAC
  // Active le canal 1 du DAC
  DACC->DACC_CHER = DACC_CHER_CH1;
  DACC->DACC_IER |= DACC_IER_EOC;
  // Active l'interruption DACC_IRQn dans le NVIC 
  NVIC_EnableIRQ(DACC_IRQn);
}

void TC0_Handler() {
    // Lit le registre d'état pour effacer le drapeau d'interruption
    TC0->TC_CHANNEL[0].TC_SR;

    // Démarre une nouvelle conversion ADC
    ADC->ADC_CR = ADC_CR_START;
}

void DACC_Handler() {
  DACC->DACC_ISR;  // Efface le registre d’état "status register"
}

void CircularBuffer() {
    int16_t tempoBuffer[BUFFER_SIZE];
    int adcBufferIndex = 0;
    for (int i = 0; i < OUTPUT_SIZE; i++) {
        while ((ADC->ADC_ISR & 0x80) == 0); // attente de la fin de la conversion
        tempoBuffer[adcBufferIndex] = ADC->ADC_CDR[7]; // sauvegarde du signal dans le buffer 

        adcBufferIndex++;   // incrementation de l'index du buffer circulaire
        uint16_t sumIndex = adcBufferIndex; // update de l'index pour le bon indexage des valeurs en fonction des coeffs 
        if (adcBufferIndex == BUFFER_SIZE) { // modulo pour remettre à 0 
            adcBufferIndex = 0;
        }

        // Calcul du filtrage sur les valeurs du buffer 
        int32_t acc = 0;
        for (int l = 0; l < BUFFER_SIZE; l++) { // multiplication des valeurs du buffer par les coeffs

            // gestion de l'index interne au buffer pour le calcul du filtre
            if (sumIndex > 0) {
                sumIndex--;
            } else {
                sumIndex = BUFFER_SIZE - 1;
            }
            // calcule du filtre sur 32 bit
            acc += filter_taps[l] * tempoBuffer[sumIndex];
        }
        filteredSignal[i] = acc >> 15; // shift de 15 bit pour repasser sur 16 bit
    }
}

void setup() {
    Serial.begin(9600);
    setupADC();
    setupDAC();
}

void loop() {
    static int bufferIndex = 0;
    while ((ADC->ADC_ISR & 0x80) == 0);
    inputSignal[bufferIndex] = ADC->ADC_CDR[7];

    // Affiche la valeur lue par l'ADC
    Serial.print("ADC: ");
    Serial.println(inputSignal[bufferIndex]);

    // Incrémente l'index du buffer
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Applique le filtre RIF
    CircularBuffer();

    // Affiche la valeur filtrée avant de l'envoyer au DAC
    Serial.print("-- DAC: ");
    Serial.println(filteredSignal[0]);

    // Écrit l'échantillon filtré au DAC
    DACC->DACC_CDR = DACC_CDR_DATA(filteredSignal[0]);
    while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
}