#include <Arduino.h>

#define NUMTAPS 101  // Nombre de coefficients du filtre FIR
#define BUFFER_SIZE 8000  // Taille du buffer pour 1 seconde d'audio à 8 kHz

// Coefficients du filtre FIR (générés par le script Python)
float coefficients[NUMTAPS] = {
  // Coefficients du filtre FIR ici
};

// Buffers pour l'enregistrement et le traitement du signal
float inputBuffer[BUFFER_SIZE];
float outputBuffer[BUFFER_SIZE];

void setupADC() {
    // Configuration de l'ADC pour utiliser la broche A0
    PMC->PMC_PCER1 |= PMC_PCER1_PID37; // Active le périphérique ADC
    ADC->ADC_MR = ADC_MR_PRESCAL(0) // Définit le diviseur de fréquence à 255
                | ADC_MR_STARTUP_SUT64 // Définit le temps de démarrage à 64 périodes d'ADC_CLK
                | ADC_MR_TRACKTIM(15) // Définit le temps de suivi à 15 périodes d'ADC_CLK
                | ADC_MR_SETTLING_AST3;// Définit le temps de stabilisation à 17 périodes d'ADC_CLK
    ADC->ADC_CHER = 0x01; // Active le canal 0 (A0) (correspond à la broche A0)
    
    // Configuration du Timer Counter 0 Channel 0 (TC0) pour la fréquence d'échantillonnage
    PMC->PMC_PCER0 |= PMC_PCER0_PID27; // Active le périphérique TC0
    TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
    // Définit la source d'horloge à TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
    // Active le déclenchement de comparaison RC
    // Définit la valeur RC pour une fréquence samplingFrequency Hz
    TC0->TC_CHANNEL[0].TC_RC = 82; // 8kHz -> ( MCK / 128 / 8000 ) -> 82
    // Active l'interruption de comparaison RC
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    // Active l'interruption TC0_IRQn dans le NVIC
    NVIC_EnableIRQ(TC0_IRQn);
    // Active le compteur
    // Active le déclenchement logiciel
    TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void TC0_Handler() {
    // Lit le registre d'état pour effacer le drapeau d'interruption
    TC0->TC_CHANNEL[0].TC_SR;
    // Démarre une nouvelle conversion ADC
    ADC->ADC_CR = ADC_CR_START;
}

void setupDAC() {
    PMC->PMC_PCER1 |= PMC_PCER1_PID38; // Active le périphérique DAC
    // Configuration du DAC en mode normal
    DACC->DACC_MR = DACC_MR_TRGEN_DIS // Désactive le déclencheur externe
                    | DACC_MR_USER_SEL_CHANNEL1 // select canal 1
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

void DACC_Handler() {
    DACC->DACC_ISR;  // Effacer le register d’état “status register”
}

void setup() {
    Serial.begin(115200);
    setupADC();
    setupDAC();
}

void loop() {
    // Lecture de 1 seconde d'audio à partir du microphone MAX9814 sur A0
    for (int i = 0; i < BUFFER_SIZE; i++) {
        while ((ADC->ADC_ISR & 0x01) == 0); // Attend la fin de la conversion
        inputBuffer[i] = ADC->ADC_CDR[0] / 1023.0;  // Normalise la lecture analogique
    }
    
    // Application du filtre FIR
    for (int i = 0; i < BUFFER_SIZE; i++) {
        outputBuffer[i] = 0;
        for (int j = 0; j < NUMTAPS; j++) {
            if (i - j >= 0) {
                outputBuffer[i] += coefficients[j] * inputBuffer[i - j];
            }
        }
    }

    // Transmission des signaux d'entrée et de sortie via le port série pour vérification
    Serial.println("Original,Filtered");
    for (int i = 0; i < BUFFER_SIZE; i++) {
        Serial.print(inputBuffer[i], 6);  // Envoyer avec une précision de 6 décimales
        Serial.print(",");
        Serial.println(outputBuffer[i], 6);  // Envoyer avec une précision de 6 décimales
    }
    
    // Pause pour éviter des enregistrements répétés accidentels
    delay(1000);
}
