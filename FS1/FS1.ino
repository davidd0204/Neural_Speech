// Définition de la fréquence du processeur
#define F_CPU 84000000UL
// Définition de la fréquence d'interruption
#define TC_FREQUENCY 1000
// Définition de la valeur de pré-division
#define TC_PRESCALER TC_CMR_TCCLKS_TIMER_CLOCK1
// Définition du nombre de ticks par interruption
#define TC_TICKS (F_CPU / TC_FREQUENCY)
// Définition de la fonction d'interruption
void TC0_Handler(void) {
  // Réinitialisation du registre STATUS pour indiquer la fin de l'interruption
  TC_GetStatus(TC0, 0);
  // Code à exécuter lors de l'interruption
}

void ADC_setup() {
  // Activation de l'horloge pour le périphérique TC0
  pmc_enable_periph_clk(ID_TC0);
  // Configuration du registre TC0
  TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_PRESCALER);
  // Configuration du registre RC pour définir le nombre de ticks par interruption
  TC_SetRC(TC0, 0, TC_TICKS);
  // Activation de l'interruption pour le périphérique TC0
  NVIC_EnableIRQ(TC0_IRQn);
  // Activation de la génération d'interruption sur le registre RC
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Activation du Timer
  TC_Start(TC0, 0);
}



void setup() {
  // put your setup code here, to run once:
  ADC_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
}
