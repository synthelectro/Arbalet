//============================================================================================
//
// ARPI
//
// Eric Penot 
// 27 juin 2017 
//
//============================================================================================
#include <FastLED.h>
//
#define TRUE              1
#define FALSE             0
#define ACTIV_LED         7  
#define ACTIV_LED_TEMPO   100
//
#define CMD_INIT          '0'
#define CMD_HELLO         'H'
#define CMD_LED_NUM       'L'
#define CMD_LED_PIN       'P'
#define CMD_TOUCH_TYPE    'T'
#define CMD_INIT_SUCCESS  'S'
#define CMD_INIT_FAILURE  'F'
#define CMD_DATA_FOLLOWS  'D'
#define CMD_BUFFER_READY  'B'
#define CMD_INIT_RETRY    5
#define PROTOCOL_VERSION  2
//
unsigned char CmdInitStatus = CMD_INIT_FAILURE;
unsigned char CmdInitRequest = CMD_INIT_RETRY;
unsigned int  NumLed    = 150;
unsigned char NumPin    = 0;
#define       DATA_PIN  20
unsigned char TchTyp    = 255;
CRGB          *Leds     = NULL;
//
//void InitStripLib(void);
//
void setup() {
  // Arrêt des interruptions
  cli();

  // Configuration des I/O
  pinMode(ACTIV_LED, OUTPUT);
  DDRE |= (1 << PINE0); // PE0 en sortie (sélection MOSI Arduino)
  DDRE |= (1 << PINE1); // PE1 en sortie (sélection MOSI Pi)
  DDRE |= (1 << PINE3); // PE3 en sortie (MOSI Arduino) 
  DDRD |= (1 << PIND5); // PD5 en sortie (sélection GPIO18 Pi)
  // Par défaut, MOSI Arduino sélectionné
  PORTE &= ~(1 << PINE0);
  PORTE |= (1 << PINE1); 
  PORTE &= ~(1 << PINE3); 
  PORTD |= (1 << PIND5);
  
  // Configuration Timer 1 pour IRQ à ~ 100Hz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 160;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);    // prescaler 1024
  TIMSK1 |= (1 << OCIE1A);  

  // Démarrage interruptions
  sei();

  // Ouverture port série avec FT230
  Serial.begin(115200);
  Serial.println("Demarrage de l'application...\n\r");

  // Ouverture port série avec Pi
  Serial1.begin(115200);
  Serial1.setTimeout(5000);

  // Initialisation FastLed pour animation
  InitStripLib();
}
//
//============================================================================================
void loop() {

  SerialEvent1();
  //SerialEvent();

  // Si la liaison avec la Pi n'est pas effectuée
  if (CmdInitStatus == CMD_INIT_FAILURE) {
    if (!CmdInitRequest) {
      Serial1.write(CMD_HELLO);
      CmdInitRequest = CMD_INIT_RETRY; }
    Animation(); }
  // Sinon, récupération des valeurs de LEDS
  else read_buffer();  
}
//============================================================================================
//
// Données en provenance de la Pi
//
//============================================================================================
void SerialEvent1() {

  static unsigned char CmdStatus = CMD_INIT;
  unsigned char IncomingByte;

  if(!Serial1.available()) return;
  // Gestion des données reçues
  IncomingByte = Serial1.read();
  // Action en fonction du déroulé de la procédure d'INIT avec la Pi
  switch (CmdStatus) {
    case CMD_INIT : 
      if (IncomingByte == CMD_HELLO) {
        Serial.println("INIT-HELLO\n\r");
        Serial1.write(PROTOCOL_VERSION);
        CmdStatus = CMD_LED_NUM; }
      break;
    case PROTOCOL_VERSION :   
      Serial.println("INIT-1\n\r");
      Serial1.write(PROTOCOL_VERSION);
      CmdStatus = CMD_LED_NUM;
      break;
    case CMD_LED_NUM:
      Serial.println("INIT-2\n\r");
      NumLed = IncomingByte;
      CmdStatus = CMD_LED_PIN;
      break;
    case CMD_LED_PIN:
      Serial.println("INIT-3\n\r");
      NumPin = IncomingByte;
      CmdStatus = CMD_TOUCH_TYPE;
      break;
    case CMD_TOUCH_TYPE:
      Serial.println("INIT-4\n\r");
      TchTyp = IncomingByte;
      Serial1.write(CMD_INIT_SUCCESS);
      CmdInitStatus = CMD_INIT_SUCCESS;
      CmdStatus = CMD_INIT;
      break;
      // Test de paramètres reçus
      if (!NumLed || !NumPin || (TchTyp == 255)) {
        CmdInitStatus = CMD_INIT_FAILURE;
        Serial1.write(CMD_INIT_FAILURE); }
      else { 
        CmdInitStatus = CMD_INIT_SUCCESS;
        Serial1.write(CMD_INIT_SUCCESS); }
      CmdStatus = CMD_INIT;
      break;
    default :
      Serial.println("INIT-Default\n\r");
      CmdStatus = CMD_INIT;
      break; }
}
//============================================================================================
void SerialEvent() {

  unsigned char IncomingByte;

  if(Serial.available()) {
    // Récupération octet reçu sur port n°1
    IncomingByte = Serial.read();
    // Et envoi sur port n°0
    Serial.write(IncomingByte); }
}
//============================================================================================
void InitStripLib(void){

  // A n'effectuer qu'une seule fois!
  if (Leds == NULL) {
    // Réservation mémoire pour les LEDS
    Leds = (CRGB *) malloc(sizeof(CRGB) * NumLed);
    // Attention : DATA_PIN n'est pas une variable
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(Leds, NumLed);
      Serial.println("INIT FASTLED SUCCESS\n\r"); }
 }
//============================================================================================
boolean read_buffer(void) {

unsigned short LedIndex;
unsigned short RgbAmount;
unsigned char  rgb[3];

  Serial.println("ReadBuffer...\n\r");
  // Demande de lecture buffer Pi
  Serial1.write(CMD_BUFFER_READY);
  // Boucle de lecture des infos sur LED
  for(LedIndex = 0; LedIndex < NumLed; ++LedIndex) {
    RgbAmount = Serial1.readBytes(rgb, 3);
    // Si les trois couleurs non reçues
    if (RgbAmount != 3) {
      CmdInitStatus = CMD_INIT_FAILURE;
      return false; }
    // Les trois couleurs ont été reçues
    else Leds[LedIndex] = CRGB(rgb[0], rgb[1], rgb[2]);
  }
  // En fin de boucle, mise à jour des LEDs
  FastLED.show();
  return true;
}
//============================================================================================
// IRQ 100Hz
//============================================================================================
ISR(TIMER1_COMPA_vect){

static unsigned char ActivLedTempo = ACTIV_LED_TEMPO;

  if (!--ActivLedTempo){ 
    digitalWrite(ACTIV_LED, !digitalRead(ACTIV_LED)); 
    ActivLedTempo = ACTIV_LED_TEMPO; 
    // Décomptage de tempos 
    CmdInitRequest--; }
}  
//============================================================================================
void Animation(void){

static unsigned int Index = 0;

  Leds[Index++] = CRGB::Black; 
  Leds[Index+4] = CRGB::White; 
  if (Index == 146) {
    Leds[Index] = CRGB::Black; 
    Leds[Index+1] = CRGB::Black; 
    Leds[Index+2] = CRGB::Black; 
    Leds[Index+3] = CRGB::Black; 
    Leds[Index+4] = CRGB::Black; 
    Index = 0;
    Leds[Index] = CRGB::White; 
    Leds[Index+1] = CRGB::White; 
    Leds[Index+2] = CRGB::White; 
    Leds[Index+3] = CRGB::White; 
    Leds[Index+4] = CRGB::White; }
  FastLED.show();
}
//============================================================================================

