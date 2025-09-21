#include <EEPROM.h>      // biblioteka do obslugi pamieci nieulotnej
#include <ModbusRtu.h>   // implementacja protokolu Modbus RTU
#include <avr/interrupt.h> // definicje obslugi przerwan AVR (uzywane w bibliotece)
#include "TriacDriver.h"  // obsluga impulsow triaka w osobnej bibliotece

#define TXEN 2                 // pin sterujacy trybem nadawania RS485
const int triacPin = 8;        // wyjscie sterujace bramka triaka
const int zeroCrossPin = A2;   // wejscie detekcji przejscia przez zero
const int resetPin = 9;        // pin resetu ustawien (przycisk)
const int relayPin = A0;       // PC0 (physical pin 23 on Nano)

uint16_t mbRegs[16] = {0};     // tablica rejestrow Modbus

TriacDriver triac(triacPin);   // obiekt obslugi triaka

#define DEFAULT_MB_ADDR 1      // domyslny adres urzadzenia Modbus
#define DEFAULT_FRAME   0      // domyslny typ ramki Modbus
#define DEFAULT_BAUD    9600   // domyslna predkosc komunikacji
#define DEFAULT_POWER   100    // domyslna moc (procent)
#define DEFAULT_PULSE_US 150   // domyslna dlugosc impulsu w microsekundach
#define MIN_PULSE_US 100       // minimalna dozwolona dlugosc impulsu
#define MAX_PULSE_US 5000      // maksymalna dozwolona dlugosc impulsu
#define DEFAULT_SOFTSTART_MS 1000 // domyslny czas narastania mocy w ms
#define MIN_SOFTSTART_MS 0        // minimalny czas narastania
#define MAX_SOFTSTART_MS 10000    // maksymalny czas narastania

volatile uint8_t currentPower = 0;      // aktualna moc sterowania
uint8_t mb_power = DEFAULT_POWER;       // docelowa moc ustawiana przez Modbus
static uint16_t pulseDuration = DEFAULT_PULSE_US; // aktualny czas impulsu
static uint16_t softStartTime = DEFAULT_SOFTSTART_MS; // czas narastania mocy
static uint8_t startPower = 0;          // moc poczatkowa przy narastaniu
static bool softStartActive = false;    // flaga aktywnego narastania
static uint16_t softStartSteps = 1;     // liczba cykli narastania
static uint16_t softStepCount = 0;      // aktualny licznik cykli
static int16_t softDelta = 0;           // roznica mocy do pokonania

volatile int last_CH1_state = 0;        // poprzedni stan wejscia zerowego

uint8_t mb_addr;     // adres urzadzenia
uint8_t mb_frame;    // typ ramki Modbus
unsigned long mb_baud; // predkosc transmisji
uint16_t relayState = 0;                // stan przekaznika

const unsigned long validBaudRates[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200}; // lista dozwolonych predkosci
const uint8_t numBaudRates = sizeof(validBaudRates)/sizeof(validBaudRates[0]);         // liczba elementow listy

Modbus slave(DEFAULT_MB_ADDR, Serial, TXEN); // obiekt klasy Modbus pracujacy jako slave

void startSoftChange(uint8_t newTarget) {
  startPower = currentPower;                 // zapamietaj moc poczatkowa
  mb_power = newTarget;                      // ustaw wartosc docelowa
  if (softStartTime == 0 || mb_power == startPower) { // brak narastania
    currentPower = mb_power;
    softStartActive = false;
  } else {
    softStartSteps = softStartTime / 20;     // liczba cykli przy 50 Hz (20 ms)
    if (softStartSteps == 0) softStartSteps = 1; // zabezpieczenie
    softStepCount = 0;
    softDelta = (int16_t)mb_power - startPower; // docelowa zmiana
    softStartActive = true;                   // aktywuj narastanie
  }
}

void updateSoftChange() {
  if (softStartActive) {
    if (softStepCount >= softStartSteps) {    // zakoncz narastanie
      currentPower = mb_power;
      softStartActive = false;
    } else {
      softStepCount++;                        // kolejny cykl
      currentPower = startPower + ((int32_t)softDelta * softStepCount) / softStartSteps;
    }
  } else {
    currentPower = mb_power;                 // brak narastania
  }
}

void resetToDefaults() {                      // przywraca wartosci domyslne
  mb_addr = DEFAULT_MB_ADDR;                  // domyslny adres
  mb_frame = DEFAULT_FRAME;                   // domyslny typ ramki
  mb_baud = DEFAULT_BAUD;                     // domyslna predkosc
  mb_power = DEFAULT_POWER;                   // domyslna moc docelowa
  currentPower = 0;                           // ustaw aktualna moc na 0
  softStartTime = DEFAULT_SOFTSTART_MS;       // domyslny czas narastania
  softStartActive = false;                    // brak narastania
  softStepCount = 0;                          // wyzeruj licznik
  softDelta = 0;
  EEPROM.update(0, mb_addr);                  // zapis do EEPROM
  EEPROM.update(1, mb_frame);                 // zapis typu ramki
  EEPROM.put(2, mb_baud);                     // zapis predkosci
  EEPROM.update(6, mb_power);                 // zapis mocy docelowej
  EEPROM.put(7, softStartTime);               // zapis czasu narastania
}

void loadSettings() {                          // wczytuje ustawienia z EEPROM
  mb_addr = EEPROM.read(0);                    // odczyt adresu
  mb_frame = EEPROM.read(1);                   // odczyt typu ramki
  EEPROM.get(2, mb_baud);                      // odczyt predkosci
  mb_power = EEPROM.read(6);                   // odczyt docelowej mocy
  EEPROM.get(7, softStartTime);                // odczyt czasu narastania
  if (mb_addr < 1 || mb_addr > 247) mb_addr = DEFAULT_MB_ADDR; // walidacja adresu
  if (mb_frame > 2) mb_frame = DEFAULT_FRAME;  // walidacja ramki
  bool foundBaud = false;                      // sprawdzanie poprawnej predkosci
  for (uint8_t i = 0; i < numBaudRates; i++)   // iteracja po dozwolonych predkosciach
    if (mb_baud == validBaudRates[i]) foundBaud = true; // czy znaleziono
  if (!foundBaud) mb_baud = DEFAULT_BAUD;      // jesli nie, ustaw domyslna
  if (mb_power > 100) mb_power = DEFAULT_POWER; // zakres mocy 0..100
  if (softStartTime < MIN_SOFTSTART_MS || softStartTime > MAX_SOFTSTART_MS)
    softStartTime = DEFAULT_SOFTSTART_MS;      // zakres czasu narastania
}

void setup() {                               // funkcja inicjujaca
  triac.begin();                             // konfiguracja pinu triaka
  pinMode(zeroCrossPin, INPUT_PULLUP);       // wejscie z podciagnieciem
  pinMode(resetPin, INPUT_PULLUP);           // przycisk resetu
  pinMode(relayPin, OUTPUT);                 // konfiguracja przekaznika
  digitalWrite(relayPin, LOW);               // domyslnie wylaczony

  if (digitalRead(resetPin) == LOW) {        // sprawdzenie czy wcisniety reset
    unsigned long t0 = millis();             // zapamietaj czas poczatkowy
    while (digitalRead(resetPin) == LOW && millis() - t0 < 2000); // czekaj na puszczenie
    if (millis() - t0 >= 1000) {             // jesli przytrzymany >1s
      resetToDefaults();                     // przywroc ustawienia fabryczne
    }
  }

  loadSettings();                            // wczytaj ustawienia zapisane w EEPROM

  startSoftChange(mb_power);                 // lagodny start od 0 do docelowej mocy

  Serial.begin(mb_baud);                     // start portu szeregowego
  slave = Modbus(mb_addr, Serial, TXEN);     // inicjacja protokolu Modbus
  slave.start();                             // uruchom slave

  // Przerwanie pin change na PC2 (A2)
  PCICR |= (1 << PCIE1);                     // wlacz przerwania grupy PCIE1
  PCMSK1 |= (1 << PCINT10);                  // maskuj przerwanie PC2

  // Konfiguracja Timer1 do trybu CTC (wyzerowany, czeka na zlecenie)
  TCCR1A = 0;                                // rejestr sterujacy A
  TCCR1B = 0;                                // rejestr sterujacy B
  TIMSK1 = 0;                                // maska przerwan

  mbRegs[0] = mb_addr;                       // wpisz aktualny adres do rejestru
  mbRegs[1] = mb_frame;                      // wpisz typ ramki
  mbRegs[2] = (uint16_t)mb_baud;             // wpisz predkosc
  mbRegs[3] = mb_power;                      // wpisz moc docelowa
  mbRegs[4] = softStartTime;                 // wpisz czas narastania
  mbRegs[5] = pulseDuration;                 // wpisz dlugosc impulsu
  mbRegs[6] = relayState;                    // stan przekaznika
}


ISR(PCINT1_vect) {                            // obsluga zmiany stanu na PC2
  if (PINC & (1 << PC2)) {                    // wykryto zbocze narastajace
    if (last_CH1_state == 0) {                // jesli poprzednio bylo niskie
      updateSoftChange();                     // zaktualizuj moc przed impulsem
      // Hardware zero cross
      // pierwszy impuls oraz zaplanowany drugi po 10 ms
      triac.triggerWithSecond(currentPower, pulseDuration, 10);
    }
    last_CH1_state = 1;                       // zapamietaj stan wysoki
  } else {
    if (last_CH1_state == 1) {                // zbocze opadajace
      last_CH1_state = 0;                     // zapamietaj stan niski
    }
  }
}

void loop() {                                 // glowna petla programu
  slave.poll(mbRegs, 16);                     // obsluga protokolu Modbus

  if (mbRegs[0] != mb_addr && mbRegs[0] >= 1 && mbRegs[0] <= 247) { // zmiana adresu
    mb_addr = mbRegs[0];                       // zapisz nowy adres
    EEPROM.update(0, mb_addr);                 // zapisz w EEPROM
    slave = Modbus(mb_addr, Serial, TXEN);     // zainicjuj od nowa
    slave.start();                             // start protokolu
  } else {
    mbRegs[0] = mb_addr;                       // odzwierciedl aktualny adres
  }
  if (mbRegs[1] != mb_frame && mbRegs[1] <= 2) { // zmiana typu ramki
    mb_frame = mbRegs[1];                   // zapisz nowa wartosc
    EEPROM.update(1, mb_frame);             // aktualizacja EEPROM
  } else {
    mbRegs[1] = mb_frame;                   // potwierdz aktualna wartosc
  }
  if (mbRegs[2] != (uint16_t)mb_baud) {       // czy zmieniono predkosc?
    bool valid = false;                       // flaga poprawnosci
    for (uint8_t i = 0; i < numBaudRates; i++) { // sprawdz dozwolone wartosci
      if (mbRegs[2] == validBaudRates[i]) {   // znaleziono zgodna
        mb_baud = validBaudRates[i];          // ustaw nowa predkosc
        EEPROM.put(2, mb_baud);               // zapisz w EEPROM
        Serial.end();                         // zamknij port szeregowy
        Serial.begin(mb_baud);                // uruchom z nowa predkoscia
        slave = Modbus(mb_addr, Serial, TXEN); // restart obiektu Modbus
        slave.start();                        // start protokolu
        valid = true;                         // znacznik poprawnosci
        break;                                // wyjdz z petli
      }
    }
    if (!valid) {                             // jesli wartosc niedozwolona
      mbRegs[2] = static_cast<uint16_t>(mb_baud);  // przywroc poprzednia
    }
  } else {
    mbRegs[2] = static_cast<uint16_t>(mb_baud); // przekaz aktualna predkosc
  }
  if (mbRegs[3] != mb_power && mbRegs[3] <= 100) { // zmiana mocy
    startSoftChange(mbRegs[3]);            // rozpoczynij lagodna zmiane mocy
    EEPROM.update(6, mb_power);            // zapis docelowej mocy
  } else {
    mbRegs[3] = mb_power;                  // zwroc docelowa moc
  }
  if (mbRegs[4] != softStartTime) {        // zmiana czasu narastania
    if (mbRegs[4] >= MIN_SOFTSTART_MS && mbRegs[4] <= MAX_SOFTSTART_MS) {
      softStartTime = mbRegs[4];           // zapisz nowa wartosc
      EEPROM.put(7, softStartTime);        // zapisz w EEPROM
    } else {
      mbRegs[4] = softStartTime;           // przywroc poprzednia
    }
  } else {
    mbRegs[4] = softStartTime;             // potwierdz aktualny czas
  }
  if (mbRegs[5] != pulseDuration) {        // zmiana dlugosci impulsu
    if (mbRegs[5] >= MIN_PULSE_US && mbRegs[5] <= MAX_PULSE_US) { // w zakresie
      pulseDuration = mbRegs[5];            // zapisz nowa wartosc
    } else {
      mbRegs[5] = pulseDuration;           // poza zakresem - przywroc
    }
  } else {
    mbRegs[5] = pulseDuration;             // potwierdz aktualna dlugosc
  }

  if (mbRegs[6] != relayState) {           // zmiana stanu przekaznika
    relayState = mbRegs[6] ? 1 : 0;         // ogranicz do 0 lub 1
    digitalWrite(relayPin, relayState ? HIGH : LOW); // ustaw pin
    mbRegs[6] = relayState;                 // potwierdz zapisany stan
  } else {
    mbRegs[6] = relayState;                // odzwierciedl aktualny stan
  }

}

