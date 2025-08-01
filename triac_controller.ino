#include <EEPROM.h>      // biblioteka do obslugi pamieci nieulotnej
#include <ModbusRtu.h>   // implementacja protokolu Modbus RTU
#include <avr/interrupt.h> // definicje obslugi przerwan AVR
#include "TriacDriver.h"  // obsluga impulsow triaka w osobnej bibliotece

#define TXEN 2                 // pin sterujacy trybem nadawania RS485
const int triacPin = 8;        // wyjscie sterujace bramka triaka
const int zeroCrossPin = A2;   // wejscie detekcji przejscia przez zero
const int resetPin = 9;        // pin resetu ustawien (przycisk)

uint16_t mbRegs[16] = {0};     // tablica rejestrow Modbus

TriacDriver triac(triacPin);   // obiekt obslugi triaka

#define DEFAULT_MB_ADDR 1      // domyslny adres urzadzenia Modbus
#define DEFAULT_FRAME   0      // domyslny typ ramki Modbus
#define DEFAULT_BAUD    9600   // domyslna predkosc komunikacji
#define DEFAULT_POWER   100    // domyslna moc (procent)
#define DEFAULT_PULSE_US 150   // domyslna dlugosc impulsu w microsekundach
#define MIN_PULSE_US 100       // minimalna dozwolona dlugosc impulsu
#define MAX_PULSE_US 5000      // maksymalna dozwolona dlugosc impulsu

volatile uint8_t mb_power = DEFAULT_POWER; // aktualna moc sterowania
static uint16_t pulseDuration = DEFAULT_PULSE_US; // aktualny czas impulsu

volatile int last_CH1_state = 0;        // poprzedni stan wejscia zerowego

uint8_t mb_addr;     // adres urzadzenia
uint8_t mb_frame;    // typ ramki Modbus
unsigned long mb_baud; // predkosc transmisji

const unsigned long validBaudRates[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200}; // lista dozwolonych predkosci
const uint8_t numBaudRates = sizeof(validBaudRates)/sizeof(validBaudRates[0]);         // liczba elementow listy

Modbus slave(DEFAULT_MB_ADDR, Serial, TXEN); // obiekt klasy Modbus pracujacy jako slave

void resetToDefaults() {                      // przywraca wartosci domyslne
  mb_addr = DEFAULT_MB_ADDR;                  // domyslny adres
  mb_frame = DEFAULT_FRAME;                   // domyslny typ ramki
  mb_baud = DEFAULT_BAUD;                     // domyslna predkosc
  mb_power = DEFAULT_POWER;                   // domyslna moc
  EEPROM.update(0, mb_addr);                  // zapis do EEPROM
  EEPROM.update(1, mb_frame);                 // zapis typu ramki
  EEPROM.put(2, mb_baud);                     // zapis predkosci
  EEPROM.update(6, mb_power);                 // zapis mocy
}

void loadSettings() {                          // wczytuje ustawienia z EEPROM
  mb_addr = EEPROM.read(0);                    // odczyt adresu
  mb_frame = EEPROM.read(1);                   // odczyt typu ramki
  EEPROM.get(2, mb_baud);                      // odczyt predkosci
  mb_power = EEPROM.read(6);                   // odczyt mocy
  if (mb_addr < 1 || mb_addr > 247) mb_addr = DEFAULT_MB_ADDR; // walidacja adresu
  if (mb_frame > 2) mb_frame = DEFAULT_FRAME;  // walidacja ramki
  bool foundBaud = false;                      // sprawdzanie poprawnej predkosci
  for (uint8_t i = 0; i < numBaudRates; i++)   // iteracja po dozwolonych predkosciach
    if (mb_baud == validBaudRates[i]) foundBaud = true; // czy znaleziono
  if (!foundBaud) mb_baud = DEFAULT_BAUD;      // jesli nie, ustaw domyslna
  if (mb_power > 100) mb_power = DEFAULT_POWER; // zakres mocy 0..100
}

void setup() {                               // funkcja inicjujaca
  triac.begin();                             // konfiguracja pinu triaka
  pinMode(zeroCrossPin, INPUT_PULLUP);       // wejscie z podciagnieciem
  pinMode(resetPin, INPUT_PULLUP);           // przycisk resetu

  if (digitalRead(resetPin) == LOW) {        // sprawdzenie czy wcisniety reset
    unsigned long t0 = millis();             // zapamietaj czas poczatkowy
    while (digitalRead(resetPin) == LOW && millis() - t0 < 2000); // czekaj na puszczenie
    if (millis() - t0 >= 1000) {             // jesli przytrzymany >1s
      resetToDefaults();                     // przywroc ustawienia fabryczne
    }
  }

  loadSettings();                            // wczytaj ustawienia zapisane w EEPROM

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
  mbRegs[3] = mb_power;                      // wpisz moc
  mbRegs[5] = DEFAULT_PULSE_US;              // wpisz dlugosc impulsu
}

// Funkcja uruchamia Timer1 na 10 ms (półokres 50 Hz)
void startSecondImpulseTimer() {
  // 16 MHz / 8 (preskaler) = 2 000 000 Hz, 1 tick = 0.5 us
  // 10 ms = 20 000 ticków
  TCNT1 = 0;                              // wyzeruj licznik
  OCR1A = 20000;  // 10ms                  // ustal wartosc porownania
  TCCR1B = (1 << WGM12) | (1 << CS11);    // tryb CTC, preskaler 8
  TIMSK1 |= (1 << OCIE1A);                // wlacz przerwanie compare match
}

// ISR Timer1: po 10 ms wyzwala drugi impuls
ISR(TIMER1_COMPA_vect) {
  triac.trigger(mb_power, pulseDuration);    // drugi impuls triaka
  TIMSK1 &= ~(1 << OCIE1A);                  // wylacz przerwanie compare
  TCCR1B = 0;                                // zatrzymaj timer
}

ISR(PCINT1_vect) {                            // obsluga zmiany stanu na PC2
  if (PINC & (1 << PC2)) {                    // wykryto zbocze narastajace
    if (last_CH1_state == 0) {                // jesli poprzednio bylo niskie
      // Hardware zero cross
      triac.trigger(mb_power, pulseDuration); // pierwszy impuls natychmiast
      startSecondImpulseTimer();              // zlecenie drugiego impulsu
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
      mbRegs[2] = (uint16_t)mb_baud;          // przywroc poprzednia
    }
  } else {
    mbRegs[2] = (uint16_t)mb_baud;            // przekaz aktualna predkosc
  }
  if (mbRegs[3] != mb_power && mbRegs[3] <= 100) { // zmiana mocy
    mb_power = mbRegs[3];                 // zapisz nowa wartosc
    EEPROM.update(6, mb_power);           // aktualizacja EEPROM
  } else {
    mbRegs[3] = mb_power;                 // zwroc aktualna moc
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
}

