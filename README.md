# Sterownik wibratorów Arduino

Ten projekt zawiera szkic `Sterownik_wibratorow_ar.ino`, który umożliwia sterowanie triakiem za pomocą protokołu Modbus RTU. Logika wyzwalania triaka została wydzielona do biblioteki `TriacDriver`, co ułatwia ponowne wykorzystanie kodu. Biblioteka generuje dwa krótkie impulsy – drugi pojawia się domyślnie po 10&nbsp;ms – co zapewnia pewne zapalenie triaka. Układ przeznaczony jest do regulacji mocy urządzeń sieciowych poprzez włączanie triaka w odpowiednim momencie względem przejścia napięcia przez zero.

Od wersji bieżącej sterownik posiada funkcję łagodnego narastania mocy. Przy uruchomieniu oraz każdej zmianie nastawy mocy sygnał wyjściowy zwiększa lub zmniejsza się liniowo w zadanym czasie.

## Wymagania
- Płytka zgodna z Arduino (np. Arduino Uno)
- Biblioteki `EEPROM.h`, `ModbusRtu.h`, `TriacDriver`
- Zewnętrzny sterownik (np. MAX485) do komunikacji RS‑485 jeśli używamy magistrali Modbus

## Podłączenia
- **D8** – bramka triaka
- **A2** – detekcja przejścia przez zero (PCINT10)
- **D9** – przytrzymanie w stanie niskim przez \>=1 s przy uruchamianiu zeruje ustawienia
- **D2** – sygnał `TXEN` sterujący transceiverem RS‑485
- **A0 / PC0 (fizyczny pin 23)** – sterowanie przekaźnikiem (wyjście)

## Rejestry Modbus
Program udostępnia 16 rejestrów typu Holding (tablica `mbRegs`). Rejestry te można odczytywać funkcją Modbus FC03 (Read Holding Registers), a zapisywać funkcjami FC06 (Write Single Register) lub FC16 (Write Multiple Registers). Najważniejsze z nich:

| Adres | Opis | Dostęp |
|-------|------|--------|
| `0` | Adres urządzenia Modbus (1–247). Po poprawnym zapisie restartuje konfigurację komunikacji. | R/W (FC03, FC06/FC16) |
| `1` | Tryb ramek: 0 – 8N1, 1 – 8E1, 2 – 8O1. | R/W (FC03, FC06/FC16) |
| `2` | Prędkość transmisji. Przyjmuje wyłącznie wartości z listy: 2400, 4800, 9600, 19200, 38400, 57600, 115200. | R/W (FC03, FC06/FC16) |
| `3` | Moc docelowa (0–100 %, 0 wyłącza triak). | R/W (FC03, FC06/FC16) |
| `4` | Czas narastania mocy w ms (0–10000). | R/W (FC03, FC06/FC16) |
| `5` | Czas impulsu w µs (100–5000). | R/W (FC03, FC06/FC16) |
| `6` | Stan przekaźnika (0 – wyłączony, 1 – włączony). | R/W (FC03, FC06/FC16) |

Pozostałe rejestry (`7`–`15`) są obecnie niewykorzystywane.

Zmiany zapisane w rejestrach `0`–`4` są przechowywane w pamięci EEPROM i po ponownym uruchomieniu wczytywane jako ustawienia domyślne. Rejestry `5` i `6` wpływają wyłącznie na bieżące działanie urządzenia.

## Domyślne ustawienia komunikacji
- Adres urządzenia Modbus: `1`
- Format ramki: `8N1` (wartość `0` w rejestrze 1)
- Prędkość transmisji: `9600` bps

Wartości te można zmienić poprzez odpowiednie rejestry Modbus.

## Reset ustawień
Aby przywrócić wartości domyślne, należy podczas włączania zasilania przytrzymać pin `resetPin` (D9) w stanie niskim przez co najmniej 1 s.

## Kompilacja
Szkic można wgrać z poziomu Arduino IDE lub narzędzia `arduino-cli`:

```bash
arduino-cli compile --fqbn arduino:avr:uno
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno
```

Przy braku `arduino-cli` w środowisku należy użyć Arduino IDE.

