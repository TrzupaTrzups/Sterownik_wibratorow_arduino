# Sterownik wibratorów Arduino

Ten projekt zawiera szkic `triac_controller.ino`, który umożliwia sterowanie triakiem za pomocą protokołu Modbus RTU. Logika wyzwalania triaka została wydzielona do biblioteki `TriacDriver`, co ułatwia ponowne wykorzystanie kodu. Biblioteka generuje dwa krótkie impulsy – drugi pojawia się domyślnie po 10&nbsp;ms – co zapewnia pewne zapalenie triaka. Układ przeznaczony jest do regulacji mocy urządzeń sieciowych poprzez włączanie triaka w odpowiednim momencie względem przejścia napięcia przez zero.

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
- **PE3** – sterowanie przekaźnikiem (wyjście)

## Rejestry Modbus
Program udostępnia 16 rejestrów (tablica `mbRegs`). Najważniejsze z nich:

| Adres | Opis |
|-------|------|
| `0` | Adres urządzenia Modbus (1–247) |
| `1` | Tryb ramek: 0 – 8N1, 1 – 8E1, 2 – 8O1 |
| `2` | Prędkość transmisji (2400–115200) |
| `3` | Moc docelowa (0–100 %, 0 wyłącza triak) |
| `4` | Czas narastania mocy w ms (0–10000) |
| `5` | Czas impulsu w µs (100–5000) |
| `6` | Stan przekaźnika (0 – wyłączony, 1 – włączony) |

Zmiany wartości w tych rejestrach są zapisywane w pamięci EEPROM i przy kolejnym uruchomieniu odczytywane jako ustawienia domyślne.

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

