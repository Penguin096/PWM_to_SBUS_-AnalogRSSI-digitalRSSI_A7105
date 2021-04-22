# PWM_to_SBUS_-AnalogRSSI-digitalRSSI_A7105
## Конвертер PWM в SBUS 
Оптимизировнанный код
* Самый компактный скетч из существующих вариантов (2294 bytes/270 bytes)
* Самые быстрые вычисления среди существующих вариантов

* * Used Arduino IDE to program this firmware onto the Arduino chip.
### Входы PWM: D2, D3, D4, D5, D6, D7, D8, D9 

## UPD Добавленна возможность преобразования любого аналогового RSSI/цифроаналогового RSSI(А7105-приемники HK-T6A,HK-TR6A и т.п.) в канал SBUS.

* Результат слияния двух проектов https://github.com/Penguin096/PWM_to_SBUS_-AnalogRSSI-digitalRSSI_A7105/tree/main/src/old_without_RSSI  ++  https://github.com/Penguin096/RSSI-A7105

* Для переключения режимов строка: //#define AnalogRSSI    // закомментировать для цифроаналогового входа RSSI для A7105

* Программа для ПК для проверки SBUS шины https://github.com/Penguin096/PWM_to_SBUS_GUI
* ![image](https://user-images.githubusercontent.com/65414023/115794990-b848b180-a3d7-11eb-9e91-6f6923723f1d.png)

