# PGA460 PX4 Driver
Драйвер ультразвукового датчика PGA460 для автопилота PX4.

## Установка
1. Скопируйте папку с драйвером в `PX4-Autopilot/src/drivers/pga460`.
2. Включите драйвер в конфиге вашей платы (например, `boards/px4/sitl/default.px4board`):
   ```cmake
   CONFIG_DRIVERS_PGA460=y
