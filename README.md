# PGA460 PX4 Driver
Драйвер ультразвукового датчика PGA460 для автопилота PX4.

## Установка
1. Скопируйте папку `pga460` в `PX4-Autopilot/src/drivers`.
2. Включите драйвер в конфиге вашей платы (например, `boards/px4/sitl/default.px4board`):
   ```cmake
   CONFIG_DRIVERS_PGA460=y
3. Скопируйте `PgaData.msg` в `PX4-Autopilot/src/msg`
4. Добавьте новый msg файл в сборку `PX4-Autopilot/msg/CMakeLists.txt`
