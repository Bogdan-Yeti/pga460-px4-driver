uint64 timestamp          # Время с момента запуска системы (микросекунды)

float32 distance_m        # Дистанция в метрах
float32 max_distance      # Максимальная дистанция по паспорту
float32 min_distance      # Рассчитывается исходя из DECPL_TIME

# Битовая маска ошибок (0 = OK)
uint8 status_flags

uint8 ERR_NO_ERROR       = 0
uint8 ERR_COMM_FAILURE   = 1    # Ошибка связи с чипом (timeout)
uint8 ERR_UART_BAUD_RATE_MISMATCH = 2 # Cлишком короткая или слишком длинная для текущих настроек чипа
uint8 ERR_UART_SYNC_STABILITY = 4 # Ширина последовательных битов в поле Sync не совпадает
uint8 ERR_UART_INVALID_MASTER_CHECKSUM = 8 # Ошибка отправленной контрольной суммы
uint8 ERR_UART_UNKNOWN_COMMAND = 16 # Ошибка команды
uint8 ERR_UART_FRAMING_FAILURE = 32 # Общая ошибка кадра
uint8 ERR_UART_INVALID_SLAVE_CHECKSUM = 64 # Ошибка контрольной суммы от устройства


