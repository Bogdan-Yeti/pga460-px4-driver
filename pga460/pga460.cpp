#include "pga460.h"

using namespace time_literals;

extern "C" __EXPORT int pga460_main(int argc, char *argv[]);

static const pga460_config_t pga460_config[] = {
    // --- Драйвер и Частота ---
    {0x14, 0x32}, // FREQ: 40kHz (при 16MHz тактовой)
    {0x15, 0x08}, // P1_PULSE_COUNT: 8 импульсов
    {0x10, 0x40}, // CURR_LIM_P1: ограничение тока
    {0x1B, 0x0F}, // DECPL_TIME: время декупажа (уменьшает слепую зону)

    // --- Time Varied Gain (Усиление) ---
    {0x1D, 0x11}, // TVG_RANGE: диапазон усиления
    {0x1E, 0x88}, // TVG_GAIN_0
    {0x1F, 0xAA}, // TVG_GAIN_1
    {0x20, 0xCC}, // TVG_GAIN_2
    {0x21, 0xFF}, // TVG_GAIN_3 (максимум на дистанции)

    // --- Thresholds (Пороги для пресета P1) ---
    {0x40, 0xEE}, {0x41, 0xEE}, {0x42, 0xDD}, // Высокие пороги в начале (0-50см)
    {0x43, 0xBB}, {0x44, 0x88}, {0x45, 0x77}, // Снижение (50см - 2м)
    {0x46, 0x66}, {0x47, 0x55}, {0x48, 0x44}  // Низкие пороги для дали (2м+)
};

PGA460::PGA460(const char *device) :
    ModuleBase(),
    ScheduledWorkItem("PGA460", px4::wq_configurations::lp_default)
{
    strncpy(_device, device, sizeof(_device) - 1);
    _device[sizeof(_device) - 1] = '\0';

    if (open_uart(_device) >= 0) {
        _initialized = true;
    }
}

int PGA460::task_spawn(int argc, char *argv[])
{
	const char *device = "/dev/ttyS1";

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		if (ch == 'd') device = myoptarg;
	}

	PGA460 *instance = new PGA460(device);
	if (!instance || !instance->_initialized) {
		delete instance;
		return PX4_ERROR;
	}

	if (!instance->init_hw()) {
		PX4_ERR("Hardware init failed, aborting");
		delete instance;
		return PX4_ERROR;
	}

	pga460_descriptor.object.store(instance);
	pga460_descriptor.task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}

void PGA460::request_stop()
{
    ModuleBase::request_stop();

    close_uart();

    if (_topic_handle != nullptr) {
        orb_unadvertise(_topic_handle);
        _topic_handle = nullptr;
    }
}

int PGA460::print_status()
{
    PX4_INFO("Running");
    PX4_INFO("UART: %s", _device);
    PX4_INFO("Errors: %u", _current_errors);
    return 0;
}

int PGA460::custom_command(int argc, char *argv[])
{
    	return print_usage("unknown command");
}

int PGA460::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("pga460", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file>", "UART device", false);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}

int PGA460::open_uart(const char *device) {
	_uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("Failed to open %s (%i)", device, errno);
		return -1;
	}

	struct termios uart_config;
	if (tcgetattr(_uart_fd, &uart_config) != 0) {
		PX4_ERR("tcgetattr failed for %s: %d", device, errno);
		close(_uart_fd);
		uart_fd = -1;
		return -1;
	}

	// Настройка: 8N1 (8 бит, без четности, 1 стоп-бит)
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~CRTSCTS;

	// Сырой ввод/вывод (Raw mode)
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config.c_oflag &= ~OPOST;

	cfsetispeed(&uart_config, B115200);
	cfsetospeed(&uart_config, B115200);

	if (tcsetattr(_uart_fd, TCSANOW, &uart_config) != 0) {
		PX4_ERR("tcsetattr failed for %s: %d", device, errno);
		close(_uart_fd);
		uart_fd = -1;
		return -1;
	}
	return _uart_fd;
}

void PGA460::close_uart() {
	if (_uart_fd >= 0) {
		close(_uart_fd);
		_uart_fd = -1;
	}
}

ssize_t PGA460::write_data(const uint8_t *buffer, size_t len) {
	if (_uart_fd < 0) return -1;
	size_t total_written = 0;

	while (total_written < len) {
	    	ssize_t n = write(_uart_fd, buffer + total_written, len - total_written);
	    	if (n > 0) {
			total_written += n;
	    	} else if (n < 0) {
			if (errno == EINTR) {
		    	continue; // ← прерваны сигналом — просто повторить
		} else if (errno == EAGAIN) {
		    	// Буфер ядра заполнен — подождать и повторить
		    	usleep(100);
		    	continue;
		} else {
		    	// Настоящая ошибка (EBADF, EIO и т.д.)
		    	PX4_ERR("UART write error: %d", errno);
		    	return -1;
		}
	    }
	}
	return (ssize_t)total_written;
}

ssize_t PGA460::read_data(size_t bytes_needed) {
	if (_uart_fd < 0) return -1;
	size_t remaining = bytes_needed - _bytes_received;

	int n = read(_uart_fd, _read_buffer + _bytes_received, remaining);

	if (n > 0) {
	    _bytes_received += n;

	} else if (n < 0) {
	    if (errno == EAGAIN || errno == EINTR) {
		// Данных пока нет — это нормально, проверим таймаут ниже

	    } else {
		// Настоящая ошибка порта
		PX4_ERR("UART read error: %d", errno);
		_current_errors |= pga_data_s::ERR_COMM_FAILURE;
		_state = State::PUBLISH_DATA;
		ScheduleDelayed(10_ms);
		return -1;
	    }
	}

	// Набрали нужное количество байт
	if (_bytes_received >= bytes_needed) {
	    _state = State::PUBLISH_DATA;
	    ScheduleDelayed(10_ms);
	    return (ssize_t)_bytes_received;
	}

	// Таймаут — данные так и не пришли
	if (hrt_elapsed_time(&_last_state_change) > 100_ms) {
	    PX4_WARN("read timeout: got %zu of %zu bytes", _bytes_received, bytes_needed);
	    _current_errors |= pga_data_s::ERR_COMM_FAILURE;
	    _state = State::PUBLISH_DATA;
	    ScheduleDelayed(10_ms);
	    return (ssize_t)_bytes_received;
	}

	// Ещё не всё пришло — перепланировать через 1мс
	ScheduleDelayed(1_ms);
	return (ssize_t)_bytes_received;
}

void PGA460::Run() {
	if (should_exit()) {
		exit_and_cleanup(pga460_descriptor);
		return;
	}
	switch (_state) {
		case State::SEND_BURST:
			send_burst_and_listen();
			_state = State::WAIT_FOR_ECHO;
			_last_state_change = hrt_absolute_time();
			ScheduleDelayed(70_ms);
			break;

		case State::WAIT_FOR_ECHO:
			request_distance();
			_bytes_received = 0;
			_state = State::READ_DATA;
			_last_state_change = hrt_absolute_time();
			ScheduleDelayed(2_ms);
			break;

		case State::READ_DATA:
			read_data(6);
			break;

		case State::PUBLISH_DATA:
			float distance = distance_processing(_read_buffer, _bytes_received);
			uorb_publisher(distance);
			_state = State::SEND_BURST;
			_bytes_received = 0;
			ScheduleDelayed(50_ms);

	}
}

uint8_t PGA460::calculate_checksum(uint8_t *data, size_t len) {
	uint16_t sum = 0;
	for (size_t i = 0; i < len; i++) {
		sum += data[i];
	}
	return (uint8_t)~(sum & 0xFF);
}

void PGA460::send_burst_and_listen() {
	uint8_t body[2];
	body[0] = 0x00;
	body[1] = 0x01;

	uint8_t checksum = calculate_checksum(body, 2);

	uint8_t packet[4] = {0x55, body[0], body[1], checksum};

	if (write_data(packet, sizeof(packet)) != sizeof(packet)) {
		_current_errors |= pga_data_s::ERR_COMM_FAILURE;
	}
}

void PGA460::request_distance() {
    	uint8_t body[1] = {0x05};
    	uint8_t checksum = calculate_checksum(body, 1);

    	uint8_t packet[3] = {0x55, body[0], checksum};

    	if (write_data(packet, sizeof(packet)) != sizeof(packet)) {
		_current_errors |= pga_data_s::ERR_COMM_FAILURE;
	}
}

float PGA460::distance_processing(uint8_t *response, size_t len) {
	if (len != 6) {
		_current_errors |= pga_data_s::ERR_COMM_FAILURE;
		tcflush(_uart_fd, TCIFLUSH);
		return -1.0f;
	}

	uint8_t calc_cs = calculate_checksum(response, 5);
	if (calc_cs != response[5]) {
		_current_errors |= pga_data_s::ERR_UART_INVALID_SLAVE_CHECKSUM;
		return -1.0f;
	}

	uint8_t diag_data = response[0];

	if (diag_data & 0x3E) {
		if (diag_data & (1 << 1)) _current_errors |= pga_data_s::ERR_UART_BAUD_RATE_MISMATCH;
		if (diag_data & (1 << 2)) _current_errors |= pga_data_s::ERR_UART_SYNC_STABILITY;
		if (diag_data & (1 << 3)) {
			_current_errors |= pga_data_s::ERR_UART_INVALID_MASTER_CHECKSUM;
			tcflush(_uart_fd, TCOFLUSH);
		}
		if (diag_data & (1 << 4)) _current_errors |= pga_data_s::ERR_UART_UNKNOWN_COMMAND;
		if (diag_data & (1 << 5)) _current_errors |= pga_data_s::ERR_UART_FRAMING_FAILURE;
		return -1.0f;
	}

	uint16_t tof = (response[1] << 8) | response[2];
	float distance = (331.0f * tof * 1e-6f) / 2.0f;
	float burst_offset = 331.0f * 8 / 40000.0f / 2.0f;
	return distance + burst_offset;
}

void PGA460::uorb_publisher(float distance_raw) {
	struct pga_data_s my_data{};

	my_data.timestamp = hrt_absolute_time();
	my_data.distance_m = distance_raw;
	my_data.max_distance = 11.0f;

	my_data.status_flags = _current_errors;

	if (_topic_handle == nullptr) {
		_topic_handle = orb_advertise(ORB_ID(pga_data), &my_data);
	} else {
		orb_publish(ORB_ID(pga_data), _topic_handle, &my_data);
	}

	_current_errors = 0;
}

bool PGA460::write_register(uint8_t reg, uint8_t value) {
	uint8_t body[3];
	body[0] = 0x10;
	body[1] = reg;
	body[2] = value;

	uint8_t checksum = calculate_checksum(body, 3);
	uint8_t packet[5] = {0x55, body[0], body[1], body[2], checksum};

	if (write_data(packet, sizeof(packet)) != (ssize_t)sizeof(packet)) {
	    PX4_ERR("write_register failed: reg=0x%02X val=0x%02X", reg, value);
	    return false; // ← теперь вызывающий знает об ошибке
	}

	return true;
    }

bool PGA460::init_hw() {
	for (size_t i = 0; i < sizeof(pga460_config) / sizeof(pga460_config[0]); i++) {
	    if (!write_register(pga460_config[i].addr, pga460_config[i].value)) {
		PX4_ERR("init_hw failed at step %zu (reg=0x%02X)",
			 i, pga460_config[i].addr);
		return false; // ← прерываем инициализацию
	    }
	    usleep(2000);
	}

	PX4_INFO("PGA460 hardware initialized successfully");
	return true;
    }

ModuleBase::Descriptor PGA460::pga460_descriptor {
    &PGA460::task_spawn,
    &PGA460::custom_command,
    &PGA460::print_usage
};

int pga460_main(int argc, char *argv[]) {
    return ModuleBase::main(PGA460::pga460_descriptor, argc, argv);
}

