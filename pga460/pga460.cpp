#include "pga460.h"

using namespace time_literals;

extern "C" __EXPORT int pga460_main(int argc, char *argv[]);

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

	PGA460 *instance = new PGA460(device);

	if (!instance) {
		return PX4_ERROR;
	}

	if (!instance->_initialized) {
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
	tcgetattr(_uart_fd, &uart_config);

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

	cfsetispeed(&uart_config, B9600);
	cfsetospeed(&uart_config, B9600);

	tcsetattr(_uart_fd, TCSANOW, &uart_config);
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
	return write(_uart_fd, buffer, len);
}

void PGA460::read_data(size_t bytes_needed) {
    uint8_t temp_buf[10];
    int n = read(_uart_fd, temp_buf, sizeof(temp_buf));

    if (n > 0) {
        for (int i = 0; i < n && _bytes_received < sizeof(_read_buffer); i++) {
            _read_buffer[_bytes_received++] = temp_buf[i];
        }
    }
    if (_bytes_received >= bytes_needed) {
        float distance = distance_processing(_read_buffer);
        uorb_publisher(distance);
        _state = State::SEND_BURST;
        ScheduleDelayed(50_ms);
    } else {
        if (hrt_elapsed_time(&_last_state_change) > 100_ms) {
            _current_errors |= pga_data_s::ERR_COMM_FAILURE;
	    uorb_publisher(-1.0f);
            _state = State::SEND_BURST;
            ScheduleDelayed(10_ms);
            return;
        }
        ScheduleDelayed(1_ms);
    }
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
			ScheduleDelayed(30_ms);
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

float PGA460::distance_processing(uint8_t *response) {
	if (sizeof(response) != 6) {
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
	return (331.0f * tof * 0.00000001f) / 2.0f;
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

int pga460_main(int argc, char *argv[]) {
    return ModuleBase::main(pga460_descriptor, argc, argv);
}

