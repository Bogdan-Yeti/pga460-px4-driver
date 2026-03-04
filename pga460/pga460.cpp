#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/pga_data.h>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>

using namespace time_literals;

extern "C" __EXPORT int pga460_main(int argc, char *argv[]);

class PGA460 : public ModuleBase, public px4::ScheduledWorkItem {
public:
	PGA460(const char *device);
    	~PGA460() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	bool _initialized{false};
	bool _waiting_for_echo{false};
	int _uart_fd{-1};
	char _device[20]{};
	uint8_t _current_errors{0};
	orb_advert_t _topic_handle = nullptr;

	void Run() override;

	void request_stop() override;

	int open_uart(const char *device);
	void close_uart();

	ssize_t write_data(const uint8_t *buffer, size_t len);
	ssize_t read_data(uint8_t *buffer, size_t len, int timeout_ms);

	uint8_t calculate_checksum(uint8_t *data, size_t len);

	void send_burst_and_listen();
	float request_distance();
	void uorb_publisher(float distance_raw);
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

ssize_t PGA460::read_data(uint8_t *buffer, size_t len, int timeout_ms) {
	if (_uart_fd < 0) return -1;
	size_t remaining = len;
	int64_t starting_time = hrt_absolute_time();
	while (remaining > 0) {

		int64_t elapsed_ms = (hrt_absolute_time() - starting_time) / 1000;

		if (elapsed_ms >= timeout_ms) {
			return -1;
		}

		struct pollfd fds[1];
		fds[0].fd = _uart_fd;
		fds[0].events = POLLIN;

		int ret = poll(fds, 1, timeout_ms - elapsed_ms);
		if (ret <= 0) return -1;

		ssize_t n = read(_uart_fd, buffer + (len - remaining), remaining);
		if (n <= 0) return n;

		remaining -= n;
	}
	return len;
}

void PGA460::Run() {
	if (should_exit()) {
		exit_and_cleanup(pga460_descriptor);
		return;
	}
	if (!_waiting_for_echo) {
		send_burst_and_listen();
		_waiting_for_echo = true;
		ScheduleDelayed(100_ms);
	} else {
		uorb_publisher(request_distance());
		_waiting_for_echo = false;
		ScheduleDelayed(100_ms);
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

float PGA460::request_distance() {
    	uint8_t body[1] = {0x05};
    	uint8_t checksum = calculate_checksum(body, 1);

    	uint8_t packet[3] = {0x55, body[0], checksum};

    	if (write_data(packet, sizeof(packet)) != sizeof(packet)) {
		_current_errors |= pga_data_s::ERR_COMM_FAILURE;
	}

	uint8_t response[6];
	int n = read_data(response, sizeof(response), 50);

	if (n != 6) {
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

static ModuleBase::Descriptor pga460_descriptor(
	&PGA460::task_spawn,
	&PGA460::custom_command,
	&PGA460::print_usage
);

int pga460_main(int argc, char *argv[]) {
    return ModuleBase::main(pga460_descriptor, argc, argv);
}

