#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <uORB/topics/pga_data.h>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>

extern "C" __EXPORT int pga460_main(int argc, char *argv[]);

class PGA460 {
public:
	PGA460() = default;
	~PGA460() { stop(); }

	int start(const char *device);

	void stop();

	static int task_main_helper(int argc, char *argv[]);

private:
	bool _task_should_exit{false};
	int _uart_fd{-1};
	char _device[20]{};

	orb_advert_t _topic_handle = nullptr;

	int open_uart(const char *device);

	ssize_t write_data(const uint8_t *buffer, size_t len);

	ssize_t read_data(uint8_t *buffer, size_t len, int timeout_ms);

	void task_main();

	uint8_t calculate_checksum(uint8_t *data, size_t len);

	void send_burst_and_listen();

	int16_t request_distance();

	void uorb_publisher(int16_t distance_raw);
};

static PGA460 *g_driver = nullptr;

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
	uart_config.c_cflag &= ~CRTSCTS;    // Без аппаратного управления потоком

	// Сырой ввод/вывод (Raw mode)
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config.c_oflag &= ~OPOST;

	cfsetispeed(&uart_config, B9600);
	cfsetospeed(&uart_config, B9600);

	tcsetattr(_uart_fd, TCSANOW, &uart_config);
	return _uart_fd;
}

ssize_t PGA460::write_data(const uint8_t *buffer, size_t len) {
	if (_uart_fd < 0) return -1;
	return write(_uart_fd, buffer, len);
}

ssize_t PGA460::read_data(uint8_t *buffer, size_t len, int timeout_ms) {
	struct pollfd fds[1];
	fds[0].fd = _uart_fd;
	fds[0].events = POLLIN;

	// Ждем данные
	int ret = poll(fds, 1, timeout_ms);

	if (ret > 0 && (fds[0].revents & POLLIN)) {
		return read(_uart_fd, buffer, len);
	}
	return ret; // 0 - timeout, <0 - error
}

void PGA460::task_main() {
	if (open_uart(_device) < 0) return;

	PX4_INFO("Driver started on %s", _device);

	while (!_task_should_exit) {
		send_burst_and_listen();

		// Ждем, пока звук долетит до препятствия и вернется.
		// Для 3-5 метров достаточно 40-60 мс.
		px4_usleep(60000);

		// Запрашиваем результат
		uorb_publisher(request_distance());
	}

	close(_uart_fd);
	_uart_fd = -1;
}

int PGA460::task_main_helper(int argc, char *argv[]) {
	g_driver->task_main();
	return 0;
}

int PGA460::start(const char *device) {
	strncpy(_device, device, sizeof(_device));
	_task_should_exit = false;

	int task_id = px4_task_spawn_cmd("pga460_driver",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT,
					1500,
					&PGA460::task_main_helper,
					nullptr);
	return (task_id > 0) ? 0 : -1;
}

void PGA460::stop() {
    	_task_should_exit = true;
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
	body[0] = 0x00; // Команда Burst and Listen
	body[1] = 0x01; // Искать 1 объект

	uint8_t checksum = calculate_checksum(body, 2);

	uint8_t packet[4] = {0x55, body[0], body[1], checksum};

	write_data(packet, sizeof(packet));
}

int16_t PGA460::request_distance() {
    	uint8_t body[1] = {0x05}; // Команда запроса результатов
    	uint8_t checksum = calculate_checksum(body, 1);

    	uint8_t packet[3] = {0x55, body[0], checksum};

    	write_data(packet, sizeof(packet));

    	uint8_t response[6];
    	int n = read_data(response, sizeof(response), 50);

    	if (n > 0) {
		return (response[1] << 8) | response[2];
    	}
    	return -1;
}

void PGA460::uorb_publisher(int16_t distance_raw) {
	struct pga_data_s my_data{};

	my_data.timestamp = hrt_absolute_time();
	my_data.distance_m = (float)distance_raw / 1000.0f;

	if (_topic_handle == nullptr) {
		_topic_handle = orb_advertise(ORB_ID(pga_data), &my_data);
	} else {
		orb_publish(ORB_ID(pga_data), _topic_handle, &my_data);
	}
}

int pga460_main(int argc, char *argv[]) {
    	if (argc < 2) {
        	PX4_INFO("Usage: pga460 {start|stop|status} [-d /dev/ttySXX]");
        	return 1;
    	}

    	const char *device = "/dev/ttyS6"; // Значение по умолчанию
    	int ch;
    	int myoptind = 1;
    	const char *myoptarg = nullptr;
    	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        	if (ch == 'd') device = myoptarg;
    	}

	if (!strcmp(argv[1], "start")) {
		if (g_driver) {
			PX4_WARN("Already running");
			return 0;
		}
		g_driver = new PGA460();
		if (g_driver->start(device) != 0) {
			delete g_driver;
			g_driver = nullptr;
			return 1;
		}
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (g_driver) {
			delete g_driver;
			g_driver = nullptr;
			PX4_INFO("Stopped");
		}
		return 0;
	}

	return 0;
}
