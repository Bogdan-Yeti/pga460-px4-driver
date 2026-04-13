#pragma once

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
#include <errno.h>

enum class State {
    SEND_BURST,
    WAIT_FOR_ECHO,
    READ_DATA,
    PUBLISH_DATA,
    TEMP_PROC
};

class PGA460 : public ModuleBase, public px4::ScheduledWorkItem {
public:
	PGA460(const char *device);
    	~PGA460() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	static ModuleBase::Descriptor pga460_descriptor;

private:
	bool _initialized{false};
	State _state{State::SEND_BURST};
	State _next_state{State::SEND_BURST};
	int _uart_fd{-1};
	char _device[20]{};

	float _temperature{10.0f};
	bool _temperature_valid{false};
	hrt_abstime _last_temp_meas{0};

	orb_advert_t _topic_handle = nullptr;

	uint8_t _current_errors{0};
	uint8_t _read_buffer[10];
	size_t _bytes_received{0};
	hrt_abstime _last_state_change{0};

	void Run() override;

	void request_stop() override;

	int open_uart(const char *device);
	void close_uart();

	ssize_t write_data(const uint8_t *buffer, size_t len);
	ssize_t read_data(size_t bytes_needed);

	uint8_t calculate_checksum(uint8_t *data, size_t len);

	void send_burst_and_listen();
	void request_distance();
	void request_temperature();
	float distance_processing(uint8_t *response, size_t len);
	float temperature_processing(uint8_t *response, size_t len);
	void uorb_publisher(float distance_raw);

	bool write_register(uint8_t reg, uint8_t value);
	bool init_hw();
};

struct pga460_config_t {
    uint8_t addr;
    uint8_t value;
};




