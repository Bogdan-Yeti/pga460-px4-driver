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
#include <poll.h>
#include <errno.h>

enum class State {
    SEND_BURST,
    WAIT_FOR_ECHO,
    READ_DATA
};

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
	State _state{State::SEND_BURST};
	int _uart_fd{-1};
	char _device[20]{};

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
	void read_data(size_t bytes_needed);

	uint8_t calculate_checksum(uint8_t *data, size_t len);

	void send_burst_and_listen();
	void request_distance();
	float distance_processing(uint8_t *response);
	void uorb_publisher(float distance_raw);
};

static ModuleBase::Descriptor pga460_descriptor(
	&PGA460::task_spawn,
	&PGA460::custom_command,
	&PGA460::print_usage
);

