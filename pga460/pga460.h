#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/pga_data.h>

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

enum class State : uint8_t {
    SEND_BURST,
    WRITING,
    WAIT_ECHO,
    READING,
    PROC_TEMP,
    PROC_DIST,
};

struct pga460_config_t {
    uint8_t addr;
    uint8_t value;
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
    int  _uart_fd{-1};
    char _device[20]{};

    uint8_t     _tx_buf[8]{};
    size_t      _tx_len{0};
    size_t      _tx_offset{0};
    State       _after_write{State::WAIT_ECHO};
    hrt_abstime _after_write_delay{0};

    uint8_t     _rx_buf[10]{};
    size_t      _rx_received{0};
    size_t      _rx_needed{0};
    State       _after_read{State::PROC_TEMP};
    hrt_abstime _rx_start_time{0};

    State _state{State::SEND_BURST};

    float       _temperature{10.0f};
    bool        _temperature_valid{false};
    hrt_abstime _last_temp_meas{0};
    uint8_t     _current_errors{0};

    orb_advert_t _topic_handle{nullptr};

    void Run() override;
    void request_stop() override;

    int  open_uart(const char *device);
    void close_uart();

    void start_write(const uint8_t *data, size_t len, State after, hrt_abstime delay);
    bool tx_flush();

    void start_read(size_t bytes, State after);
    bool rx_collect();

    uint8_t calculate_checksum(const uint8_t *data, size_t len);
    bool    parse_diag_byte(uint8_t diag);
    void    cmd_burst();
    void    cmd_dist_req();
    void    cmd_temp_req();
    float   parse_distance();
    float   parse_temperature();
    void    publish(float distance);

    bool write_register(uint8_t reg, uint8_t value);
    bool init_hw();
};

