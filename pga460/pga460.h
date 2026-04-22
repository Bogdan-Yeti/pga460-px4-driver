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

#define PGA460_SYNC_BYTE          0x55U

#define PGA460_CMD_TEMP_REQ       0x04U
#define PGA460_CMD_DIST_REQ       0x05U
#define PGA460_CMD_BURST          0x00U
#define PGA460_CMD_WRITE_REG      0x10U

#define PGA460_COUNT_OBJECT       0x01U

#define PGA460_TEMP_RESP_LEN      4U
#define PGA460_DIST_RESP_LEN      6U

#define PGA460_DIAG_ERROR_MASK    0x3EU
#define PGA460_DIAG_BAUD_ERR      (1U << 1)
#define PGA460_DIAG_SYNC_ERR      (1U << 2)
#define PGA460_DIAG_CHECKSUM_ERR  (1U << 3)
#define PGA460_DIAG_UNKNOWN_CMD   (1U << 4)
#define PGA460_DIAG_FRAMING_ERR   (1U << 5)

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
    bool    validate_response(size_t expected_len);
    void    cmd_burst();
    void    cmd_dist_req();
    void    cmd_temp_req();
    float   parse_distance();
    float   parse_temperature();
    void    publish(float distance);

    bool write_register(uint8_t reg, uint8_t value);
    bool init_hw();
};

