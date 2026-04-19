#include "pga460.h"
#include <math.h>

using namespace time_literals;

extern "C" __EXPORT int pga460_main(int argc, char *argv[]);

static const pga460_config_t pga460_config[] = {
    // Frequency & pulse
    {0x14, 0x32}, // FREQ: 40 kHz
    {0x15, 0x08}, // P1_PULSE_COUNT: 8 pulses
    {0x10, 0x40}, // CURR_LIM_P1
    {0x1B, 0x0F}, // DECPL_TIME: reduces blind zone

    // Time Varied Gain
    {0x1D, 0x11}, // TVG_RANGE
    {0x1E, 0x88}, // TVG_GAIN_0
    {0x1F, 0xAA}, // TVG_GAIN_1
    {0x20, 0xCC}, // TVG_GAIN_2
    {0x21, 0xFF}, // TVG_GAIN_3

    // Thresholds for preset P1
    {0x40, 0xEE}, {0x41, 0xEE}, {0x42, 0xDD}, // 0–50 cm  (high)
    {0x43, 0xBB}, {0x44, 0x88}, {0x45, 0x77}, // 50 cm–2 m
    {0x46, 0x66}, {0x47, 0x55}, {0x48, 0x44}, // 2 m+     (low)
};

// ─── Module boilerplate ────────────────────────────────────────────────────────

PGA460::PGA460(const char *device) :
    ModuleBase(),
    ScheduledWorkItem("PGA460", px4::wq_configurations::lp_default)
{
    strncpy(_device, device, sizeof(_device) - 1);
    _device[sizeof(_device) - 1] = '\0';
    open_uart(_device);
}

int PGA460::task_spawn(int argc, char *argv[])
{
    const char *device = "/dev/ttyS1";
    int ch, myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        if (ch == 'd') device = myoptarg;
    }

    PGA460 *instance = new PGA460(device);
    if (!instance || instance->_uart_fd < 0) {
        delete instance;
        return PX4_ERROR;
    }

    if (!instance->init_hw()) {
        PX4_ERR("Hardware init failed");
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
    if (_topic_handle) { orb_unadvertise(_topic_handle); _topic_handle = nullptr; }
}

int PGA460::print_status()
{
    PX4_INFO("Running on %s, errors: 0x%02X", _device, _current_errors);
    return 0;
}

int PGA460::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int PGA460::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_USAGE_NAME("pga460", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file>", "UART device", false);
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");
    return 0;
}

// ─── State machine ─────────────────────────────────────────────────────────────

void PGA460::Run()
{
    if (should_exit()) {
        exit_and_cleanup(pga460_descriptor);
        return;
    }

    switch (_state) {

    case State::SEND_BURST:
        if (!_temperature_valid || hrt_elapsed_time(&_last_temp_meas) >= 10_s) {
            cmd_temp_req();   // → WRITING → READING → PROC_TEMP
        } else {
            cmd_burst();      // → WRITING → WAIT_ECHO
        }
        break;

    case State::WRITING:
        if (tx_flush()) {
            _state = _after_write;
            ScheduleDelayed(_after_write_delay);
        } else {
            ScheduleDelayed(500_us);
        }
        break;

    case State::WAIT_ECHO:
        cmd_dist_req();       // → WRITING → READING → PROC_DIST
        break;

    case State::READING:
        if (rx_collect()) {
            _state = _after_read;
            ScheduleNow();
        } else {
            ScheduleDelayed(1_ms);
        }
        break;

    case State::PROC_TEMP: {
        float temp = parse_temperature();
        if (isfinite(temp)) {
            _temperature       = temp;
            _temperature_valid = true;
            _last_temp_meas    = hrt_absolute_time();
        } else if (_temperature_valid) {
            _last_temp_meas = hrt_absolute_time();
        }
        _state = State::SEND_BURST;
        ScheduleDelayed(_temperature_valid ? 1_ms : 50_ms);
        break;
    }

    case State::PROC_DIST:
        publish(parse_distance());
        _state = State::SEND_BURST;
        ScheduleDelayed(50_ms);
        break;
    }
}

// ─── Non-blocking TX ──────────────────────────────────────────────────────────

void PGA460::start_write(const uint8_t *data, size_t len, State after, hrt_abstime delay)
{
    memcpy(_tx_buf, data, len);
    _tx_len            = len;
    _tx_offset         = 0;
    _after_write       = after;
    _after_write_delay = delay;
    _state             = State::WRITING;
}

bool PGA460::tx_flush()
{
    while (_tx_offset < _tx_len) {
        ssize_t n = write(_uart_fd, _tx_buf + _tx_offset, _tx_len - _tx_offset);
        if (n > 0) {
            _tx_offset += n;
        } else if (errno == EAGAIN || errno == EINTR) {
            return false;
        } else {
            PX4_ERR("UART write error: %d", errno);
            _current_errors |= pga_data_s::ERR_COMM_FAILURE;
            _tx_offset = _tx_len;
            return true;
        }
    }
    return true;
}

// ─── Non-blocking RX ──────────────────────────────────────────────────────────

void PGA460::start_read(size_t bytes, State after)
{
    _rx_received = 0;
    _rx_needed   = bytes;
    _after_read  = after;
}

bool PGA460::rx_collect()
{
    if (_rx_received == 0) _rx_start_time = hrt_absolute_time();

    while (_rx_received < _rx_needed) {
        ssize_t n = read(_uart_fd, _rx_buf + _rx_received, _rx_needed - _rx_received);
        if (n > 0) {
            _rx_received += n;
        } else if (errno == EAGAIN || errno == EINTR) {
            if (hrt_elapsed_time(&_rx_start_time) > 100_ms) {
                PX4_WARN("read timeout: %zu/%zu bytes", _rx_received, _rx_needed);
                _current_errors |= pga_data_s::ERR_COMM_FAILURE;
                return true;
            }
            return false;
        } else {
            PX4_ERR("UART read error: %d", errno);
            _current_errors |= pga_data_s::ERR_COMM_FAILURE;
            return true;
        }
    }
    return true;
}

// ─── Protocol ─────────────────────────────────────────────────────────────────

uint8_t PGA460::calculate_checksum(const uint8_t *data, size_t len)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return (uint8_t)~(sum & 0xFF);
}

bool PGA460::parse_diag_byte(uint8_t diag)
{
    if (!(diag & 0x3E)) return true;
    if (diag & (1 << 1)) _current_errors |= pga_data_s::ERR_UART_BAUD_RATE_MISMATCH;
    if (diag & (1 << 2)) _current_errors |= pga_data_s::ERR_UART_SYNC_STABILITY;
    if (diag & (1 << 3)) { _current_errors |= pga_data_s::ERR_UART_INVALID_MASTER_CHECKSUM; tcflush(_uart_fd, TCOFLUSH); }
    if (diag & (1 << 4)) _current_errors |= pga_data_s::ERR_UART_UNKNOWN_COMMAND;
    if (diag & (1 << 5)) _current_errors |= pga_data_s::ERR_UART_FRAMING_FAILURE;
    return false;
}

void PGA460::cmd_burst()
{
    uint8_t body[2]   = {0x00, 0x01};
    uint8_t packet[4] = {0x55, body[0], body[1], calculate_checksum(body, 2)};
    start_write(packet, sizeof(packet), State::WAIT_ECHO, 70_ms);
}

void PGA460::cmd_dist_req()
{
    uint8_t body[1]   = {0x05};
    uint8_t packet[3] = {0x55, body[0], calculate_checksum(body, 1)};
    start_read(6, State::PROC_DIST);
    start_write(packet, sizeof(packet), State::READING, 2_ms);
}

void PGA460::cmd_temp_req()
{
    uint8_t body[1]   = {0x04};
    uint8_t packet[3] = {0x55, body[0], calculate_checksum(body, 1)};
    start_read(4, State::PROC_TEMP);
    start_write(packet, sizeof(packet), State::READING, 1_ms);
}

float PGA460::parse_distance()
{
    if (_rx_received != 6) {
        _current_errors |= pga_data_s::ERR_COMM_FAILURE;
        tcflush(_uart_fd, TCIFLUSH);
        return NAN;
    }
    if (calculate_checksum(&_rx_buf[1], 4) != _rx_buf[5]) {
        _current_errors |= pga_data_s::ERR_UART_INVALID_SLAVE_CHECKSUM;
        return NAN;
    }
    if (!parse_diag_byte(_rx_buf[0])) return NAN;

    uint16_t tof     = ((uint16_t)_rx_buf[1] << 8) | _rx_buf[2];
    float    v_sound = 331.0f + 0.6f * _temperature;
    float    dist    = (v_sound * tof * 1e-6f) / 2.0f;
    float    offset  = v_sound * 8.0f / 40000.0f / 2.0f;  // burst-length correction
    return dist + offset;
}

float PGA460::parse_temperature()
{
    if (_rx_received != 4) {
        _current_errors |= pga_data_s::ERR_COMM_FAILURE;
        tcflush(_uart_fd, TCIFLUSH);
        return NAN;
    }
    if (calculate_checksum(&_rx_buf[1], 2) != _rx_buf[3]) {
        _current_errors |= pga_data_s::ERR_UART_INVALID_SLAVE_CHECKSUM;
        return NAN;
    }
    if (!parse_diag_byte(_rx_buf[0])) return NAN;

    return (_rx_buf[1] - 64) / 1.5f;
}

void PGA460::publish(float distance)
{
    pga_data_s msg{};
    msg.timestamp    = hrt_absolute_time();
    msg.distance_m   = distance;
    msg.max_distance = 11.0f;
    msg.status_flags = _current_errors;
    _current_errors  = 0;

    if (_topic_handle == nullptr) {
        _topic_handle = orb_advertise(ORB_ID(pga_data), &msg);
    } else {
        orb_publish(ORB_ID(pga_data), _topic_handle, &msg);
    }
}

// ─── UART ─────────────────────────────────────────────────────────────────────

int PGA460::open_uart(const char *device)
{
    _uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_uart_fd < 0) { PX4_ERR("Failed to open %s (%i)", device, errno); return -1; }

    struct termios cfg;
    if (tcgetattr(_uart_fd, &cfg) != 0) {
        PX4_ERR("tcgetattr failed: %d", errno);
        close(_uart_fd); _uart_fd = -1; return -1;
    }

    cfg.c_cflag |= (CLOCAL | CREAD);
    cfg.c_cflag  = (cfg.c_cflag & ~CSIZE) | CS8;
    cfg.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    cfg.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    cfg.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    cfg.c_oflag &= ~OPOST;
    cfsetispeed(&cfg, B115200);
    cfsetospeed(&cfg, B115200);

    if (tcsetattr(_uart_fd, TCSANOW, &cfg) != 0) {
        PX4_ERR("tcsetattr failed: %d", errno);
        close(_uart_fd); _uart_fd = -1; return -1;
    }
    return _uart_fd;
}

void PGA460::close_uart()
{
    if (_uart_fd >= 0) { close(_uart_fd); _uart_fd = -1; }
}

// ─── Hardware init (blocking — runs before work queue starts) ─────────────────

bool PGA460::write_register(uint8_t reg, uint8_t value)
{
    uint8_t body[3]   = {0x10, reg, value};
    uint8_t packet[5] = {0x55, body[0], body[1], body[2], calculate_checksum(body, 3)};

    size_t offset = 0;
    while (offset < sizeof(packet)) {
        ssize_t n = write(_uart_fd, packet + offset, sizeof(packet) - offset);
        if      (n > 0)                             offset += n;
        else if (errno == EAGAIN || errno == EINTR) usleep(100);
        else { PX4_ERR("write_register failed: reg=0x%02X", reg); return false; }
    }
    return true;
}

bool PGA460::init_hw()
{
    for (size_t i = 0; i < sizeof(pga460_config) / sizeof(*pga460_config); i++) {
        if (!write_register(pga460_config[i].addr, pga460_config[i].value)) {
            PX4_ERR("init_hw failed at step %zu (reg=0x%02X)", i, pga460_config[i].addr);
            return false;
        }
        usleep(2000);
    }
    PX4_INFO("PGA460 initialized");
    return true;
}

// ─── Entry point ──────────────────────────────────────────────────────────────

ModuleBase::Descriptor PGA460::pga460_descriptor {
    &PGA460::task_spawn,
    &PGA460::custom_command,
    &PGA460::print_usage
};

int pga460_main(int argc, char *argv[]) {
    return ModuleBase::main(PGA460::pga460_descriptor, argc, argv);
}

