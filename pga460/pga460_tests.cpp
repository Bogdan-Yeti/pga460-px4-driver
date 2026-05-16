#include <gtest/gtest.h>
#include "pga460.h"
#include <math.h>

// TEST_F generates a subclass of the fixture, which is NOT a friend of PGA460.
// All private-member access must go through protected wrapper methods here,
// since PGA460UnitTest itself IS a friend.
class PGA460UnitTest : public ::testing::Test {
protected:
	// PGA460 constructor calls open_uart; /dev/null is not a tty so
	// tcgetattr fails and _uart_fd stays -1.  Pure computation still works.
	PGA460UnitTest() : _drv("/dev/null") {}

	// ── rx buffer helpers ────────────────────────────────────────────────

	void set_temp_response(uint8_t raw_temp, uint8_t noise = 0x00, bool corrupt = false)
	{
		_drv._rx_buf[0] = 0x00; // diag: no errors
		_drv._rx_buf[1] = raw_temp;
		_drv._rx_buf[2] = noise;
		uint8_t cs = PGA460::calculate_checksum(&_drv._rx_buf[1], TEMP_RESP_LEN - 2);
		_drv._rx_buf[3] = corrupt ? (uint8_t)(cs ^ 0xFF) : cs;
		_drv._rx_received = TEMP_RESP_LEN;
	}

	void set_dist_response(uint16_t tof, bool corrupt = false)
	{
		_drv._rx_buf[0] = 0x00; // diag: no errors
		_drv._rx_buf[1] = (tof >> 8) & 0xFF;
		_drv._rx_buf[2] = tof & 0xFF;
		_drv._rx_buf[3] = 0x00; // width
		_drv._rx_buf[4] = 0x00; // amplitude
		uint8_t cs = PGA460::calculate_checksum(&_drv._rx_buf[1], DIST_RESP_LEN - 2);
		_drv._rx_buf[5] = corrupt ? (uint8_t)(cs ^ 0xFF) : cs;
		_drv._rx_received = DIST_RESP_LEN;
	}

	// ── private-method wrappers (friend access lives here) ───────────────

	float           parse_temperature()          { return _drv.parse_temperature(); }
	float           parse_distance()             { return _drv.parse_distance(); }
	bool            parse_diag_byte(uint8_t d)   { return _drv.parse_diag_byte(d); }
	uint8_t         current_errors()             { return _drv._current_errors; }
	void            reset_errors()               { _drv._current_errors = 0; }
	void            set_temperature(float t)     { _drv._temperature = t; }

	PGA460 _drv;
};

// ── calculate_checksum ────────────────────────────────────────────────────────

TEST_F(PGA460UnitTest, ChecksumEmptyData)
{
	// sum=0 → ~0 & 0xFF = 0xFF
	uint8_t dummy = 0;
	EXPECT_EQ(PGA460::calculate_checksum(&dummy, 0), 0xFF);
}

TEST_F(PGA460UnitTest, ChecksumSingleByte)
{
	uint8_t data[] = {0x04};
	// sum=4 → ~4 & 0xFF = 0xFB
	EXPECT_EQ(PGA460::calculate_checksum(data, 1), 0xFB);
}

TEST_F(PGA460UnitTest, ChecksumMultiByteWrap)
{
	// 16-bit accumulation must wrap to 8-bit complement
	uint8_t data[] = {0xFF, 0xFF, 0x01};
	uint16_t sum = 0xFF + 0xFF + 0x01; // = 0x1FF
	EXPECT_EQ(PGA460::calculate_checksum(data, 3), (uint8_t)~(sum & 0xFF));
}

// ── parse_temperature ─────────────────────────────────────────────────────────

TEST_F(PGA460UnitTest, TemperatureAt0C)
{
	// raw=64 → (64-64)/1.5 = 0°C
	set_temp_response(64);
	EXPECT_FLOAT_EQ(parse_temperature(), 0.0f);
}

TEST_F(PGA460UnitTest, TemperatureAt20C)
{
	// raw=94 → (94-64)/1.5 = 20°C
	set_temp_response(94);
	EXPECT_FLOAT_EQ(parse_temperature(), 20.0f);
}

TEST_F(PGA460UnitTest, TemperatureNegative)
{
	// raw=49 → (49-64)/1.5 = -10°C — confirms uint8 underflow fix works
	set_temp_response(49);
	EXPECT_NEAR(parse_temperature(), -10.0f, 0.01f);
}

TEST_F(PGA460UnitTest, TemperatureBadChecksumReturnsNAN)
{
	set_temp_response(94, 0x00, /*corrupt=*/true);
	EXPECT_TRUE(isnan(parse_temperature()));
}

// ── parse_distance ────────────────────────────────────────────────────────────

TEST_F(PGA460UnitTest, DistanceZeroToF)
{
	// tof=0 → dist=0, result is blind-zone offset only (must be > 0)
	set_temperature(20.0f);
	set_dist_response(0);
	float d = parse_distance();
	EXPECT_FALSE(isnan(d));
	EXPECT_GT(d, 0.0f);
}

TEST_F(PGA460UnitTest, DistanceKnownValue)
{
	// At 20°C: v=331+0.6*20=343 m/s
	// tof=1000 µs → dist=343*1000e-6/2=0.1715 m
	// offset=343*8/40000/2=0.03437 m → total≈0.2059 m
	set_temperature(20.0f);
	set_dist_response(1000);
	EXPECT_NEAR(parse_distance(), 0.2059f, 0.001f);
}

TEST_F(PGA460UnitTest, DistanceBadChecksumReturnsNAN)
{
	set_temperature(20.0f);
	set_dist_response(1000, /*corrupt=*/true);
	EXPECT_TRUE(isnan(parse_distance()));
}

// ── parse_diag_byte ───────────────────────────────────────────────────────────

TEST_F(PGA460UnitTest, DiagClean)
{
	EXPECT_TRUE(parse_diag_byte(0x00));
	EXPECT_EQ(current_errors(), 0);
}

TEST_F(PGA460UnitTest, DiagBaudError)
{
	reset_errors();
	EXPECT_FALSE(parse_diag_byte(DIAG_BAUD_ERR));
	EXPECT_TRUE(current_errors() & pga_data_s::ERR_UART_BAUD_RATE_MISMATCH);
}

TEST_F(PGA460UnitTest, DiagMultipleErrors)
{
	reset_errors();
	EXPECT_FALSE(parse_diag_byte(DIAG_SYNC_ERR | DIAG_FRAMING_ERR));
	EXPECT_TRUE(current_errors() & pga_data_s::ERR_UART_SYNC_STABILITY);
	EXPECT_TRUE(current_errors() & pga_data_s::ERR_UART_FRAMING_FAILURE);
}

