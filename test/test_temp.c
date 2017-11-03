#include "unity.h"
#include "temp.h"
#include "temp_statics.h"

#include "mock_serial.h"
#include "mock_sendf.h"
#include "mock_analog.h"
#include "mock_heater.h"

volatile uint8_t debug_flags;

static uint16_t expected;

void setUp(void)
{
}

void tearDown(void)
{
}

static void expect_temp_c(const uint16_t s)
{
    expected = s;
}

static void given_sensor_with_value(temp_sensor_t t, uint16_t analog_value)
{
    analog_read_ExpectAndReturn(t, analog_value);
    temp_read_thermistor(t);
    int16_t result = temp_read_thermistor(t);
    TEST_ASSERT_EQUAL_INT16(expected, result);
}

void test_temp_table_lookup(void)
{
    expect_temp_c(2525);
    given_sensor_with_value(0, 0);
}

void test_zero_analog_read(void)
{
    expect_temp_c(-31);
    given_sensor_with_value(0, 1023);
}

void test_temp_sensor_tick(void)
{
    analog_read_ExpectAndReturn(0, 1023);
    analog_read_ExpectAndReturn(1, 1);
    temp_sensor_tick();
    int16_t result = temp_get(0);
    TEST_ASSERT_EQUAL_INT16(-31, result);
}