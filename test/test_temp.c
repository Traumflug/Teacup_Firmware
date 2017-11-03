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

static void expect(const float s)
{
    expected = (uint16_t)(s * 4);
}

static void given(temp_sensor_t t, uint16_t analog_value)
{
    analog_read_ExpectAndReturn(t, analog_value);
    temp_read_thermistor(t);
    uint16_t result = temp_read_thermistor(t);
    TEST_ASSERT_EQUAL_UINT16(expected, result);
}

void test_temp_table_lookup(void)
{
    expect(631.25);
    given(0, 0);
}

void test_zero_analog_read(void)
{
    expect(0);
    given(0, 1023);
}