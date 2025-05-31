#include "WGS84.h"
#include "WGS84_params.h"
#include "unity.h"
#include "unity_internals.h"

void test_wgs84_from_string(void) {
  const char *nmea = "$GPGGA,123456.78,1234.56789,N,12345.67890,E,1,12,0.5,100.0,M,0.0,M,,*47,$GPGSA,A,3,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,,0.5,0.5,0.5*3A";
  wgs84_coordinates_t coor = {0};
  WGS84_err_t err = wgs84_from_string(nmea, &coor);
  TEST_ASSERT_EQUAL(WGS84_ERR_OK, err);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, coor.height_AMSL);
  TEST_ASSERT_EQUAL_UINT8(12, coor.lat.deg);
  TEST_ASSERT_EQUAL_FLOAT(34.56789f, coor.lat.min);
  TEST_ASSERT_EQUAL_FLOAT(12.57615f, coor.lat.dec);
  TEST_ASSERT_EQUAL_UINT8(123, coor.lon.deg);
  TEST_ASSERT_EQUAL_FLOAT(45.67890f, coor.lon.min);
  TEST_ASSERT_EQUAL_FLOAT(123.76132f, coor.lon.dec);
  TEST_ASSERT_EQUAL_FLOAT(0.5f * WGS84_UERE, coor.variance[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.5f * WGS84_UERE, coor.variance[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.5f * WGS84_UERE, coor.variance[2]);
}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_wgs84_from_string);

  UNITY_END();
}

void setUp(void) {}
void tearDown(void) {}