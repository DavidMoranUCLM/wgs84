#ifndef WGS84_H
#define WGS84_H

#include "inttypes.h"

typedef enum lattitude_e { WGS84_NORTH, WGS84_SOUTH } lattitude_t;

typedef enum longitude_e { WGS84_EAST, WGS84_WEST } longitude_t;

typedef enum WGS84_err_e{
  WGS84_ERR_OK,
  WGS84_ERR_INCORRECT_NMEA_SENTENCE,
  WGS84_ERR_BAD_ARGS,

}WGS84_err_t;

typedef struct wgs84_coordinates_s {
  float height_AMSL;
  struct {
    uint8_t deg;
    float min;
    float dec;
    lattitude_t dir;
  } lat;
  struct{
    uint8_t deg;
    float min;
    float dec;
    longitude_t dir;
  } lon;
  float variance[3]; // (lat, lon, height)
} wgs84_coordinates_t;

float wgs84_delta_lat2meters(wgs84_coordinates_t *coor, float delta_lat_deg);
float wgs84_delta_lon2meters(wgs84_coordinates_t *coor, float delta_lon_deg);
float wgs84_meters2delta_lat(wgs84_coordinates_t *coor, float delta_meters);
float wgs84_meters2delta_lon(wgs84_coordinates_t *coor, float delta_meters);

float wgs84_get_N(float phi_rad);
float wgs84_get_M(float phi_rad);
float wgs84_get_N_diff(float phi_rad);
float wgs84_get_M_diff(float phi_rad);


WGS84_err_t wgs84_get_full_range(wgs84_coordinates_t *coor, float *lat, float *lon);
WGS84_err_t wgs84_degree2decimal(wgs84_coordinates_t *coor);
WGS84_err_t wgs84_decimal2degree(wgs84_coordinates_t *coor);
WGS84_err_t wgs84_set_dec_coor(float lon, float lat, wgs84_coordinates_t *coor);

WGS84_err_t wgs84_from_string(const char *nmea, wgs84_coordinates_t *coor);
WGS84_err_t wgs84_ENU_delta_update(wgs84_coordinates_t *coor, float dx, float dy, float dz);

WGS84_err_t wgs84_ENU_delta_update_jac(wgs84_coordinates_t *coor, float dx, float dy, float dz, float jac_coor[3][3],
                                       float jac_delta[3][3]);
#endif