#include "WGS84.h"

#include "WGS84_params.h"
#define __USE_GNU
#include "math.h"
#include "stdio.h"
#include "string.h"

#define CHECK_ARGS(ptr)                                                     \
  do {                                                                      \
    if ((ptr) == NULL) {                                                    \
      fprintf(stderr, "Null pointer detected: %s (in %s at %s:%d)\n", #ptr, \
              __func__, __FILE__, __LINE__);                                \
      return WGS84_ERR_BAD_ARGS;                                            \
    }                                                                       \
  } while (0)

#define CHECK_WGS84_ERR(err)                                              \
  do {                                                                    \
    if ((err) != WGS84_ERR_OK) {                                          \
      fprintf(stderr, "WGS84 ERR (in %s at %s:%d)\n", __func__, __FILE__, \
              __LINE__);                                                  \
      return err;                                                         \
    }                                                                     \
  } while (0)

/**
 *
 * Private definitions
 *
 **/

const char *shift_n_commas(const char *str, int n) {
  const char *p = str;
  while (n-- > 0 && *p != '\0') {
    p = strchr(p + 1, ',');
    if (p == NULL) {
      return NULL;  // Not enough commas
    }
  }
  return p + 1;
}

WGS84_err_t get_lat_label_from_char(const char c, wgs84_coordinates_t *coor) {
  CHECK_ARGS(coor);
  switch (c) {
    case 'N':
      coor->lat.dir = WGS84_NORTH;
      return WGS84_ERR_OK;

    case 'S':
      coor->lat.dir = WGS84_SOUTH;
      return WGS84_ERR_OK;
    default:
      return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }
}

WGS84_err_t get_lon_label_from_char(const char c, wgs84_coordinates_t *coor) {
  CHECK_ARGS(coor);
  switch (c) {
    case 'W':
      coor->lon.dir = WGS84_WEST;
      return WGS84_ERR_OK;

    case 'E':
      coor->lon.dir = WGS84_EAST;
      return WGS84_ERR_OK;
    default:
      return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }
}

WGS84_err_t degree2decimal(wgs84_coordinates_t *coor) {
  CHECK_ARGS(coor);
  coor->lat.dec = (float)coor->lat.deg + (coor->lat.min / 60.f);
  coor->lon.dec = (float)coor->lon.deg + (coor->lon.min / 60.f);
  return WGS84_ERR_OK;
}

WGS84_err_t decimal2degree(wgs84_coordinates_t *coor) {
  CHECK_ARGS(coor);
  coor->lat.deg = (uint8_t)coor->lat.dec;
  coor->lat.min = (coor->lat.dec - (float)coor->lat.deg) * 60.f;

  coor->lon.deg = (uint8_t)coor->lon.dec;
  coor->lon.min = (coor->lon.dec - (float)coor->lon.deg) * 60.f;
  return WGS84_ERR_OK;
}

WGS84_err_t set_dec_coor(float lon, float lat, wgs84_coordinates_t *coor) {
  if (fabs(lon) > 180.f) {
    lon = (lon > 0) ? fmodf(lon, -180) : fmodf(lon, 180);
  }

  if (fabs(lat) > 90.f) {
    lat = (lat > 0) ? fmodf(lat, -90) : fmodf(lat, 90);
  }

  coor->lat.dec = fabs(lat);
  coor->lat.dir = lat > 0 ? WGS84_NORTH : WGS84_SOUTH;

  coor->lon.dec = fabs(lon);
  coor->lon.dir = lat > 0 ? WGS84_EAST : WGS84_WEST;

  return WGS84_ERR_OK;
}

/**
 *
 * Public definitions
 *
 **/

WGS84_err_t wgs84_from_string(const char *nmea, wgs84_coordinates_t *coor) {
  CHECK_ARGS(coor);
  CHECK_ARGS(nmea);

  unsigned int lon_deg, lat_deg;
  float lon_min, lat_min;
  char lat_dir, lon_dir;

  const char *gga = strstr(nmea, "$GPGGA");
  if (gga == NULL) {
    return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }

  if (sscanf(gga, "$GPGGA,%*f,%2u%f,%c,%3u%f,%c,%*d,%*d,%*f,%f", &lat_deg, &lat_min, &lat_dir, &lon_deg, &lon_min, &lon_dir, &coor->height_AMSL) != 7) {
    return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }

  CHECK_WGS84_ERR(get_lat_label_from_char(lat_dir, coor));
  CHECK_WGS84_ERR(get_lon_label_from_char(lon_dir, coor));

    coor->lat.deg = lat_deg;
    coor->lat.min = lat_min;

    coor->lon.deg = lon_deg;
    coor->lon.min = lon_min;

  CHECK_WGS84_ERR(degree2decimal(coor));

  const char *gsa = strstr(nmea, "$GPGSA");
  if (gsa == NULL) {
    return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }

  float pdop;
  int fix_mode;
  // GPGSA: $GPGSA,<mode1>,<fix_mode>,<sat1>,...,<sat12>,<pdop>,<hdop>,<vdop>*hh
  if (sscanf(gsa, "$GPGSA,%*c,%d,%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%f,%f,%f",
             &fix_mode, &pdop, &coor->variance[0], &coor->variance[2]) != 4) {
    return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }
  coor->variance[1] = coor->variance[0];  // GPGSA does not provide separate

  coor->variance[0] *= coor->variance[0]*WGS84_UERE;
  coor->variance[1] *= coor->variance[1]*WGS84_UERE;
  coor->variance[2] *= coor->variance[2]*WGS84_UERE;

  return WGS84_ERR_OK;
}

WGS84_err_t wgs84_ENU_delta_update(wgs84_coordinates_t *coor, float dx,
                                   float dy, float dz) {
  CHECK_ARGS(coor);
  CHECK_WGS84_ERR(degree2decimal(coor));

  float d_lat, d_lon, d_h;
  float phi_rad, phi_rad_2, phi_rad_3, phi_rad_4, phi_rad_5, phi_rad_6;
  float lat = coor->lat.dir == WGS84_NORTH ? coor->lat.dec
                                           : -coor->lat.dec;  // lat [-90,90]
  float lon = coor->lon.dir == WGS84_EAST ? coor->lon.dec
                                          : -coor->lon.dec;  // lon [-180,180]

  phi_rad = lat * M_PI / 180.f;  // convert to radians
  float c_phi = cosf(phi_rad);
  float s_phi = sinf(phi_rad);
  float c_phi_2 = c_phi * c_phi;
  float s_phi_2 = s_phi * s_phi;
  float inv_c_phi_2 = 1.f / c_phi_2;
  float inv_c_phi = 1.f / c_phi;

  float v1 = 1 - 0.006694f * s_phi_2;

  d_lon = 1.5678e-7f * dy * sqrtf(v1) * inv_c_phi;
  d_lat = 1.57842e-7f * dx * powf(v1, 1.5f);
  d_h = dz;

  lat += d_lat * 180.f / M_PI;  // convert to degrees
  lon += d_lon * 180.f / M_PI;  // convert to degrees
  coor->height_AMSL += d_h;

  set_dec_coor(lon, lat, coor);

  CHECK_WGS84_ERR(decimal2degree(coor));
  return WGS84_ERR_OK;
}

/** jac_delta
 $\left(\begin{array}{ccc}
\text{1.57842e-7}\,{{\left(1.0-0.00669438\,{\sin \left(\phi \right)}^2 \right)}}^{3/2}  & 0 & 0\\
0 & \frac{\text{1.56786e-7}\,\sqrt{1.0-0.00669438\,{\sin \left(\phi \right)}^2 }}{\cos \left(\phi \right)} & 0\\
0 & 0 & 1.0
\end{array}\right)$
 */

/** jac_coor
$\left(\begin{array}{ccc}
1.0-\text{3.16997e-9}\,\mathrm{dx}\,\cos \left(\phi \right)\,\sin \left(\phi \right)\,\sqrt{1.0-0.00669438\,{\sin \left(\phi \right)}^2 } & 0 & 0\\
\frac{\text{1.56786e-7}\,\mathrm{dy}\,\sin \left(\phi \right)\,\sqrt{1.0-0.00669438\,{\sin \left(\phi \right)}^2 }}{{\cos \left(\phi \right)}^2 }-\frac{\text{1.04958e-9}\,\mathrm{dy}\,\sin \left(\phi \right)}{\sqrt{1.0-0.00669438\,{\sin \left(\phi \right)}^2 }} & 1.0 & 0\\
0 & 0 & 1.0
\end{array}\right)$
  */

WGS84_err_t wgs84_ENU_delta_update_jac(wgs84_coordinates_t *coor, float dx,
                                       float dy, float dz, float *jac_coor,
                                       float *jac_delta) {
  CHECK_ARGS(coor);
  CHECK_ARGS(jac_coor);
  CHECK_ARGS(jac_delta);

  float lat = coor->lat.dir == WGS84_NORTH ? coor->lat.dec
                                           : -coor->lat.dec;  // lat [-90,90]
  float lon = coor->lon.dir == WGS84_EAST ? coor->lon.dec
                                          : -coor->lon.dec;  // lon [-180,180]

  float phi_rad = lat * M_PI / 180.f;  // convert to radians
  float s_phi = sinf(phi_rad);
  float c_phi = cosf(phi_rad);
  float s_phi_2 = s_phi * s_phi;
  float c_phi_2 = c_phi * c_phi;

  float v1 = 1.0f - 0.00669438f * s_phi_2;
  float sqrt_v1 = sqrtf(v1);
  float pow_v1_1_5 = powf(v1, 1.5f);

  // Fill jac_delta (row-major, 3x3)
  jac_delta[0] = 1.57842e-7f * pow_v1_1_5;
  jac_delta[1] = 0.0f;
  jac_delta[2] = 0.0f;
  jac_delta[3] = 0.0f;
  jac_delta[4] = 1.56786e-7f * sqrt_v1 / c_phi;
  jac_delta[5] = 0.0f;
  jac_delta[6] = 0.0f;
  jac_delta[7] = 0.0f;
  jac_delta[8] = 1.0f;

  // Fill jac_coor (row-major, 3x3)
  jac_coor[0] = 1.0f - 3.16997e-9f * dx * c_phi * s_phi * sqrt_v1;
  jac_coor[1] = 0.0f;
  jac_coor[2] = 0.0f;
  jac_coor[3] = 1.56786e-7f * dy * s_phi * sqrt_v1 / c_phi_2
                - 1.04958e-9f * dy * s_phi / sqrt_v1;
  jac_coor[4] = 1.0f;
  jac_coor[5] = 0.0f;
  jac_coor[6] = 0.0f;
  jac_coor[7] = 0.0f;
  jac_coor[8] = 1.0f;

  return WGS84_ERR_OK;
}