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

WGS84_err_t set_lat_label_from_char(const char c, wgs84_coordinates_t *coor) {
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

WGS84_err_t set_lon_label_from_char(const char c, wgs84_coordinates_t *coor) {
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

/**
 *
 * Public definitions
 *
 **/

float wgs84_get_M(float phi_rad) {
  // M = (1 - e^2) / (1 - e^2 * sin^2(phi))^(3/2)
  static float num = WGS84_MAJOR_SEMI_AXIS_M * (1 - WGS84_ECCENTRICITY_SQUARED);
  float sin_phi = sinf(phi_rad);
  float denom = powf(1 - WGS84_ECCENTRICITY_SQUARED * sin_phi * sin_phi, 1.5f);
  if (denom == 0) {
    return 0;  // Avoid division by zero
  }
  return num / denom;
}

float wgs84_get_M_diff(float phi_rad) {
  // $-\frac{3\,a\,e^2 \,\sin \left(2\,\phi \right)\,{\left(e^2
  // -1\right)}}{2\,{{\left(1-e^2 \,{\sin \left(\phi \right)}^2 \right)}}^{5/2}
  // }$
  static float num = (3 / 2) * WGS84_MAJOR_SEMI_AXIS_M *
                     WGS84_ECCENTRICITY_SQUARED *
                     (1 - WGS84_ECCENTRICITY_SQUARED);
  float sin_phi = sinf(phi_rad);
  float dem = powf(1 - WGS84_ECCENTRICITY_SQUARED * sin_phi * sin_phi, 2.5f);

  if (dem == 0) {
    return 0;  // Avoid division by zero
  }
  return (num * sinf(2 * phi_rad)) /
         dem;  // derivative of M with respect to phi
}

float wgs84_get_N(float phi_rad) {
  // N = a / sqrt(1 - e^2 * sin^2(phi))
  float sin_phi = sinf(phi_rad);
  float denom = sqrtf(1 - WGS84_ECCENTRICITY_SQUARED * sin_phi * sin_phi);
  if (denom == 0) {
    return 0;  // Avoid division by zero
  }
  return WGS84_MAJOR_SEMI_AXIS_M / denom;
}

float wgs84_get_N_diff(float phi_rad) {
  // $-\frac{a\,e^2 \,\sin \left(\phi \right)}{{\left(1-e^2 \,{\sin \left(\phi
  // \right)}^2 \right)}^{3/2}}$
  static float num = WGS84_MAJOR_SEMI_AXIS_M * WGS84_ECCENTRICITY_SQUARED / 2.f;
  float sin_phi = sinf(phi_rad);
  float denom = powf(1 - WGS84_ECCENTRICITY_SQUARED * sin_phi * sin_phi, 1.5f);
  if (denom == 0) {
    return 0;  // Avoid division by zero
  }
  return (num * sinf(2 * phi_rad)) / denom;
}

float wgs84_delta_lat2meters(wgs84_coordinates_t *coor, float delta_lat_deg) {
  CHECK_ARGS(coor);
  float phi_rad, lambda_rad;
  wgs84_get_full_range(coor, &phi_rad, &lambda_rad);
  phi_rad *= M_PI / 180.f;     // convert to radians
  lambda_rad *= M_PI / 180.f;  // convert to radians
  float M = wgs84_get_M(phi_rad);
  if (M == 0) {
    return 0;  // Avoid division by zero
  }
  return M * delta_lat_deg * M_PI / 180.f;  // convert to radians
}

float wgs84_delta_lon2meters(wgs84_coordinates_t *coor, float delta_lon_deg) {
  CHECK_ARGS(coor);
  float phi_rad, lambda_rad;
  wgs84_get_full_range(coor, &phi_rad, &lambda_rad);
  phi_rad *= M_PI / 180.f;     // convert to radians
  lambda_rad *= M_PI / 180.f;  // convert to radians
  float N = wgs84_get_N(phi_rad);
  if (N == 0) {
    return 0;  // Avoid division by zero
  }
  return N * cosf(phi_rad) * delta_lon_deg * M_PI /
         180.f;  // convert to radians
}

float wgs84_meters2delta_lat(wgs84_coordinates_t *coor, float meters) {
  CHECK_ARGS(coor);
  float phi_rad, lambda_rad;
  wgs84_get_full_range(coor, &phi_rad, &lambda_rad);
  phi_rad *= M_PI / 180.f;     // convert to radians
  lambda_rad *= M_PI / 180.f;  // convert to radians
  float M = wgs84_get_M(phi_rad);
  if (M == 0) {
    return 0;  // Avoid division by zero
  }
  return meters * 180.f / (M * M_PI);  // convert to degrees
}

float wgs84_meters2delta_lon(wgs84_coordinates_t *coor, float meters) {
  CHECK_ARGS(coor);
  float phi_rad, lambda_rad;
  wgs84_get_full_range(coor, &phi_rad, &lambda_rad);
  phi_rad *= M_PI / 180.f;     // convert to radians
  lambda_rad *= M_PI / 180.f;  // convert to radians
  float N = wgs84_get_N(phi_rad);
  if (N == 0) {
    return 0;  // Avoid division by zero
  }
  return meters * 180.f / (N * cosf(phi_rad) * M_PI);  // convert to degrees
}

WGS84_err_t wgs84_get_full_range(wgs84_coordinates_t *coor, float *lat,
                                 float *lon) {
  CHECK_ARGS(coor);
  CHECK_ARGS(lon);
  CHECK_ARGS(lat);

  *lat = coor->lat.dir == WGS84_NORTH ? coor->lat.dec
                                      : -coor->lat.dec;  // lat [-90,90]
  *lon = coor->lon.dir == WGS84_EAST ? coor->lon.dec
                                     : -coor->lon.dec;  // lon [-180,180]

  return WGS84_ERR_OK;
}

WGS84_err_t wgs84_degree2decimal(wgs84_coordinates_t *coor) {
  CHECK_ARGS(coor);
  coor->lat.dec = (float)coor->lat.deg + (coor->lat.min / 60.f);
  coor->lon.dec = (float)coor->lon.deg + (coor->lon.min / 60.f);
  return WGS84_ERR_OK;
}

WGS84_err_t wgs84_decimal2degree(wgs84_coordinates_t *coor) {
  CHECK_ARGS(coor);
  coor->lat.deg = (uint8_t)coor->lat.dec;
  coor->lat.min = (coor->lat.dec - (float)coor->lat.deg) * 60.f;

  coor->lon.deg = (uint8_t)coor->lon.dec;
  coor->lon.min = (coor->lon.dec - (float)coor->lon.deg) * 60.f;
  return WGS84_ERR_OK;
}

WGS84_err_t wgs84_set_dec_coor(float lon, float lat,
                               wgs84_coordinates_t *coor) {
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

  const char *gsa = strstr(nmea, "$GPGSA");
  if (gsa == NULL) {
    printf("GPGSA parsing error: %d\n", __LINE__);
    return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }

  // if (sscanf(gga, "$GPGGA,%*f,%2u%f,%c,%3u%f,%c,%*d,%*d,%*f,%f", &lat_deg,
  //            &lat_min, &lat_dir, &lon_deg, &lon_min, &lon_dir,
  //            &coor->height_AMSL) != 7) {
  //             printf("GPGSA parsing error: %d\n", __LINE__);
  //   return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  // }

  char lat_buff[16], lon_buff[16];
  float height;

  if (sscanf(gga, "$GPGGA,%*[^,],%15[^,],%c,%15[^,],%c,%*d,%*d,%*f,%f", lat_buff,
             &lat_dir, lon_buff, &lon_dir, &height) != 5) {
    printf("GPGGA parsing error: %d\n", __LINE__);
    return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }

  lat_deg = (lat_buff[0] - '0') * 10 + (lat_buff[1] - '0');
  lat_min = atof(&lat_buff[2]);

  lon_deg = (lon_buff[0] - '0') * 100 + (lon_buff[1] - '0') * 10 + (lon_buff[2] - '0');
  lon_min = atof(&lon_buff[3]);
  coor->height_AMSL = height;

  CHECK_WGS84_ERR(set_lat_label_from_char(lat_dir, coor));
  CHECK_WGS84_ERR(set_lon_label_from_char(lon_dir, coor));

  coor->lat.deg = lat_deg;
  coor->lat.min = lat_min;

  coor->lon.deg = lon_deg;
  coor->lon.min = lon_min;

  CHECK_WGS84_ERR(wgs84_degree2decimal(coor));

  // GPGSA: $GPGSA,<mode1>,<fix_mode>,<sat1>,...,<sat12>,<pdop>,<hdop>,<vdop>*hh
  if (sscanf(shift_n_commas(gsa, 16), "%f,%f", &coor->variance[0],
             &coor->variance[2]) != 2) {
    printf("GPGSA parsing error: %d\n", __LINE__);
    return WGS84_ERR_INCORRECT_NMEA_SENTENCE;
  }
  coor->variance[1] = coor->variance[0];  // GPGSA does not provide separate

  coor->variance[0] *= WGS84_UERE;
  coor->variance[1] *= WGS84_UERE;
  coor->variance[2] *= WGS84_UERE;

  // change variance from meters to degrees

  float lat, lon;

  wgs84_get_full_range(coor, &lat, &lon);

  float lat_rad = lat * M_PI / 180.f;  // convert to radians
  float lon_rad = lon * M_PI / 180.f;  // convert to radians

  // Apply the LaTeX formula for variance transformation from meters to degrees
  // coor->variance[0] = wgs84_meters2delta_lat(coor, coor->variance[0]);
  coor->variance[0] *= coor->variance[0];  // square the variance

  // coor->variance[1] = wgs84_meters2delta_lon(coor, coor->variance[1]);
  coor->variance[1] *= coor->variance[1];  // square the variance

  // variance[2] (height): just squared (meters^2)
  coor->variance[2] = coor->variance[2] * coor->variance[2];

  return WGS84_ERR_OK;
}

WGS84_err_t wgs84_ENU_delta_update(wgs84_coordinates_t *coor, float dx,
                                   float dy, float dz) {
  CHECK_ARGS(coor);
  CHECK_WGS84_ERR(wgs84_degree2decimal(coor));

  float d_lat, d_lon, d_h;
  float phi_rad;
  float lat, lon;
  wgs84_get_full_range(coor, &lat, &lon);
  phi_rad = lat * M_PI / 180.f;  // convert to radians

  float c_phi = cosf(phi_rad);
  float s_phi = sinf(phi_rad);

  d_lon = dy / (wgs84_get_N(phi_rad) * c_phi);  // d_lon in radians
  d_lat = dx / (wgs84_get_M(phi_rad));          // d_lat in radians
  d_h = dz;

  lat += d_lat * 180.f / M_PI;  // convert to degrees
  lon += d_lon * 180.f / M_PI;  // convert to degrees
  coor->height_AMSL += d_h;

  wgs84_set_dec_coor(lon, lat, coor);

  CHECK_WGS84_ERR(wgs84_decimal2degree(coor));
  return WGS84_ERR_OK;
}

WGS84_err_t wgs84_ENU_delta_update_jac(wgs84_coordinates_t *coor, float dx,
                                       float dy, float dz, float jac_coor[3][3],
                                       float jac_delta[3][3]) {
  CHECK_ARGS(coor);
  CHECK_ARGS(jac_coor);
  CHECK_ARGS(jac_delta);

  float lat, lon;

  CHECK_WGS84_ERR(wgs84_degree2decimal(coor));

  CHECK_WGS84_ERR(wgs84_get_full_range(coor, &lat, &lon));
  float phi_rad = lat * M_PI / 180.f;     // convert to radians
  float lambda_rad = lon * M_PI / 180.f;  // convert to radians

  float c_phi = cosf(phi_rad);
  float s_phi = sinf(phi_rad);
  float c_phi_2 = c_phi * c_phi;
  float s_phi_2 = s_phi * s_phi;

  float M = wgs84_get_M(phi_rad);
  float N = wgs84_get_N(phi_rad);
  float diff_M = wgs84_get_M_diff(phi_rad);
  float diff_N = wgs84_get_N_diff(phi_rad);

  // Zero division handling
  float inv_M = (M != 0.f) ? (1.f / M) : 0.f;
  float inv_N = (N != 0.f) ? (1.f / N) : 0.f;
  float inv_c_phi = (c_phi != 0.f) ? (1.f / c_phi) : 0.f;
  float inv_c_phi_2 = (c_phi_2 != 0.f) ? (1.f / c_phi_2) : 0.f;

  jac_coor[0][0] = 1.f - dx * diff_M * inv_M * inv_M;
  jac_coor[0][1] = 0.f;
  jac_coor[0][2] = 0.f;
  jac_coor[1][0] =
      dy * ((s_phi * inv_N / c_phi_2) - (diff_N * inv_N * inv_N / c_phi));
  jac_coor[1][1] = 1.f;
  jac_coor[1][2] = 0.f;
  jac_coor[2][0] = 0.f;
  jac_coor[2][1] = 0.f;
  jac_coor[2][2] = 1.f;

  jac_delta[0][0] = inv_M;
  jac_delta[0][1] = 0.f;
  jac_delta[0][2] = 0.f;
  jac_delta[1][0] = 0.f;
  jac_delta[1][1] = inv_N / c_phi;
  jac_delta[1][2] = 0.f;
  jac_delta[2][0] = 0.f;
  jac_delta[2][1] = 0.f;
  jac_delta[2][2] = 1.f;

  return WGS84_ERR_OK;
}