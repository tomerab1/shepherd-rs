// C interface to the Shepherd routing engine (see ffi/src/lib.rs).
// Strings returned as `char *` are JSON; free them with shepherd_string_free.
#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct ShepherdEngine ShepherdEngine;

ShepherdEngine *shepherd_open(const char *graph_path, const char *avoid_area_name,
                              const char *avoid_area_geojson, char **error_out);
void shepherd_close(ShepherdEngine *e);
void shepherd_string_free(char *s);

char *shepherd_info(const ShepherdEngine *e);
/// lonlat = [lon0, lat0, lon1, lat1, ...] for n points.
char *shepherd_route(const ShepherdEngine *e, const double *lonlat, size_t n, const char *exclude);
/// 0 = estimate, 1 = timed-only, 2 = off. Returns the new metric version.
uint64_t shepherd_set_signal_mode(const ShepherdEngine *e, uint8_t mode);
char *shepherd_lights(const ShepherdEngine *e, double west, double south, double east, double north);
