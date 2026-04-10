#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "APP_STATE.h"

/* Boot parameter dump and runtime telemetry print API. */

void app_print_current_params(void);

void app_print_telemetry_columns(void);

void app_print_telemetry(const app_state_t *s);

#endif /* TELEMETRY_H_ */
