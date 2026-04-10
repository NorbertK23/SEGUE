#ifndef CONTROL_H_
#define CONTROL_H_

#include "APP_STATE.h"

/* Top-level APP control interface. */

uint8_t app_init(app_state_t *s);

void app_drive_off(app_state_t *s);

void app_enter_fault(app_state_t *s, const char *msg);

void app_control_step(app_state_t *s);

void app_run(app_state_t *s);

#endif /* CONTROL_H_ */
