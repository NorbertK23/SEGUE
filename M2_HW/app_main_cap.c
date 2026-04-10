#include <avr/io.h>
#include <avr/interrupt.h>

#include "SETTINGS.h"
#include "CONTROL.h"

/* Minimal APP entry point. Startup and runtime policy live in CONTROL.c. */
int main(void)
{
    /* Static storage keeps the full APP state off the stack. */
    static app_state_t app;

    app_state_reset(&app);

    if (!app_init(&app)) {
        app_enter_fault(&app, "init failed");
    }

    app_run(&app);

    return 0;
}
