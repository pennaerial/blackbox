#ifndef V1_WIRELESS_H
#define V1_WIRELESS_H

#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define WIRELESS_SIK_DEFAULT_BAUD 57600
#define WIRELESS_MAX_LINE_LENGTH 219  // Payload bytes, excluding the appended '\n'.

typedef struct {
    uart_port_t uart_port;  // A free HP UART, separate from the ESP-IDF console.
    gpio_num_t tx_pin;      // ESP32 TX -> radio RX. Select for the actual board.
    gpio_num_t rx_pin;      // ESP32 RX <- radio TX, or GPIO_NUM_NC for TX only.
    int baud_rate;         // Match the radio's SERIAL_SPEED, not its AIR_SPEED.
} wireless_config_t;

// Holybro SiK transparent serial data mode: 8N1, RTS/CTS disabled on both ends.
// Configure/pair the radios separately; this module does not send AT commands,
// change RF settings, or verify that a remote radio is receiving the data.
//
// Call once from one task, before using the other functions. Owns the selected
// UART and creates a background TX task for the lifetime of the application.
// Rejects invalid pins, the configured console UART, and an installed driver.
// Failed setup releases its driver/queue so it can be retried.
// Integration later requires wireless.c in SRCS and esp_driver_uart in REQUIRES
// (or PRIV_REQUIRES). No GPIO assignment is assumed here.
esp_err_t wirelessSetup(const wireless_config_t *config);

// Copy one nonempty CSV/text line without CR/LF into a one-slot pending queue.
// Adds '\n' on the wire. The caller may reuse its buffer immediately afterwards.
// A new line replaces any pending line; the line currently transmitting finishes.
// Safe for multiple sending tasks after setup; task context only, not an ISR.
// ESP_OK means queued, not delivered over RF. INVALID_STATE means not set up;
// INVALID_ARG means null/empty/embedded CR or LF; INVALID_SIZE means too long.
esp_err_t wirelessSendLine(const char *line);

// Optional raw receive, for one reader task after successful setup with RX wired.
// Returns byte count (zero on timeout), or -1 for invalid arguments/state or a
// driver error. Data is not NUL-terminated and may contain only part of a line.
// timeout_ticks is in FreeRTOS ticks: use 0 to poll or pdMS_TO_TICKS(ms) to wait.
int wirelessRead(uint8_t *buffer, size_t capacity, TickType_t timeout_ticks);

#endif /* V1_WIRELESS_H */
