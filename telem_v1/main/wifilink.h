#ifndef V1_WIFILINK_H
#define V1_WIFILINK_H

#include <stdbool.h>

// The logger's own WiFi network (access point) for setting it up at the field.
// Join this network on the laptop, then from ground_station/ run `uv run dashboard.py --wifi`,
// which connects to TCP port WIFI_LINK_PORT at 192.168.4.1 (the access point's address).
// The password needs at least 8 characters. Change both here, and the SSID in dashboard.py.
#define WIFI_LINK_SSID "blackbox"
#define WIFI_LINK_PASSWORD "pennair-blackbox"
#define WIFI_LINK_CHANNEL 6
#define WIFI_LINK_PORT 3333

// Laptop -> logger, one text line each:
//   LOG              close the open log, if any, and start the next log_NNN.csv
//   CAL,LEVEL        average the IMU with the aircraft held level and still
//   CAL,PITCH        average the IMU with the nose raised and still
//   CAL,SAVE,x,y,z   write both poses and the IMU position from the CG (cm) to cal_NNN.txt
//   WIFI,OFF         turn WiFi off until the next reset (handled here, not by the logger loop)
// Logger -> laptop: every CSV row, plus lines starting with '#' (status, replies, errors).
typedef enum {
    WIFI_CMD_LOG,
    WIFI_CMD_CAL_LEVEL,
    WIFI_CMD_CAL_PITCH,
    WIFI_CMD_CAL_SAVE,
} wifi_cmd_type_t;

typedef struct {
    wifi_cmd_type_t type;
    float value[3];     // WIFI_CMD_CAL_SAVE: IMU position from the CG, cm (x fwd, y left, z up)
} wifi_cmd_t;

// Starts the access point and a background task that serves one laptop at a time. A new
// connection replaces the old one. If the laptop drops off, WiFi stays on and waits for it to
// reconnect; only the WIFI,OFF command turns it off. Returns false if WiFi could not start.
bool wifiLinkStart(void);

// Queues one line (no newline) for the laptop. Never blocks: the line is dropped when no
// laptop is connected or the queue is full. Call from the logger task only.
void wifiLinkSendLine(const char *line);

// Takes the next command from the laptop, if there is one. Never blocks.
bool wifiLinkGetCommand(wifi_cmd_t *cmd);

#endif /* V1_WIFILINK_H */
