#ifndef V1_SDCARD_H
#define V1_SDCARD_H

#include <stdbool.h>
#include <stddef.h>

// Mounts the card (FAT, never formatted). Does not start a log.
bool sdSetup(void);
bool sdMounted(void);

// Closes the open log, if any, and starts the next free log_NNN.csv with its header.
// Returns NULL on success, or a short reason for the laptop. Retries the mount first.
const char *sdStartLog(void);
// Name of the open log ("log_004.csv"), or NULL when not logging.
const char *sdLogName(void);
// Appends one row to the open log (no-op when none) and syncs about once a second, so a
// power cut loses at most about a second. Returns false on a write/sync error, which also
// closes the log; a new sdStartLog() can try again.
bool sdLogLine(const char *line);

// Writes text to the next free cal_NNN.txt; name gets the file name.
// Returns NULL on success, or a short reason for the laptop.
const char *sdWriteCalibration(const char *text, char *name, size_t nameLen);

#endif /* V1_SDCARD_H */
