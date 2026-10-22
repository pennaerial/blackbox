#ifndef V1_SDCARD_H
#define V1_SDCARD_H

#include <stdbool.h>

bool sdSetup(void);
// Returns false on a write/sync error; the caller should stop logging to SD.
bool sdLogLine(const char *line);

#endif /* V1_SDCARD_H */
