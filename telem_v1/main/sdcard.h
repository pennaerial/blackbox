#ifndef V1_SDCARD_H
#define V1_SDCARD_H

bool sdSetup(void);
// Call only after sdSetup() succeeds.
void sdLogLine(const char *line);

#endif /* V1_SDCARD_H */
