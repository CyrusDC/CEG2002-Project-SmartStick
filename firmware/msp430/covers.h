#ifndef SMARTSTICK_COVERS_H
#define SMARTSTICK_COVERS_H

#include <stdbool.h>

extern volatile bool cover_open;  /* Track whether the servo covers are open (C). */

void covers_init(void);
void covers_suspend(void);
void covers_resume(void);
void openCover(void);
void closeCover(void);

#endif
