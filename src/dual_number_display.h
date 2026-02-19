#ifndef DUAL_NUMBER_DISPLAY_H
#define DUAL_NUMBER_DISPLAY_H

#include <lvgl.h>

// Create dual number displays for a screen (top and bottom halves)
// Reads all settings from screen_configs[screen_num]
void dual_number_display_create(int screen_num);

// Update dual number displays with new values, units, and descriptions
void dual_number_display_update_top(int screen_num, float value, const char* unit, const char* description);
void dual_number_display_update_bottom(int screen_num, float value, const char* unit, const char* description);

// Destroy dual number displays for a screen
void dual_number_display_destroy(int screen_num);

#endif // DUAL_NUMBER_DISPLAY_H
