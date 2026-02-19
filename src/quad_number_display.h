#ifndef QUAD_NUMBER_DISPLAY_H
#define QUAD_NUMBER_DISPLAY_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Create quad number display for a screen (4 quadrants: TL, TR, BL, BR)
// Reads all settings from screen_configs[screen_num]
void quad_number_display_create(int screen_num);

// Destroy quad number display for a screen
void quad_number_display_destroy(int screen_num);

// Update quadrant values
void quad_number_display_update_tl(int screen_num, float value, const char* unit, const char* description);
void quad_number_display_update_tr(int screen_num, float value, const char* unit, const char* description);
void quad_number_display_update_bl(int screen_num, float value, const char* unit, const char* description);
void quad_number_display_update_br(int screen_num, float value, const char* unit, const char* description);

#ifdef __cplusplus
}
#endif

#endif // QUAD_NUMBER_DISPLAY_H
