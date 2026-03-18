// screen_config_c_api.h - C-friendly API for ScreenConfig
#ifndef SCREEN_CONFIG_C_API_H
#define SCREEN_CONFIG_C_API_H

#include "calibration_types.h" // For GaugeCalibrationPoint
#include <stdint.h>

// Avoid including C++-heavy headers here (UI C files include this header).
// If `NUM_SCREENS` is not defined by the build, default to 5.
#ifndef NUM_SCREENS
#define NUM_SCREENS 5
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Display type enumeration
typedef enum {
	DISPLAY_TYPE_GAUGE = 0,        // Traditional gauge with needles
	DISPLAY_TYPE_GAUGE_NUMBER = 4  // Gauge (top needle) + Number (center readout)
} DisplayType;

// Number display font size
typedef enum {
	NUMBER_FONT_SMALL = 0,     // 48pt
	NUMBER_FONT_MEDIUM = 1,    // 72pt
	NUMBER_FONT_LARGE = 2,     // 96pt
	NUMBER_FONT_XLARGE = 3,    // 120pt
	NUMBER_FONT_XXLARGE = 4    // 144pt
} NumberFontSize;

typedef struct {
	GaugeCalibrationPoint cal[2][5]; // 2 gauges, 5 points each
	char icon_paths[2][128];         // 2 icons (top/bottom)
	uint8_t show_bottom;             // 1 = show bottom gauge/icon, 0 = hide (single gauge mode)
	char background_path[128];      // optional per-screen background image path
	uint8_t icon_pos[2];            // icon position per icon: 0=top,1=right,2=bottom,3=left
	float min[2][5];                 // min for each zone
	float max[2][5];                 // max for each zone
	char color[2][5][8];             // color hex for each zone
	int transparent[2][5];           // transparency for each zone
	int buzzer[2][5];                // buzzer enabled for each zone
	uint8_t display_type;            // DisplayType: 0=Gauge, 4=Gauge+Number
	// Gauge+Number display settings (top gauge + center number)
	char gauge_num_center_path[128]; // Signal K path for center number
	uint8_t gauge_num_center_font_size; // NumberFontSize
	char gauge_num_center_font_color[8]; // Hex color for center number
} __attribute__((packed)) ScreenConfig;

// Expose the screen_configs array to C files
extern ScreenConfig screen_configs[NUM_SCREENS];

#ifdef __cplusplus
}
#endif

#endif // SCREEN_CONFIG_C_API_H
