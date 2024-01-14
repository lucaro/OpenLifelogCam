#ifndef __CONFIG_H__
#define __CONFIG_H__

// Select camera board model
// Currently only the ESP32-CAM board from AI-Thinker is supported
#define CAMERA_MODEL_AI_THINKER

// Enable deep-sleep in between pictures
#define WITH_SLEEP

// Enable GNSS(GPS/Glonass/Gallileo/etc...) support
#define WITH_GNSS


// Version number of firmware
#define VERSION_STR "0.1"

#endif // __CONFIG_H__
