/**
 * ESP32-CAM_Interval.ino - Capture pictures to SD card at set interval.
 *
 * Based on the code from https://robotzero.one/time-lapse-esp32-cameras/
 * Original Copyright: Copyright (c) 2019, Robot Zero One
 *
 * Copyright (c) 2019, David Imhoff <dimhoff.devel@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "config.h"

#include "Arduino.h"

#include "esp_camera.h"
#include <esp_wifi.h>
#include <esp_bt.h>

#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

// GPIO (rtc_gpio_hold_en())
#include "driver/rtc_io.h"

// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include <dirent.h>

// GNSS
#include <MicroNMEA.h>


#include "io_defs.h"
#include "camera.h"
#include "configuration.h"
#include "exif.h"

// Minimum sleep time.
// If next capture is less then this many micro seconds away, then stay awake.
#define MIN_SLEEP_TIME (10000000L)
// Wake-up this many micro seconds before capture time to allow initialization.
#define WAKE_USEC_EARLY (1500000L)

#define GPS_NMEA_TIMEOUT_MS (5L * 1000L)
#define GPS_DATETIME_TIMEOUT_MS (120L * 1000L)

// Minimal unix time for clock to be considered valid.
#define NOT_BEFORE_TIME 1704063600

// Timelapse directory name format: /sdcard/logXXXX/
#define CAPTURE_DIR_PREFIX "log"
#define CAPTURE_DIR_PREFIX_LEN 3

// RTC memory storage
RTC_DATA_ATTR struct {
  struct timeval next_capture_time;
  bool retry = false;
  int dir_idx = 0;
} nv_data;

// Globals
static char capture_path[8 + CAPTURE_DIR_PREFIX_LEN + 4 + 1];
static struct timeval capture_interval_tv;
static struct timeval next_capture_time;
char nmeaBuffer[255];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));


/************************ Initialization ************************/
void setup() {
  bool time_set = false;
  bool is_wakeup = false;

  //Set clock frequency to 20MHz
  //setCpuFrequencyMhz(40);

  //Make sure radios are disabled 
  btStop();
  esp_wifi_stop();
  esp_bt_controller_disable();

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED) {
    if(nv_data.retry) {
      nv_data.retry = false;
    } else {
      is_wakeup = true;
    }
  }

  Serial.begin(9600);
  Serial.println();

  // Configure red LED
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, HIGH);

  // Init SD Card
  if (!init_sdcard(!is_wakeup)) {
    goto fail;
  }



  // TODO: error log to file?

  // Load config file
  if (!cfg.loadConfig(!is_wakeup)) {
    Serial.println("Error loading config\n");
    goto fail;
  }
  update_exif_from_cfg(cfg);
  capture_interval_tv.tv_sec = cfg.getCaptureInterval() / 1000;
  capture_interval_tv.tv_usec = (cfg.getCaptureInterval() % 1000) * 1000;

  // Get current Time
  time_set = init_time_gnss();

  if (time_set) {
    time_t now = time(NULL);
    Serial.printf("Current time: %s", ctime(&now));
  } else {
    Serial.println("Failed to determine current date/time\n");
  }

  // Inititialize next capture time
  if (is_wakeup) {
    next_capture_time = nv_data.next_capture_time;
  } else {
    (void)gettimeofday(&next_capture_time, NULL);
  }
  Serial.printf("Next image at: %s", ctime(&next_capture_time.tv_sec));

  // Initialize capture directory
  if (!init_capture_dir(is_wakeup)) {
    goto fail;
  }

  // camera init
  if (!camera_init()) {
    goto fail;
  }

  Serial.println("--- Initialization Done ---");

  return;

fail:
  // sleep for 5 minutes, try again
  nv_data.retry = true;
  esp_sleep_enable_timer_wakeup(300000000L);
  esp_deep_sleep_start();
}

/**
 * Get time from GNSS
 *
 * @returns True on succes, false on failure
 */
bool init_time_gnss() {
  Serial.print("Waiting for time from GNSS: ");

  long start_millis;

  // Wait for some NMEA to be recognized
  start_millis = millis();
  while (millis() - start_millis < GPS_NMEA_TIMEOUT_MS) {
    while (Serial.available()) {
      char c = Serial.read();
      nmea.process(c);
    }

    // check if there is atleast some nmea
    if (nmea.getNavSystem()) {
      break;
    }
  }
  if (!nmea.getNavSystem()) {
    Serial.println("Failed, No NMEA data received");
    // TODO: maybe preserve this information across deep sleeps, so that GNSS
    // isn't tryed after deep sleep if no GNSS receiver is available.
    return false;
  }

  // Wait for a valid date/time
  bool have_time = false;
  start_millis = millis();
  while (millis() - start_millis < GPS_DATETIME_TIMEOUT_MS) {
    if (nmea.getYear() > 2018 && nmea.getYear() < 2038) {
      // NOTE: MTK33xx GPS starts at 2018, so also have an upper limit. By 2038
      // the time_t will probably overflow... so use that as limit.
      have_time = true;
      break;
    }

    while (Serial.available()) {
      char c = Serial.read();
      nmea.process(c);
    }
  }

  if (!have_time) {
    Serial.println("Failed, unable to get valid date from GPS");
    return false;
  }

  // Construct time
  struct tm tm;
  tm.tm_year = nmea.getYear() - 1900;
  tm.tm_mon = nmea.getMonth() - 1;
  tm.tm_mday = nmea.getDay();
  tm.tm_hour = nmea.getHour();
  tm.tm_min = nmea.getMinute();
  tm.tm_sec = nmea.getSecond();
  time_t t = mktime(&tm);
  if (t == (time_t)-1) {
    Serial.println("Failed, time could not be converted");
    return false;
  }

  // Set time
  struct timeval now = { t, 0 };
  if (settimeofday(&now, NULL) != 0) {
    Serial.println("Failed, unable to set time");
    return false;
  }

  setenv("TZ", cfg.getTzInfo(), 1);
  tzset();

  Serial.println("Done");

  Serial.print("Lat: ");
  Serial.print(nmea.getLatitude());
  Serial.print(" Lon: ");
  Serial.print(nmea.getLongitude());
  Serial.println();

  return true;
}


/**
 * Mount SD Card
 */
static bool init_sdcard(bool print) {
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 1,
  };
  sdmmc_card_t *card;


  if (print) {
    Serial.print("Mounting SD card... ");
  }
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config,
                                &card);
  if (ret == ESP_OK) {
    if (print) {
      Serial.println("Done");
    }
  } else {
    if (print) {
      Serial.println("FAILED");
    }
    //error is always printed
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s\n", esp_err_to_name(ret));
    return false;
  }

  if (print) {
    Serial.printf("Card size: %lluMB\n", ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
  }

  return true;
}


/**
 * Create new directory to store images
 */
static bool init_capture_dir(bool reuse_last_dir) {
  DIR *dirp;
  struct dirent *dp;
  int dir_idx = nv_data.dir_idx;

  // Find unused directory name
  if ((dirp = opendir("/sdcard/")) == NULL) {
    Serial.println("couldn't open directory /sdcard/");
    return -1;
  }

  if(dir_idx == 0){
    do {
      errno = 0;
      if ((dp = readdir(dirp)) != NULL) {
        if (strlen(dp->d_name) != CAPTURE_DIR_PREFIX_LEN + 4)
          continue;
        if (strncmp(dp->d_name, CAPTURE_DIR_PREFIX,
                    CAPTURE_DIR_PREFIX_LEN)
            != 0)
          continue;

        char *endp;
        int idx = strtoul(&(dp->d_name[CAPTURE_DIR_PREFIX_LEN]), &endp, 10);
        if (*endp != '\0')
          continue;

        if (idx > dir_idx) {
          dir_idx = idx;
        }
      }
    } while (dp != NULL);
  }

  (void)closedir(dirp);

  if (errno != 0) {
    Serial.println("Error reading directory /sdcard/");
    return false;
  }

  if (!reuse_last_dir) {
    dir_idx += 1;
  }

  nv_data.dir_idx = dir_idx;

  // Create new dir
  snprintf(capture_path, sizeof(capture_path),
           "/sdcard/" CAPTURE_DIR_PREFIX "%04u", dir_idx);

  if (!reuse_last_dir) {
    if (mkdir(capture_path, 0644) != 0) {
      Serial.print("Failed to create directory: ");
      Serial.println(capture_path);
      return false;
    }
  }

  Serial.print("Storing pictures in: ");
  Serial.println(capture_path);

  return true;
}

/************************ Main ************************/

/**
 * Take picture and save to SD card
 */
static void save_photo() {
  camera_fb_t *fb;

  if (cfg.getEnableBusyLed()) {
    digitalWrite(LED_GPIO_NUM, LOW);
  }

  // Capture image
  fb = camera_capture();

  // Generate filename
  time_t now = time(NULL);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  char filename[sizeof(capture_path) + 15 + 4 + 1];
  size_t capture_path_len = strlen(capture_path);
  strcpy(filename, capture_path);
  strftime(&filename[capture_path_len], sizeof(filename) - capture_path_len,
           "/%Y%m%d_%H%M%S.jpg", &timeinfo);

  // Generate Exif header
  const uint8_t *exif_header = NULL;
  size_t exif_len = 0;
  update_exif_gps(nmea);
  get_exif_header(fb, &exif_header, &exif_len);

  size_t data_offset = get_jpeg_data_offset(fb);

  for(uint8_t i = 0; i < 3; ++i) {
    // Save picture
    FILE *file = fopen(filename, "w");
    if (file != NULL) {
      size_t ret = 0;
      if (exif_header != NULL) {
        ret = fwrite(exif_header, exif_len, 1, file);
        if (ret != 1) {
          Serial.println("Failed\nError while writing header to file");
          data_offset = 0;
        }
      } else {
        data_offset = 0;
      }

      ret = fwrite(&fb->buf[data_offset], fb->len - data_offset, 1, file);
      if (ret != 1) {
        Serial.println("Failed\nError while writing to file");
      } else {
        Serial.printf("Saved as %s\n", filename);
      }
      fclose(file);
      break;
    } else {
      Serial.printf("Failed\nCould not open file: %s\n", filename);
      delay(500);
    }
  }

  camera_fb_return(fb);

  if (cfg.getEnableBusyLed()) {
    digitalWrite(LED_GPIO_NUM, HIGH);
  }
}

void loop() {
  // Take picture if interval passed
  // NOTE: This breaks if clock jumps are introduced. Make sure to use
  // adjtime().
  struct timeval now;
  (void)gettimeofday(&now, NULL);
  if (!timercmp(&now, &next_capture_time, <)) {
    save_photo();

    timeradd(&next_capture_time, &capture_interval_tv, &next_capture_time);
  }

  // Process Serial input
  while (Serial.available()) {
    char c = Serial.read();
    nmea.process(c);
  }


  // Sleep till next capture time
  struct timeval time_to_next_capture;
  (void)gettimeofday(&now, NULL);
  if (timercmp(&now, &next_capture_time, <)) {
    timersub(&next_capture_time, &now, &time_to_next_capture);

    uint64_t sleep_time = ((uint64_t)time_to_next_capture.tv_sec) * 1000000L + time_to_next_capture.tv_usec;

    // Wake earlier to allow initialization
    if (sleep_time > WAKE_USEC_EARLY) {
      sleep_time -= WAKE_USEC_EARLY;
    } else {
      sleep_time = 0;
    }

    if (sleep_time >= MIN_SLEEP_TIME) {
      Serial.printf("Sleeping for %llu us\n", sleep_time);
      Serial.flush();

      // Preserve non-volatile data
      nv_data.next_capture_time = next_capture_time;

      camera_deinit();

      // Lock pin states (need to be unlocked at init again)
      rtc_gpio_hold_en(gpio_num_t(CAM_PWR_GPIO_NUM));

      esp_sleep_enable_timer_wakeup(sleep_time);
      esp_deep_sleep_start();
      // This line will never be reached....
    } else {

      //Serial.printf("Sleeptime %llu smaller than threshold %llu \n", sleep_time, MIN_SLEEP_TIME);
      //Serial.flush();
      delay(100);

    }
  }

}
