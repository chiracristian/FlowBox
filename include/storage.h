#ifndef STORAGE_H
#define STORAGE_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * Initializes the USB MSC peripheral subsystem. 
 */
esp_err_t storage_usb_init(void);

/**
 * Mounts the microSD card to the internal VFS system at "/sdcard".
 */
esp_err_t storage_mount_local(void);

/**
 * Unmounts the microSD card from the local system.
 */
void storage_unmount_local(void);

/**
 * Starts the USB Mass Storage Class service, exposing the SD card to a PC.
 */
esp_err_t storage_enable_usb_msc(void);

/**
 * Disables the USB Mass Storage Class service.
 */
void storage_disable_usb_msc(void);

/**
 * Checks if the USB interface is currently connected to a host PC.
 */
bool storage_is_usb_connected(void);

#endif // STORAGE_H
