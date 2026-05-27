#include "storage.h"
#include <stdio.h>
#include <string.h>
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "esp_log.h"

static const char *TAG = "FlowBox_Storage";

#define MOUNT_POINT      "/sdcard"
#define SDMMC_CMD_PIN    2
#define SDMMC_D0_PIN     42
#define SDMMC_CLK_PIN    1

static sdmmc_card_t *s_card = NULL;
static bool s_local_mounted = false;
static bool s_usb_msc_active = false;

// Matches tusb_msc_callback_t signature
static void usb_status_cb(tinyusb_msc_event_t *event)
{
    if (event->type == TINYUSB_MSC_EVENT_MOUNT_CHANGED) {
        ESP_LOGI(TAG, "USB MSC event: Mount status changed. Mounted: %d", 
                 event->mount_changed_data.is_mounted);
    }
}

esp_err_t storage_mount_local(void)
{
    if (s_local_mounted) return ESP_OK;
    
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SDMMC_CLK_PIN;
    slot_config.cmd = SDMMC_CMD_PIN;
    slot_config.d0 = SDMMC_D0_PIN;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret == ESP_OK) s_local_mounted = true;
    return ret;
}

void storage_unmount_local(void)
{
    if (!s_local_mounted) return;
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
    s_local_mounted = false;
}

esp_err_t storage_usb_init(void)
{
    const tinyusb_config_t tusb_cfg = { 
        .device_descriptor = NULL, 
        .string_descriptor = NULL, 
        .external_phy = false, 
        .configuration_descriptor = NULL 
    };
    return tinyusb_driver_install(&tusb_cfg);
}

// Modify storage_enable_usb_msc to keep the controller separate from the storage backend
esp_err_t storage_enable_usb_msc(void)
{
    if (s_usb_msc_active) return ESP_OK;
    if (s_local_mounted) storage_unmount_local();

    // Init hardware
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1; slot_config.clk = SDMMC_CLK_PIN; slot_config.cmd = SDMMC_CMD_PIN; slot_config.d0 = SDMMC_D0_PIN;
    
    s_card = malloc(sizeof(sdmmc_card_t));
    sdmmc_host_init(); 
    sdmmc_host_init_slot(host.slot, &slot_config);
    sdmmc_card_init(&host, s_card);

    // Instead of auto-init, configure the MSC stack manually
    tinyusb_msc_sdmmc_config_t msc_config = {
        .card = s_card,
        .callback_mount_changed = usb_status_cb,
        .mount_config = { .format_if_mount_failed = false, .max_files = 4 }
    };
    
    // Only call this when you WANT to talk to the PC
    tinyusb_msc_storage_init_sdmmc(&msc_config);
    
    s_usb_msc_active = true;
    return ESP_OK;
}

void storage_disable_usb_msc(void)
{
    tinyusb_msc_storage_deinit();
    s_usb_msc_active = false;
}

bool storage_is_usb_connected(void) 
{ 
    return tinyusb_msc_storage_in_use_by_usb_host(); 
}
