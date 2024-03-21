#include <stdio.h>
#include <string.h>
#include "sdcard.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"


static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
#define PIN_NUM_MISO  CONFIG_EXAMPLE_PIN_MISO
#define PIN_NUM_MOSI  CONFIG_EXAMPLE_PIN_MOSI
#define PIN_NUM_CLK   CONFIG_EXAMPLE_PIN_CLK
#define PIN_NUM_CS    CONFIG_EXAMPLE_PIN_CS

#define EXAMPLE_MAX_CHAR_SIZE    82

//Initialize card struct
sdmmc_card_t *card;

static esp_err_t sd_write_file(const char *path, char *data, char *mode){
    ESP_LOGI(TAG, "Opening file %s", path);
    if((int) *mode != 'w' && (int) *mode !='a'){
        ESP_LOGE(TAG, "Invalid writing mode");
        return ESP_FAIL;
    }

    FILE *file = fopen(path, mode);

    if (file == NULL){
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(file, data);
    fclose(file);
    ESP_LOGI(TAG,"File written");
    return ESP_OK;
}

static esp_err_t sd_read_file(const char *path){
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *file = fopen(path, "r");
    if (file == NULL){
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];

    while(fgets(line, sizeof(line), file)){
        printf( "Reading from line: %s", line);
    }
     fclose(file);
    return ESP_OK;
}


void sdspi_init(){
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 10,
        .allocation_unit_size = 16 * 1024
    };

    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    //host frequency change, default is 20HMz (range 400kHz - 20MHz for SDSPI)
     //host.max_freq_khz = 10000;


    //bus config and init
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    //initialize slot
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;


    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}


void sdspi_test(void)
{
    const char *file_test = MOUNT_POINT"/test.txt";
    sd_write_file(file_test, "test\n", "a");
    sd_read_file(file_test);
}


void sdspi_close(){
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    const char mount_point[] = MOUNT_POINT;
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}