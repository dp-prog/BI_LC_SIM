#ifdef __cplusplus
extern "C" {
#endif


#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <string.h>


#include "esp_system.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"


/* SD */
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#define MOUNT_POINT "/sdcard"

static const char *TAG_SD = "SD";


#define USE_SPI_MODE

#ifndef SPI_DMA_CHAN
	#define SPI_DMA_CHAN    1
#endif //SPI_DMA_CHAN

// Pin mapping when using SPI mode.
#define PIN_MISO			2
#define PIN_MOSI			15
#define PIN_CLK				14
#define PIN_CS				13

#include <expat.h>
#include <sdcard.h>

#ifdef XML_LARGE_SIZE
# if defined(XML_USE_MSC_EXTENSIONS) && _MSC_VER < 1400
#  define XML_FMT_INT_MOD "I64"
# else
#  define XML_FMT_INT_MOD "ll"
# endif
#else
# define XML_FMT_INT_MOD "l"
#endif

#ifdef XML_UNICODE_WCHAR_T
# include <wchar.h>
# define XML_FMT_STR "ls"
#else
# define XML_FMT_STR "s"
#endif


static char	*last_content;

static void XMLCALL startElement(void *userData, const XML_Char *name, const XML_Char **atts){
  int *depthPtr = (int *)userData;
  (void)atts;

  for (int i = 0; i < *depthPtr; i++) putchar('\t');
  printf("Tag: %" XML_FMT_STR " >> ", name);
  for(int j = 0; atts[j]; j += 2){
      printf("%s='%s'; ", atts[j], atts[j + 1]);
  }
  printf("\n");
  *depthPtr += 1;
}

static void XMLCALL endElement(void *userData, const XML_Char *name){
  int *depthPtr = (int *)userData;
//  for (int i = 0; i < *depthPtr; i++) putchar('\t');
//  printf("Content of element %s was \"%s\"\n", name, last_content);
  (void)name;
  *depthPtr -= 1;
}

static void XMLCALL valueElement(void* userData, const XML_Char *val, int len){
  char str[len+1];	//char *tmp = malloc(len + 1);
  strncpy(str, val, len);
  str[len] = '\0';

  int *depthPtr = (int *)userData;
//  for (int i = 0; i < *depthPtr; i++) putchar('\t');
//  printf("value: %" XML_FMT_STR "\n", str);
//  last_content = str;
}

void read_file(char *filename){
	char file_mount_point[16];
	sprintf (file_mount_point, MOUNT_POINT"/%s", filename);
	ESP_LOGI(TAG_SD, "Opening file %s", file_mount_point);
	FILE* f = fopen(file_mount_point, "r");
	if (f == NULL) {
		ESP_LOGE(TAG_SD, "Failed to open file");
		return;
	}

	  char buf[BUFSIZ];
	  XML_Parser parser = XML_ParserCreate(NULL);
	  if (! parser) {
	    fprintf(stderr, "Couldn't allocate memory for parser\n");
	    return;
	  }

	  int done;
	  int depth = 0;
	  XML_SetUserData(parser, &depth);
	  XML_SetElementHandler(parser, startElement, endElement);
	  XML_SetCharacterDataHandler(parser, valueElement);

	  do {
	    size_t len = fread(buf, 1, sizeof(buf), f);
	    done = len < sizeof(buf);

	    if (XML_Parse(parser, buf, len, done) == XML_STATUS_ERROR) {
	      fprintf(stderr, "%" XML_FMT_STR " at line %" XML_FMT_INT_MOD "u\n", \
	              XML_ErrorString(XML_GetErrorCode(parser)), \
	              XML_GetCurrentLineNumber(parser));
	      return;
	    }
	  } while (!done);
	  XML_ParserFree(parser);
	fclose(f);
	ESP_LOGI(TAG_SD, "Close file %s", filename);
}

void sdcard_task(void *pvParameters) {
/* SD card init */
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG_SD, "Initializing SD card. Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "Failed to initialize bus.");
        return;
    }
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_SD, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG_SD, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    sdmmc_card_print_info(stdout, card);		// Card has been initialized, print its properties

    read_file("Config_BI_LC.xml");
    read_file("KpModbusSlave_KIP-LC.xml");



    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

	// All done, unmount partition and disable SDMMC or SPI peripheral
	esp_vfs_fat_sdcard_unmount(mount_point, card);
	ESP_LOGI(TAG_SD, "Card unmounted");
}




#ifdef __cplusplus
}
#endif
