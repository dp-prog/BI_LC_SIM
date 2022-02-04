#ifdef __cplusplus
extern "C" {
#endif


#include "freertos/FreeRTOS.h"
#include "string.h"
#include "esp_log.h"


#include <stdio.h>
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_err.h"
#include "esp_log.h"

//#include "mb_master.h"


#include "esp_modbus_master.h"
#include "mbcontroller.h"		// for mbcontroller defines and api
#include "modbus_params.h"		// for modbus parameters structures

//#include "mbcontroller.h"

#include "mb_master.h"

holding_reg_params_t holding_reg_params = { 0 };
input_reg_params_t input_reg_params = { 0 };
coil_reg_params_t coil_reg_params = { 0 };
discrete_reg_params_t discrete_reg_params = { 0 };

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 30

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS          (1000)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_RATE_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS                 (100)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_RATE_MS)

#define MASTER_TAG "MASTER_TEST"

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MASTER_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

// Enumeration of modbus device addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_HOLD_DATA_0,	// BI_ID	0x01	1	uint16	0...65535	Идентификатор БИ
    CID_HOLD_DATA_1,	// Asc_Device	0x04	4	uint16	0...65535	Номер передаваемого пакета
    CID_HOLD_DATA_2,	// V_pwr	0x05	5	uint16	(0...+65,535 В)*1000	Напряжение питания БИ
    CID_HOLD_DATA_3,	// Флаги состояния БИ-М	0x07	7	uint16	0 - вскрытие, 1..3 - дат.коррозии, 4 - системная ошибка	Неисправность, вскрытие, обрыв цепей, ДК (датчики коррозии)
    CID_HOLD_DATA_4,	// V_Sum_1	0x10	16	int16	(-10...+10 В)*100	Суммарный потенциал 1
    CID_HOLD_DATA_5,	// V_Pol_1	0x11	17	int16	(-10...+10 В)*100	Поляризационный потенциал 1
    CID_HOLD_DATA_6,	// I_Pol_1	0x12	18	int16	(-50...+50 мА)*100	Ток поляризации 1

    CID_HOLD_DATA_7,	// Time_Now_Hi	0x30	48	uint16	UnixTime 32-bit	Текущее системное время сервера, старшие 2 байта
    CID_HOLD_DATA_8,	// Time_Now_Lo	0x31	49	uint16		Текущее системное время сервера, младшие 2 байта
    CID_HOLD_DATA_9,	// K_V_sum_1	0x33	51	uint16	(0...65535)/10000  (по умолчанию  10000)	Калибровочный коэффициент канала измерения суммарного потенциала
    CID_HOLD_DATA_10,	// K_V_pol_1	0x34	52	uint16	(0...65535)/10000 (по умолчанию  10000)	Калибровочный коэффициент канала измерения поляризационного потенциала
    CID_HOLD_DATA_11,	// K_I_pol_1	0x35	53	uint16	(0...65535)/10000 (по умолчанию  10000)	Калибровочный коэффициент канала измерения тока поляризации
    CID_HOLD_DATA_12,	// Slave_ID	0x3F	63	uint16	0..254	Сетевой адрес подключенного Slave-устройства сбора данных (БИ-М)
    CID_HOLD_DATA_13,	// Slave_ID_Change	0x40	64	uint16	0...65535	Смена адреса подключенного Slave-устройства сбора данных (1 - сохранение адреса)
    CID_HOLD_DATA_14,	// Set_BI_ID	0x41	65	uint16	0...65535	Установка идентификатора БИ
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
	{ CID_HOLD_DATA_0, STR("BI_ID"), STR("DevID"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 1, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_1, STR("Asc_Device"), STR("cnt"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 4, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_2, STR("V_pwr"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 5, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_3, STR("Flags"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 7, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_4, STR("V_Sum_1"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 16, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_5, STR("V_Pol_1"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 17, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_6, STR("I_Pol_1"), STR("mA"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 18, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_7, STR("Time_Now_Hi"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 48, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_8, STR("Time_Now_Lo"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 49, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_9, STR("K_V_sum_1"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 51, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_10, STR("K_V_pol_1"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 52, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_11, STR("K_I_pol_1"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 53, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_12, STR("Slave_ID"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 63, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_13, STR("Slave_ID_Change"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 64, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
	{ CID_HOLD_DATA_14, STR("Set_BI_ID"), STR("DevID"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 65, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, OPTS( 0, 65535, 1 ), PAR_PERMS_READ_WRITE },
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor)
{
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
       switch(param_descriptor->mb_param_type)
       {
           case MB_PARAM_HOLDING:
               instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_INPUT:
               instance_ptr = ((void*)&input_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_COIL:
               instance_ptr = ((void*)&coil_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_DISCRETE:
               instance_ptr = ((void*)&discrete_reg_params + param_descriptor->param_offset - 1);
               break;
           default:
               instance_ptr = NULL;
               break;
       }
    } else {
        ESP_LOGE(MASTER_TAG, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    float value = 0;
    bool alarm_state = false;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    ESP_LOGI(MASTER_TAG, "Start modbus test...");

    for(uint16_t retry = 0; retry <= MASTER_MAX_RETRY && (!alarm_state); retry++) {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++)
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void* temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
                if ((param_descriptor->param_type == PARAM_TYPE_ASCII) &&
                        (param_descriptor->cid == CID_HOLD_TEST_REG)) {
                   // Check for long array of registers of type PARAM_TYPE_ASCII
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                                            (uint8_t*)temp_data_ptr, &type);
                    if (err == ESP_OK) {
                        ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%08x) read successful.",
                                                 param_descriptor->cid,
                                                 (char*)param_descriptor->param_key,
                                                 (char*)param_descriptor->param_units,
                                                 *(uint32_t*)temp_data_ptr);
                        // Initialize data of test array and write to slave
                        if (*(uint32_t*)temp_data_ptr != 0xAAAAAAAA) {
                            memset((void*)temp_data_ptr, 0xAA, param_descriptor->param_size);
                            *(uint32_t*)temp_data_ptr = 0xAAAAAAAA;
                            err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key,
                                                              (uint8_t*)temp_data_ptr, &type);
                            if (err == ESP_OK) {
                                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%08x), write successful.",
                                                            param_descriptor->cid,
                                                            (char*)param_descriptor->param_key,
                                                            (char*)param_descriptor->param_units,
                                                            *(uint32_t*)temp_data_ptr);
                            } else {
                                ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) write fail, err = 0x%x (%s).",
                                                        param_descriptor->cid,
                                                        (char*)param_descriptor->param_key,
                                                        (int)err,
                                                        (char*)esp_err_to_name(err));
                            }
                        }
                    } else {
                        ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                                param_descriptor->cid,
                                                (char*)param_descriptor->param_key,
                                                (int)err,
                                                (char*)esp_err_to_name(err));
                    }
                } else {
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)&value, &type);
                    if (err == ESP_OK) {
                        *(float*)temp_data_ptr = value;
                        if ((param_descriptor->mb_param_type == MB_PARAM_HOLDING) ||
                            (param_descriptor->mb_param_type == MB_PARAM_INPUT)) {
                            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %f (0x%x) read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            value,
                                            *(uint32_t*)temp_data_ptr);
                            if (((value > param_descriptor->param_opts.max) ||
                                (value < param_descriptor->param_opts.min))) {
                                    alarm_state = true;
                                    break;
                            }
                        } else {
                            uint16_t state = *(uint16_t*)temp_data_ptr;
                            const char* rw_str = (state & param_descriptor->param_opts.opt1) ? "ON" : "OFF";
                            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %s (0x%x) read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            (const char*)rw_str,
                                            *(uint16_t*)temp_data_ptr);
                            if (state & param_descriptor->param_opts.opt1) {
                                alarm_state = true;
                                break;
                            }
                        }
                    } else {
                        ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (int)err,
                                            (char*)esp_err_to_name(err));
                    }
                }
                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS); //
    }

    if (alarm_state) {
        ESP_LOGI(MASTER_TAG, "Alarm triggered by cid #%d.",
                                        param_descriptor->cid);
    } else {
        ESP_LOGE(MASTER_TAG, "Alarm is not triggered after %d retries.",
                                        MASTER_MAX_RETRY);
    }
    ESP_LOGI(MASTER_TAG, "Destroy master...");
    ESP_ERROR_CHECK(mbc_master_destroy());
}

// Modbus master initialization
esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
            .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
            .mode = MB_MODE_RTU,
#endif
            .baudrate = MB_DEV_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MASTER_CHECK((master_handler != NULL), ESP_ERR_INVALID_STATE,
                                "mb controller initialization fail.");
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                              CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);

    err = mbc_master_start();
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);
    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}




#ifdef __cplusplus
}
#endif
