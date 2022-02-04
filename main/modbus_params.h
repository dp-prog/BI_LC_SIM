/*=====================================================================================
 * Description:
 *   The Modbus parameter structures used to define Modbus instances that
 *   can be addressed by Modbus protocol. Define these structures per your needs in
 *   your application. Below is just an example of possible parameters.
 *====================================================================================*/
#ifndef _DEVICE_mb_PARAMS
#define _DEVICE_mb_PARAMS

/* predefine options */
#define CONFIG_MB_UART_PORT_NUM		2		/* Number of UART port used for Modbus connection */
#define CONFIG_MB_SLAVE_ADDR		1		/* The address of device in Modbus network */
#define CONFIG_MB_UART_BAUD_RATE	9600	/* The communication speed of the UART */
//#define CONFIG_MB_COMM_MODE_ASCII	1
#define CONFIG_MB_COMM_MODE_RTU		1

#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   /* Number of UART port used for Modbus connection */
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  /* The communication speed of the UART */


/* Использую
 * Circuit A: Collision Detection Circuit
 * Эта схема предпочтительнее, поскольку позволяет обнаруживать коллизии.
 * Приемник в драйвере линии постоянно включен, что позволяет UART контролировать шину RS485.
 * Подавление эха выполняется периферийным устройством UART, когда
 * бит UART_RS485_CONF_REG.UART_RS485TX_RX_EN включен.
 */

#define CONFIG_MB_UART_TXD			12
#define CONFIG_MB_UART_RXD			34
#define CONFIG_MB_UART_RTS			4
#define CONFIG_MB_UART_CTS			UART_PIN_NO_CHANGE

static const char *TAG_MB_SLAVE = "MB_SLAVE";


// This file defines structure of modbus parameters which reflect correspond modbus address space
// for each modbus register type (coils, discreet inputs, holding registers, input registers)
#pragma pack(push, 1)
typedef struct
{
    uint8_t discrete_input0:1;
    uint8_t discrete_input1:1;
    uint8_t discrete_input2:1;
    uint8_t discrete_input3:1;
    uint8_t discrete_input4:1;
    uint8_t discrete_input5:1;
    uint8_t discrete_input6:1;
    uint8_t discrete_input7:1;
    uint8_t discrete_input_port1:8;
} discrete_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t coils_port0;
    uint8_t coils_port1;
} coil_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    float input_data0; // 0
    float input_data1; // 2
    float input_data2; // 4
    float input_data3; // 6
    uint16_t data[150]; // 8 + 150 = 158
    float input_data4; // 158
    float input_data5;
    float input_data6;
    float input_data7;
} input_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    float holding_data0;
    float holding_data1;
    float holding_data2;
    float holding_data3;
    uint16_t test_regs[150];
    float holding_data4;
    float holding_data5;
    float holding_data6;
    float holding_data7;
} holding_reg_params_t;
#pragma pack(pop)

extern holding_reg_params_t		holding_reg_params;
extern input_reg_params_t		input_reg_params;
extern coil_reg_params_t		coil_reg_params;
extern discrete_reg_params_t	discrete_reg_params;

// Таблица соответствия регистров модбас @todo
#define V_Sum_1					holding_reg_params.holding_data[0x10]
#define V_Pol_1					holding_reg_params.holding_data[0x11]
#define I_Pol_1					holding_reg_params.holding_data[0x12]

#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_SLAVE_ADDR   (CONFIG_MB_SLAVE_ADDR)      // The address of device in Modbus network
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART

//#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
//#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
//#define MB_REG_DISCRETE_INPUT_START         (0x0000)
//#define MB_REG_COILS_START                  (0x0000)
//#define MB_REG_INPUT_START		            (INPUT_OFFSET(input_data[0]))		// register offset input area 0
//#define MB_REG_HOLDING_START		        (HOLD_OFFSET(holding_data[0]))
//
//#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
//#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD | MB_EVENT_HOLDING_REG_RD | MB_EVENT_DISCRETE_RD | MB_EVENT_COILS_RD)
//#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)
//#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)


#endif // !defined(_DEVICE_mb_PARAMS)
