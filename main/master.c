//8N2 , 19200 , ID 126 , 0X03 
#include "string.h"
#include "esp_log.h"
#include "modbus_params.h"  
#include "mbcontroller.h"
#include "sdkconfig.h"
#include <endian.h>
#include <stdint.h>
#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   
#define MB_DEV_SPEED    (19200) 
#define MASTER_MAX_CIDS num_device_parameters
#define UPDATE_CIDS_TIMEOUT_MS          (150)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_RATE_MS)
// Timeout between polls
#define POLL_TIMEOUT_MS                 (150)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_RATE_MS)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }
float bigEndianToFloat(uint32_t bigEndianValue) {
    union {
        uint32_t intValue;
        float floatValue;
    } data; 

    data.intValue = __builtin_bswap32(bigEndianValue);
    return data.floatValue;
}
float littleEndianToFloat(uint32_t littleEndianValue) {
    union {
        uint32_t intValue;
        float floatValue;
    } data;

    data.intValue = littleEndianValue;
    return data.floatValue;
}
/*
Chuyển little endian swap bit to big endian 
manual pac 3100 output rtu big endian 
input esp32 rs485 little swap endian 
*/
float transferendian(uint32_t data_read) {
                             uint32_t data[3] ;
                             data[0] = data_read&0x000000ff; 
                             data[1] = data_read&0x0000ff00;
                             data[1]=  (data[1] >> 8) & 0xFF ;
                             data[2] = data_read&0x00ff0000;
                             data[2]=(data[2]>>16)&0xff; 
                             data[3] = data_read&0xff000000;
                             data[3]=(data[3]>>24)&0xff; 
                            //  printf ("%x = %x %x %X %x\n",data_read,data[0],data[1],data[2],data[3]); 
                             uint32_t data_transfer = data[1]<<24 | data[0]<<16 | data[2]<<8 | data[3]; 
                             float result = littleEndianToFloat(data_transfer);
                            //  printf("ket qua = %x = %f\n",data_read,result);
                             return result;
}

static const char *TAG = "MASTER_TEST";

enum {
    MB_DEVICE_ADDR1 = 126 // ADDRESS MODBUS 
};

enum {
    CID_UMIDADE = 0,
};
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
const mb_parameter_descriptor_t device_parameters[] = {
    { CID_UMIDADE, STR("VOLT B-N "), STR("%rH"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 3, 2,
            HOLD_OFFSET(UMIDADE), PARAM_TYPE_FLOAT, 2, OPTS( 0, 0, 0 ), PAR_PERMS_READ_TRIGGER  },
};
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));
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
        ESP_LOGE(TAG, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
static void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    float value ;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    ESP_LOGI(TAG, "Start modbus test...");

        while (1){
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++)
        {
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void* temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)&value, &type);
                    if (err == ESP_OK) {
                            
                        *(float*)(temp_data_ptr) = value; // tham chiếu đến địa chỉ temp data 
                        if ((param_descriptor->mb_param_type == MB_PARAM_HOLDING) ) {
                            float volt = transferendian (*(uint32_t*)temp_data_ptr);  
                            ESP_LOGI(TAG, "DATA READ #%d %s value = %f read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            volt
                                            );                 
                             printf("---------------------------------------------------------------------------------------\n");
                        } 

                        
                    } else {
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (int)err,
                                            (char*)esp_err_to_name(err));
                    }
                
                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS); //
        }

    // if (alarm_state) {
    //     ESP_LOGI(TAG, "Alarm triggered by cid #%d.",
    //                                     param_descriptor->cid);
    // } else {
    //     ESP_LOGE(TAG, "Alarm is not triggered after %d retries.",
    //                                     MASTER_MAX_RETRY);
    // }
    // ESP_LOGI(TAG, "Destroy master...");
    // ESP_ERROR_CHECK(mbc_master_destroy());
}

// Modbus master initialization
static esp_err_t master_init(void)
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
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                              CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}
float littleEndianBytesToFloat(uint8_t bytes[4]) {
    // Sử dụng union để chuyển đổi từ byte sang float mà không cần ép kiểu.
    union {
        uint32_t intValue;
        float floatValue;
    } data;

    // Gán các byte little endian vào data.intValue.
    data.intValue = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];

    return data.floatValue;
}

void app_main(void)
{
    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    master_operation_func(NULL);
}
