#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "applications/proto/uart.pb.h"

struct tx_buffer {
    const uint8_t * data;
    size_t length;
};

static struct tx_buffer tx_pending_buffer;

#define UART_DEVICE_NODE DT_NODELABEL(usart1)

static const struct device * const dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define STACK_SIZE 1024


LOG_MODULE_REGISTER(main_tx, LOG_LEVEL_DBG);


static K_THREAD_STACK_DEFINE(uart_rx_stack, STACK_SIZE);

static struct k_thread thread_data;

bool encode_message(uint8_t *buffer, size_t buffer_size, size_t *message_length){
    SensorData data = SensorData_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);

    data.thumbf = 146;
    data.indexf = 40;
    data.middlef = 82;
    data.ringf = 200;
    data.pinkyf = 304;

    bool status =  pb_encode(&stream, SensorData_fields, &data);
    *message_length = stream.bytes_written;

    if (!status)
    {
        LOG_ERR("Encoding failed: %s", PB_GET_ERROR(&stream));
    }

    return status;

   
    


}



static void uart_tx_thread(void *p1, void *p2, void *p3){
    // const char * message = "Hello\n";
    // size_t size = strlen(message);
    // int rc;
    uint8_t tx_buffer[64];
    size_t encoded_length;
    int rc;

    LOG_INF("TX thread is running...");
    while (1){
        if (encode_message(tx_buffer + 1, sizeof(tx_buffer) -1, &encoded_length)){
            tx_buffer[0] = (uint8_t)encoded_length;

            rc = uart_tx(dev, tx_buffer, encoded_length + 1, SYS_FOREVER_US);
                
             if (rc != 0){
                LOG_ERR("Failed to transmit: %d", rc);
        }   else {
            // tx_pending_buffer.data = (const uint8_t *)message;
            // tx_pending_buffer.length = size;
            LOG_HEXDUMP_INF(tx_buffer, encoded_length + 1, "TX Sent");
            
            LOG_INF("Sent!!!");
        }

        }
        
       
        k_sleep(K_SECONDS(3));
    }


}

static void uart_callback_func(const struct device *dev, struct uart_event *evt, void *user_data){

   LOG_INF("Event: %d", evt->type);


   switch (evt->type){
    case UART_TX_DONE:
        //  LOG_HEXDUMP_INF(tx_pending_buffer.data, tx_pending_buffer.length, "TX Sent");

        LOG_INF("TX Done");
         break;
    
        
    default:
        LOG_WRN("Unhandled event %d", evt->type);
        break;
    
    }
   


}
int main(void){
    if (!device_is_ready(dev)){
        LOG_ERR("Uart is not ready");
        return 1;
    }

    LOG_INF("Uart device is ready");

    uart_callback_set(dev, uart_callback_func, NULL);   

    k_tid_t tid = k_thread_create(&thread_data, uart_rx_stack, K_THREAD_STACK_SIZEOF(uart_rx_stack),uart_tx_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);

    k_thread_name_set(tid, "TX_blud");



}


