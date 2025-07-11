#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "applications/proto/uart.pb.h"


LOG_MODULE_REGISTER(main_rx, LOG_LEVEL_DBG);

K_SEM_DEFINE(uart_data_ready, 0, 1);
K_SEM_DEFINE(buf_fer_ready, 0, 1);

#define UART_DEVICE_NODE DT_NODELABEL(usart1)

static const struct device * const dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define STACK_SIZE 1024
#define RX_CHUNK_LEN 32
#define RX_BUF_SIZE 64
#define RX_MSG_QUEUE 10

struct rx_msg {
    uint8_t bytes[64];
    uint32_t length;
};

static K_THREAD_STACK_DEFINE(uart_rx_stack, STACK_SIZE);

static struct k_thread thread_data;



static uint8_t rx_buffer[2][RX_BUF_SIZE];

volatile uint8_t * data = NULL;
volatile size_t len = 0;

static volatile uint8_t idx_rx_buffer;


K_MSGQ_DEFINE(rx_msg_queue, sizeof(struct rx_msg), RX_MSG_QUEUE , 4);

///******** Encode and Decode **********/

 bool decode_message(uint8_t *buffer, size_t message_length ){

    SensorData data = SensorData_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);
    bool status = pb_decode(&stream, SensorData_fields, &data);

    LOG_INF("decoding...");
    if (status){

        LOG_INF("Data frm Sensors: %d, %d, %d, %d, %d ", (int)data.thumbf, (int)data.indexf, (int)data.middlef, (int)data.ringf, (int)data.pinkyf);

    }else {

        LOG_ERR("Decoding failed: %s", PB_GET_ERROR(&stream));
    }

    return status;
}


static void uart_callback_func(const struct device * dev, struct uart_event *evt, void * user_data){
    // LOG_DBG("UART event received on %s", dev->name);
    int rc;
    switch (evt->type){
    case UART_RX_RDY:
        data = evt->data.rx.buf + evt->data.rx.offset;
        len = evt->data.rx.len;
        size_t rx_used = evt->data.rx.offset + evt->data.rx.len;
        int err = uart_err_check(dev);
        if (err != 0){
             if (err & UART_ERROR_OVERRUN) {
        LOG_ERR("UART Overrun Error");
        }
        if (err & UART_ERROR_PARITY) {
        LOG_ERR("UART Parity Error");
        }
        if (err & UART_ERROR_FRAMING) {
        LOG_ERR("UART Framing Error");
        }
        LOG_ERR("UART error check 0x%x", err);
        }
        k_sem_give(&uart_data_ready);
     
        break;
    case UART_RX_BUF_REQUEST:
        LOG_INF("Buffer request");
         rc = uart_rx_buf_rsp(dev, rx_buffer[idx_rx_buffer], RX_BUF_SIZE);
        __ASSERT_NO_MSG(rc==0);
        idx_rx_buffer = (idx_rx_buffer + 1) % 2;

        break;
    case UART_RX_STOPPED:
        LOG_INF("RX Stopped");
        break;

    case UART_RX_BUF_RELEASED:

        LOG_INF("Buffer released");
        break;
    case UART_RX_DISABLED:
        LOG_INF("RX disabled callback");
        k_sem_give(&buf_fer_ready);
        rc = uart_rx_enable(dev, rx_buffer[idx_rx_buffer], RX_BUF_SIZE, 1000);
        if (rc != 0){
                LOG_ERR("Failed to enable RX... %d", rc);
        }
        break;
   

    default:
        break;
    
    }
}

static void decoder_thread(void *a, void *b, void *c){

    struct rx_msg incoming_data;

    while(1){

        if (k_msgq_get(&rx_msg_queue, &incoming_data, K_FOREVER) == 0){
 
            if (incoming_data.length < 2 || incoming_data.length > sizeof(incoming_data.bytes)){
                LOG_ERR("Invalid message length: %u", incoming_data.length);
                continue;
            }

            size_t prefix_len = incoming_data.bytes[0];

            LOG_INF("Printing length of the message: %zu", prefix_len);

            if (prefix_len + 1 != incoming_data.length){
                LOG_ERR("Incoming data's length doees not match with pfx: %d", incoming_data.length);
                continue;
            }

            bool lmao = decode_message(&incoming_data.bytes[1], prefix_len);

            if (!lmao){

                LOG_ERR("Decoding failed");
            }else{

                LOG_INF("Decoded success");


            }
        
            

        }

        k_sleep(K_MSEC(100));
    }
}

int main(void){

    k_sleep(K_MSEC(100));

    if (!device_is_ready(dev)){
        LOG_ERR("Uart device is not ready");
        return -ENODEV;
    }

    idx_rx_buffer = 0;
    
    uart_callback_set(dev, uart_callback_func, NULL);

   int rx_test = uart_rx_enable(dev, rx_buffer[idx_rx_buffer], RX_BUF_SIZE, 1000);
    if (rx_test != 0) {
        LOG_ERR("Initial uart_rx_enable() failed: %d", rx_test);
    return rx_test;
    }


    LOG_INF("Uart enabled, listening....");


    k_tid_t tid = k_thread_create(&thread_data, uart_rx_stack, K_THREAD_STACK_SIZEOF(uart_rx_stack),decoder_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);

    k_thread_name_set(tid, "TX_blud");

    while (1){

        k_sem_take(&uart_data_ready, K_FOREVER);

        uint8_t * data_process = data;
        size_t length = len;

        LOG_HEXDUMP_INF(data_process, length, "RX_RDY");

    
            uint8_t expected_len = data_process[0];
      
            if (length < expected_len + 1){
                LOG_WRN("Incomplete message, discard");
                continue;
            }

            struct rx_msg new_msg = {
                .length = expected_len + 1
            };
             memcpy(new_msg.bytes, data_process, new_msg.length);

   
            if (k_msgq_put(&rx_msg_queue, &new_msg, K_NO_WAIT) != 0) {
                LOG_ERR("Failed to enqueue complete message");
            } else {
                LOG_INF("Message enqueued: %d bytes", new_msg.length);
            }
        

        data = NULL;
        len = 0;
    }
}
