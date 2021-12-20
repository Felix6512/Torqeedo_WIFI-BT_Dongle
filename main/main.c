#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <sys/param.h>
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "hal/uart_types.h"
#include "nvs_flash.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/uart_reg.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "include/foo.h"





#define TAG "Project_Info"

static QueueHandle_t uart0_receive_queue, uart1_receive_queue, uart2_receive_queue;

QueueSetMemberHandle_t xActivatedMember;

struct sockaddr_in6 Android_addr;

uint8_t UARTx_is_recieving = 0;
uint8_t UART1_firstBytes[50] = {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0};
uint8_t UDP_Input_Speed [2] = {0, 0};
uint8_t UART1_Byte_No = 0;
uint8_t UART1_OutBytes[8] = {0,0,0,0,0,0,0,0};
//_Bool UART1_replace = 0;
_Bool Android = 0;

uint16_t D_System_Flags = 0,
		D_Motor_Voltage = 0,
		D_Motor_Current = 0,
		D_Motor_Power = 0,
		D_Motor_Speed = 0,
		D_Battery_Voltage = 0,
		D_Battery_Current = 0,
		D_GPS_Speed = 0,
		D_GPS_Miles = 0,
		D_Range_Minutes;
uint8_t D_Motor_Temp1 = 0,
		D_Motor_Temp2 = 0,
		D_Battery_Charge = 0;

uint8_t UART_Pattern_Speed[5] = {172, 20, 1, 137, 173};
uint8_t UART_Pattern_Status[3] = {172, 20, 41};
uint8_t UART_Pattern_System[3] = {172, 20, 42};

static TaskHandle_t handle_UART_INIT = NULL, handle_UART1_Receive = NULL,  handle_UART_Sender = NULL;

#define RD_BUF_SIZE (BUF_SIZE)
#define BUF_SIZE (1024)

#define PORT 5000

#define ESP_WIFI_SSID      "Bootslan"
#define ESP_WIFI_PASS      "12345678"
#define ESP_WIFI_CHANNEL   13
#define MAX_STA_CONN       8

const int uart_num0 = UART_NUM_0;
uart_config_t uart_config0 = {
	.baud_rate = 115200,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.rx_flow_ctrl_thresh = 122,
};
const int uart_num1 = UART_NUM_1;
uart_config_t uart_config1 = {
	.baud_rate = 19200,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.rx_flow_ctrl_thresh = 122,
	.source_clk = UART_SCLK_APB,
};
const int uart_num2 = UART_NUM_2;
uart_config_t uart_config2 = {
	.baud_rate = 19200,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.rx_flow_ctrl_thresh = 122,
	.source_clk = UART_SCLK_APB,
};


uint8_t crc8( uint8_t *addr, uint8_t len)
{
     uint8_t crc=0;

     for (uint8_t i=0; i<len;i++)
     {
           uint8_t inbyte = addr[i];
           for (uint8_t j=0;j<8;j++)
           {
                 uint8_t mix = (crc ^ inbyte) & 0x01;
                 crc >>= 1;
                 if (mix)
                       crc ^= 0x8C;

                 inbyte >>= 1;
           }
     }
     return crc;
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            //ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        //ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {

            //ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } /*else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }*/

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%d|%d", rx_buffer[0], rx_buffer[1]);
                if(len == 2){
                	UDP_Input_Speed[0] = rx_buffer[0];
                	UDP_Input_Speed[1] = rx_buffer[1];
                	Android = 1;
                	Android_addr = source_addr;
                }
                /*char reply[2] = "OK";
                int err = sendto(sock, reply, 2, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }*/
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            //ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        //ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {
        	vTaskDelay(250/ portTICK_RATE_MS);
            if(Android){
				//socklen_t socklen = sizeof(Android_addr);
				char reply[20];
				reply[0] = D_Battery_Charge;
				reply[1] = D_Motor_Voltage/256;
				reply[2] = D_Motor_Voltage && 0x00FF;
				reply[3] = D_Motor_Current/256;
				reply[4] = D_Motor_Current && 0x00FF;
				reply[5] = D_Motor_Power/256;
				reply[6] = D_Motor_Power && 0x00FF;
				reply[7] = D_Motor_Speed/256;
				reply[8] = D_Motor_Speed && 0x00FF;
				reply[9] = D_Battery_Voltage/256;
				reply[10] = D_Battery_Voltage && 0x00FF;
				reply[11] = D_Battery_Current/256;
				reply[12] = D_Battery_Current && 0x00FF;
				reply[13] = D_GPS_Speed/256;
				reply[14] = D_GPS_Speed && 0x00FF;
				reply[15] = D_GPS_Miles/256;
				reply[16] = D_GPS_Miles && 0x00FF;
				reply[17] = D_Range_Minutes/256;
				reply[18] = D_Range_Minutes && 0x00FF;


				int err = sendto(sock, reply, 19, 0, (struct sockaddr *)&Android_addr, sizeof(Android_addr));
				if (err < 0) {
					ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
					break;
				}
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        //wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        //ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        //wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        //ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
}

static void UART1_Receive(void *arg)
{
	vTaskSuspend(NULL);

	uart_event_t event1;
	uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
	ESP_LOGI(TAG, "Receive UART1 start.");
	for( ;; ){
		if(xQueueReceive(uart1_receive_queue, (void * )&event1, (portTickType)portMAX_DELAY)) {
			if(event1.size == 1){
				switch(event1.type) {
					case UART_DATA:
						gpio_set_level(GPIO_NUM_27, 1);
						uart_read_bytes(uart_num1, dtmp, 1, portMAX_DELAY);
						//uart_write_bytes(uart_num0, (const char*)dtmp, 1);
						if(dtmp[0] == 172)
						{
							if(UARTx_is_recieving == 0){
								UARTx_is_recieving = 1;
								UART1_Byte_No = 0;
							}

						}

						if(UARTx_is_recieving == 1)
						{
							UART1_Byte_No++;
							UART1_firstBytes[UART1_Byte_No-1] = dtmp[0];
							//uart_write_bytes(uart_num0, (const char*)dtmp, 1);

						}

						if(dtmp[0] == 173)
						{
							//uart_write_bytes(uart_num0, (const char*)UART1_firstBytes, UART1_Byte_No);

							//EOP Request of Remote?
							if((UART1_Byte_No == 5) &&
									(UART_Pattern_Speed[0] = UART1_firstBytes[0]) &&
									(UART_Pattern_Speed[1] = UART1_firstBytes[1]) &&
									(UART_Pattern_Speed[2] = UART1_firstBytes[2]) &&
									(UART_Pattern_Speed[3] = UART1_firstBytes[3]) &&
									(UART_Pattern_Speed[4] = UART1_firstBytes[4]))
							{
								vTaskResume(handle_UART_Sender);
							}

							if((UART1_Byte_No == 32)&&
									(UART_Pattern_Status[0] = UART1_firstBytes[0]) &&
									(UART_Pattern_Status[1] = UART1_firstBytes[1]) &&
									(UART_Pattern_Status[2] = UART1_firstBytes[2]))
							{

								D_System_Flags = UART1_firstBytes[4] + (256 * UART1_firstBytes[3]);
								D_Motor_Voltage = UART1_firstBytes[8] + (256 * UART1_firstBytes[7]);
								D_Motor_Current = UART1_firstBytes[10] + (256 * UART1_firstBytes[9]);
								D_Motor_Power = UART1_firstBytes[12] + (256 * UART1_firstBytes[11]);
								D_Motor_Speed = UART1_firstBytes[14] + (256 * UART1_firstBytes[13]);
								D_Battery_Charge = UART1_firstBytes[17];
								D_Battery_Voltage = UART1_firstBytes[19] + (256 * UART1_firstBytes[18]);
								D_Battery_Current = UART1_firstBytes[21] + (256 * UART1_firstBytes[20]);
								D_GPS_Speed = UART1_firstBytes[23] + (256 * UART1_firstBytes[22]);
								D_GPS_Miles	= UART1_firstBytes[25] + (256 * UART1_firstBytes[24]);
								D_Range_Minutes = UART1_firstBytes[27] + (256 * UART1_firstBytes[26]);
								ESP_LOGI(TAG, "MV: %d| MC: %d| MP: %d| MS: %d| BCH: %d| BV: %d| BC: %d| GS: %d| GM: %d| RM: %d| ",
										D_Motor_Voltage,D_Motor_Current,D_Motor_Power,D_Motor_Speed,
										D_Battery_Charge,D_Battery_Voltage,D_Battery_Current,
										D_GPS_Speed,D_GPS_Miles,D_Range_Minutes);
								//uart_write_bytes(uart_num0, (const char*)UART1_firstBytes, UART1_Byte_No);


							}

							if((UART1_Byte_No == 15)&&
									(UART_Pattern_System[0] = UART1_firstBytes[0]) &&
									(UART_Pattern_System[1] = UART1_firstBytes[1]) &&
									(UART_Pattern_System[2] = UART1_firstBytes[2]))
							{

							}

							UARTx_is_recieving = 0;

							uart_flush(uart_num1);
						}
						break;
					default:
						ESP_LOGI(TAG, "uart1 event type: %d", event1.type);
						break;
				}
			}
			else{
				ESP_LOGI(TAG, "uart1 event size: %d", event1.size);
			}
		}
	}
}


static void UART_sender(void *arg)
{
	uint8_t Bytes_to_send[9];
	//wait for startup
	vTaskSuspend(NULL);
	vTaskDelay(100/ portTICK_RATE_MS);

	for( ;; ){
		//wait for trigger
		vTaskSuspend(NULL);
		gpio_set_level(GPIO_NUM_26, 1);
		Bytes_to_send[0] = 0;
		Bytes_to_send[1] = 0;
		Bytes_to_send[2] = 0;
		Bytes_to_send[3] = 5;
		Bytes_to_send[4] = 0;
		Bytes_to_send[5] = UDP_Input_Speed[0];
		Bytes_to_send[6] = UDP_Input_Speed[1];
		Bytes_to_send[7] = crc8(Bytes_to_send,7);
		Bytes_to_send[0] = 172;
		Bytes_to_send[8] = 173;
		vTaskDelay(10/ portTICK_RATE_MS);
		uart_write_bytes(uart_num1, (const char*)Bytes_to_send, 9);
		uart_write_bytes(uart_num0, (const char*)Bytes_to_send, 9);
		gpio_set_level(GPIO_NUM_26, 0);
		gpio_set_level(GPIO_NUM_27, 0);
	}
}

static void UART_INIT(void *arg)
{
	ESP_ERROR_CHECK(uart_param_config(uart_num0, &uart_config0));
	ESP_ERROR_CHECK(uart_param_config(uart_num1, &uart_config1));
	ESP_ERROR_CHECK(uart_param_config(uart_num2, &uart_config2));

	ESP_ERROR_CHECK(uart_set_pin(uart_num0, 1, 3, 22, 19));
	ESP_ERROR_CHECK(uart_set_pin(uart_num1, 17, 16, 4, 23));
	ESP_ERROR_CHECK(uart_set_pin(uart_num2, 15, 18, 13, 12));

	ESP_ERROR_CHECK(uart_driver_install(uart_num0, BUF_SIZE * 2, 0, 1, &uart0_receive_queue, 0));
	ESP_ERROR_CHECK(uart_driver_install(uart_num1, BUF_SIZE * 2, 0, 1, &uart1_receive_queue, 0));
	ESP_ERROR_CHECK(uart_driver_install(uart_num2, BUF_SIZE * 2, 0, 1, &uart2_receive_queue, 0));

	uart_set_mode(uart_num1, UART_MODE_RS485_HALF_DUPLEX);
	uart_set_mode(uart_num2, UART_MODE_RS485_HALF_DUPLEX);

	ESP_LOGI(TAG, "\nAll UARTs started.");

	vTaskResume(handle_UART1_Receive);
	vTaskResume(handle_UART_Sender);

	for( ;; )
	{
		vTaskSuspend(NULL);
	}
}

int foon(int x)    /* Function definition */
{
    return x + 5;
}

void app_main(void)
{
	esp_log_level_set(TAG, ESP_LOG_INFO);

    //GPIO-Setup
	gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
	//gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
	//gpio_set_level(GPIO_NUM_4, 0);
	gpio_set_level(GPIO_NUM_27, 0);
	gpio_set_level(GPIO_NUM_26, 0);

	//WIFI-Setup
	esp_err_t wifi_ret = nvs_flash_init();
	if (wifi_ret == ESP_ERR_NVS_NO_FREE_PAGES || wifi_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  wifi_ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(wifi_ret);

	ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
	wifi_init_softap();

	int x = 0;
	x=foon(5);
	ESP_LOGE(TAG, "Test %d",x);

	//FreeRTOS-Setup

	xTaskCreatePinnedToCore(UART1_Receive, "UART1_Receive", 2048, NULL, 12, &handle_UART1_Receive, 0);
	xTaskCreatePinnedToCore(UART_sender, "UART_sender", 2048, NULL, 15, &handle_UART_Sender, 0);
	xTaskCreatePinnedToCore(UART_INIT, "UART_INIT", 2048, NULL, tskIDLE_PRIORITY, &handle_UART_INIT, 0);
	xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL, 1);
	xTaskCreatePinnedToCore(udp_client_task, "udp_client", 4096, (void*)AF_INET, 5, NULL, 1);


	for( ;; ){
		vTaskDelay(1000/ portTICK_RATE_MS);
	}
}

