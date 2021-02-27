/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// added 
#include <esp_pm.h>
//to install gpio isr service
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include <scli.h>
#include <diag_cli.h>
#include <wifi_cli.h>
#include "nvs.h"

static const char *TAG = "[app_main]";

#define TCP_QUEUE_LENGTH  (100)
#define MAX_TCP_BUF  (4*8*1024*2)  //approximate 0.5 seconds at 16K sample rate
#define MAX_TCP_SEND_SIZE  (4*1024) //send the data in chunks max received pkts

xTaskHandle xHandleTaskTcp;
QueueHandle_t xHandleQueueTcp;
struct xQ_TcpPacket
{
  uint16_t data_index;  /* index of data start  */
  uint16_t data_size;   /* size of data written */
};
struct tcp_send_info {
uint8_t send_buf[MAX_TCP_BUF];
volatile int rd_index;
volatile int wr_index;
volatile int wr_count;
xSemaphoreHandle lock;
};

struct tcp_send_info tcp_send;
uint8_t tcp_send_pkt[MAX_TCP_SEND_SIZE];

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

#if 0  //For Test define fixed values

#undef EXAMPLE_WIFI_SSID
#define EXAMPLE_WIFI_SSID "ASUR300_1"

#undef EXAMPLE_WIFI_PASS
#define EXAMPLE_WIFI_PASS "asusRTN53siva"

#undef HOST_IP_ADDR
#define HOST_IP_ADDR "192.168.1.3"

#endif

#undef PORT
#define PORT 3333

static uint16_t server_port_number = PORT;

#define SERVER_PORT_STRING_KEY "server_port"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

//static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

static int disconnect_count = 0;
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        //if connects then store it in flash
        esp_wifi_set_storage(WIFI_STORAGE_FLASH);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    case SYSTEM_EVENT_STA_LOST_IP:
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED. Reconnecting ... %d", event->event_id);
        if(disconnect_count > 20) {
            printf("Stopping the reconnection...May be the SSID or PASSPHRASE are incorrect\n");
            printf("\nPlease use the following command to set the <ssid> and <passphrase>\n");
            printf("   wifi-set <ssid> <passphrase> \n");
            printf("\nPress Ctrl + c to restart \n");
            break;
        }
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        disconnect_count++;
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}
static int suppress_print = 0;
static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
#if 0
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
#else
    wifi_config_t wifi_config_2;
    
    ESP_ERROR_CHECK( esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config_2));
    printf("Using ssid= %s, passphrase= %s Server Port number= %d\n",wifi_config_2.sta.ssid, wifi_config_2.sta.password, server_port_number);
    if((wifi_config_2.sta.ssid[0] == 0) || (wifi_config_2.sta.password[0] == 0)) {
        printf("ERROR ssid or passphrase not set\n");
        suppress_print = 1;
    }
    //wifi_config_t wifi_config;
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    //memcpy(wifi_config.sta.ssid, wifi_config_2.sta.ssid, 32);
    //memcpy(wifi_config.sta.password, wifi_config_2.sta.password, 64);
    strcpy((char*)wifi_config.sta.ssid, (char*)wifi_config_2.sta.ssid);
    strcpy((char*)wifi_config.sta.password, (char*)wifi_config_2.sta.password);
    wifi_config.sta.bssid_set = false;
    
#endif

    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}
static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;
    EventBits_t uxBits = 0;
    int count = 0;

    ESP_LOGI(TAG, "Waiting for AP (Access Point) connection...");
    if(suppress_print == 1) {
        printf("\nPlease use the following command to set the <ssid> and <passphrase>\n");
        printf("   wifi-set <ssid> <passphrase> \n");
    }

    //xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    while(uxBits == 0) {
        uxBits = xEventGroupWaitBits(wifi_event_group, bits, false, true, 1000/ portTICK_PERIOD_MS);
        count++;
        if(suppress_print == 0) {
            printf("Waiting for AP connection... %d seconds\n", count);
        }
    }

    ESP_LOGI(TAG, "Connected to AP");
}
static void init_tcp_send_info(void)
{
    tcp_send.rd_index = 0;
    tcp_send.wr_index = 0;
    tcp_send.wr_count = 0;
    if(tcp_send.lock)
      vSemaphoreDelete(tcp_send.lock);
    tcp_send.lock = xSemaphoreCreateBinary();
    assert(tcp_send.lock);
    xSemaphoreGive(tcp_send.lock);
    return;
}

//use this as maximum chunk size to send
static int max_d2h_size = 1024;

void tcp_msg_send_data(uint8_t *data, int size)
{
    struct tcp_send_info *t = &tcp_send;
    struct xQ_TcpPacket packet;
    packet.data_index = t->wr_index;
    packet.data_size  = size;

    if(size > max_d2h_size)
        max_d2h_size = size;

    //copy the data into tcp circular buffer
    int copy_count = size;

    //check if the data copy needs to wrap around
    if((t->wr_index + size) >= MAX_TCP_BUF)  {
      copy_count = MAX_TCP_BUF - t->wr_index;
      memcpy(&t->send_buf[t->wr_index], data, copy_count);
      copy_count = size - copy_count;
      t->wr_index = 0;
      data += copy_count;
    }
    //copy the remaining only if non-zero count
    if(copy_count > 0) {
      memcpy(&t->send_buf[t->wr_index], data, copy_count);
      t->wr_index += copy_count;
    }
    //update the write count protected
    xSemaphoreTake(t->lock, portMAX_DELAY);
    t->wr_count += size;  
    xSemaphoreGive(t->lock);

    //printf("Buf write %d, %d " , t->wr_index,t->wr_count);
    //printf("Buf in 0x%02X %02X %02X %02X", data[0],data[1],data[2],data[3]);
    //data = &t->send_buf[t->wr_index - size];
    //printf("Buf out 0x%02X %02X %02X %02X", data[0],data[1],data[2],data[3]);

    //send the message only after copying data
    xQueueSend(xHandleQueueTcp, ( void * )&packet, 0 );
    
    //wait for the data buffer have enough space for next pkt)
    TickType_t  start_count = xTaskGetTickCount();
    TickType_t  loop_count = start_count;
    while(t->wr_count > (MAX_TCP_BUF - max_d2h_size)) {
        vTaskDelay(10/portTICK_PERIOD_MS);
        //if it exceeds 1 second print a message
        if((xTaskGetTickCount() - loop_count) > (1000/portTICK_PERIOD_MS)) {
            loop_count = xTaskGetTickCount();
            printf("%s: Error waiting for TCP send- Timeout %d = \n", TAG, loop_count - start_count); 
        }
    }
    return;
}

static int get_tcp_send_data(uint8_t *data, int max_size)
{
    struct tcp_send_info *t = &tcp_send;
    int copy_count = 0;
    int wr_count = t->wr_count; //take current value to work with
    if(wr_count > 0)
    {
        if(max_size > wr_count)
           max_size = wr_count;
       
        copy_count = max_size;
        if((t->rd_index + max_size) >= MAX_TCP_BUF)  {
            copy_count = MAX_TCP_BUF - t->rd_index;
            memcpy(data, &t->send_buf[t->rd_index], copy_count);
            copy_count = max_size - copy_count;
            t->rd_index = 0;
            data += copy_count;
        }
        //copy the remaining only if non-zero count
        if(copy_count > 0) {
          memcpy(data, &t->send_buf[t->rd_index], copy_count);
          t->rd_index += copy_count;
        }
        
        copy_count = max_size;
        
        //update the write count protected
        xSemaphoreTake(t->lock, portMAX_DELAY);
        t->wr_count -= max_size;
        xSemaphoreGive(t->lock);

    }
    return copy_count;
}


static volatile int tcp_socket = -1;
int check_tcp_socket(void)
{
    if (tcp_socket < 0)
        return 0;
    else
        return 1;
}
void send_to_tcp_socket(uint8_t *buf, int size)
{
    if(check_tcp_socket())
    {
        int err = send(tcp_socket, buf, size, 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
            tcp_socket = -1;
            return;
        }
    }
    return;
    
}


static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    
    TickType_t  start_count = xTaskGetTickCount();


    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#define MAX_CONN_ATTEMPTS (10)

        int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket binded");

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        int connection_count = 0;
        //while((xTaskGetTickCount() - start_count)  < (MAX_CONN_ATTEMPTS*pdMS_TO_TICKS(1000))) {
        while(connection_count < MAX_CONN_ATTEMPTS) {
            
        struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
        uint addrLen = sizeof(sourceAddr);
        int sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            //wait 1 sec before trying connect again
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
            //break;
        } else {
            connection_count++;
            start_count = xTaskGetTickCount();
            ESP_LOGI(TAG, "Socket accepted");
        }

        tcp_socket = -1;

        //update the socket after it is connected
        tcp_socket = sock;
        BaseType_t xResult;
        struct xQ_TcpPacket rx_msg;
        while (1) {

            //if problem with socket break and restart
            if(tcp_socket < 0) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));

#if 0 //0=do not read
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }
#endif
            //vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            //also need to reset the send buffer
            init_tcp_send_info();
        }
        }
    }
    vTaskDelete(NULL);
}

extern void esp32_init_ql_spi(void);
extern int h2d_protocol_init(void);
extern signed portBASE_TYPE StartRtosTaskHost( void);

void init_ql_components(void)
{
    
    gpio_install_isr_service(0); //ESP_INTR_FLAG_DEFAULT);

    //enable SPI Master
    esp32_init_ql_spi();
    printf("%s :spi master initialized\n", TAG);

   // init h2d protocol
    h2d_protocol_init();
    printf("%s :h2d task created\n", TAG);

    //create ql host task
    StartRtosTaskHost();
    printf("%s :host task created\n", TAG);

    return;
}

uint16_t get_server_port_number(void)
{
    return server_port_number;
}

nvs_handle flash_handle;
void get_server_port_from_nvs(void)
{
    esp_err_t err;
    err = nvs_open("storage", NVS_READWRITE, &flash_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    err = nvs_get_u16(flash_handle, SERVER_PORT_STRING_KEY, &server_port_number);
    switch (err) {
        case ESP_OK:
            printf("Server Port Number = %d \n", server_port_number);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The %s value is not initialized yet\n", SERVER_PORT_STRING_KEY);
            server_port_number = PORT;
            printf("Using default Server Port Number %d \n", server_port_number);
            break;
        default :
            printf("Error (%s) reading nvs flash \n", esp_err_to_name(err));
            break;
    }

    return;
}
void set_server_port_in_nvs(uint16_t server_port)
{
    esp_err_t err;
    err = nvs_set_u16(flash_handle, SERVER_PORT_STRING_KEY, server_port);
    switch (err) {
        case ESP_OK:
            printf("Server Port Number = %d is stored \n", server_port);
            server_port_number = server_port;
            break;
        default :
            printf("Error (%s) writing to nvs flash \n", esp_err_to_name(err));
            break;
    }
    
    return;
}

void app_main()
{

    scli_init();
    diag_register_cli();
    wifi_register_cli();
    
    server_port_number = PORT;

    ESP_ERROR_CHECK( nvs_flash_init() );
    get_server_port_from_nvs();

    init_tcp_send_info();
    init_ql_components();

    initialise_wifi();
    wait_for_ip();

    /* Create queue for tcp_ip_task */
    xHandleQueueTcp = xQueueCreate( TCP_QUEUE_LENGTH, sizeof(struct xQ_TcpPacket) );
#define STACK_SIZE_TASK_TCP (8*1024)
#define PRIORITY_TASK_TCP   (configMAX_PRIORITIES - 6)

    //xTaskCreate(tcp_server_task, "tcp_server", 2*4096, NULL, 5, NULL);
    StackType_t *task_stack = (StackType_t *) heap_caps_calloc(1, STACK_SIZE_TASK_TCP, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    static StaticTask_t tcp_task_buf;
    
    assert(task_stack);
    xHandleTaskTcp  = xTaskCreateStatic(tcp_server_task, "tcp_server", STACK_SIZE_TASK_TCP, NULL, PRIORITY_TASK_TCP, task_stack, &tcp_task_buf);
    assert(xHandleTaskTcp );
    
//    init_tcp_send_info();

//    init_ql_components();
}
