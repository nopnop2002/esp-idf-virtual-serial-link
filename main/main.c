/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/message_buffer.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h" // esp_read_mac
#include "mdns.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
//#include "sdkconfig.h"

#include "parameter.h"
#include "esp_now.h"
#include "espnow_task.h"

static const char *TAG = "MAIN";

#if CONFIG_WIFI_MODE
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;
#endif // CONFIG_WIFI_MODE

int line_status = false;

MessageBufferHandle_t xMessageBufferSend;
MessageBufferHandle_t xMessageBufferRecv;
QueueHandle_t xQueueESPNOWSend;
QueueHandle_t xQueueESPNOWRecv;

#if CONFIG_WIFI_MODE
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG,"connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

esp_err_t wifi_init_sta(void)
{
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
		ESP_EVENT_ANY_ID,
		&event_handler,
		NULL,
		&instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
		IP_EVENT_STA_GOT_IP,
		&event_handler,
		NULL,
		&instance_got_ip));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_ESP_WIFI_SSID,
			.password = CONFIG_ESP_WIFI_PASSWORD,
			/* Setting a password implies station will connect to all security modes including WEP/WPA.
			 * However these modes are deprecated and not advisable to be used. Incase your Access point
			 * doesn't support WPA2, these mode can be enabled by commenting below line */
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,

			.pmf_cfg = {
				.capable = true,
				.required = false
			},
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	esp_err_t ret_value = ESP_OK;
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
		WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
		ret_value = ESP_FAIL;
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
		ret_value = ESP_FAIL;
	}

	/* The event will not be processed after unregister */
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
	vEventGroupDelete(s_wifi_event_group);
	return ret_value;
}
#endif // CONFIG_WIFI_MODE

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
	/* initialization */
	size_t rx_size = 0;

	/* read */
	uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
	esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
	ESP_LOGD(__FUNCTION__, "CONFIG_TINYUSB_CDC_RX_BUFSIZE=%d rx_size=%d", CONFIG_TINYUSB_CDC_RX_BUFSIZE, rx_size);
	if (ret == ESP_OK) {
		ESP_LOGD(__FUNCTION__, "Data from channel=%d rx_size=%d", itf, rx_size);
		//ESP_LOG_BUFFER_HEXDUMP(__FUNCTION__, buf, rx_size, ESP_LOG_INFO);
#if CONFIG_WIFI_MODE
		PAYLOAD_t payload;
		payload.length = rx_size;
		payload.type = DATA;
		memcpy(payload.buffer, buf, rx_size);
		size_t send = xMessageBufferSendFromISR(xMessageBufferSend, &payload, sizeof(payload), NULL);
		if (send != sizeof(payload)) ESP_LOGW(__FUNCTION__, "xMessageBufferSendFromISR fail");
#endif

#if CONFIG_ESPNOW_MODE
		example_espnow_event_t evt;
		evt.len = rx_size;
		evt.id = EXAMPLE_ESPNOW_SEND_RQ;
		memcpy(evt.payload, buf, rx_size);
		if (xQueueSendFromISR(xQueueESPNOWSend, &evt, NULL) != pdTRUE) {
			ESP_LOGW(__FUNCTION__, "xQueueSendFromISR fail");
		}

#endif

	} else {
		ESP_LOGE(__FUNCTION__, "tinyusb_cdcacm_read fail");
	}

#if 0
	/* write back */
	tinyusb_cdcacm_write_queue(itf, buf, rx_size);
	tinyusb_cdcacm_write_flush(itf, 0);
#endif
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
	int dtr = event->line_state_changed_data.dtr;
	int rts = event->line_state_changed_data.rts;
	ESP_LOGI(__FUNCTION__, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
	if (dtr == 1 && rts == 1) {
		ESP_LOGW("USB", "Connected");
		line_status = true;
	} else {
		ESP_LOGW("USB", "Releaseed");
		line_status = false;
	}
}


#if CONFIG_WIFI_MODE
// Timer call back
static void timer_udp_cb(TimerHandle_t xTimer)
{
	//ESP_LOGI(__FUNCTION__, "timer_udp_cb");
	PAYLOAD_t payload;
	payload.length = 0;
	payload.type = PING;
	xMessageBufferSendFromISR(xMessageBufferSend, &payload, sizeof(payload), NULL);
}

/** Generate host name based on sdkconfig, optionally adding a portion of MAC address to it.
 *	@return host name string allocated from the heap
 */
static char* generate_hostname(void)
{
	uint8_t mac[6];
	char   *hostname;
	esp_read_mac(mac, ESP_MAC_WIFI_STA);
	if (-1 == asprintf(&hostname, "%s-%02X%02X%02X", CONFIG_MDNS_HOSTNAME, mac[3], mac[4], mac[5])) {
		abort();
	}
	return hostname;
}

#define USE_TEXT_ITEM 0

static void initialise_mdns(void)
{
	char * hostname = generate_hostname();

	//initialize mDNS
	ESP_ERROR_CHECK( mdns_init() );
	//set mDNS hostname (required if you want to advertise services)
	ESP_ERROR_CHECK( mdns_hostname_set(hostname) );
	ESP_LOGI(__FUNCTION__, "mdns hostname set to: [%s]", hostname);
	free(hostname);

#if 0
	//set default mDNS instance name
	ESP_ERROR_CHECK( mdns_instance_name_set(CONFIG_MDNS_INSTANCE) );
	ESP_LOGI(__FUNCTION__, "mdns instance name set to: [%s]", CONFIG_MDNS_INSTANCE);
#endif

#if USE_TEXT_ITEM
	//structure with TXT records
	mdns_txt_item_t serviceTxtData[3] = {
		{"board", "esp32"},
		{"u", "user"},
		{"p", "password"}
	};
#endif

	//initialize service
	char service_type[64];
	sprintf(service_type, "_service_%d", CONFIG_UDP_PORT); //prepended with underscore
	ESP_LOGI(__FUNCTION__, "service_type=[%s]", service_type);
	//ESP_ERROR_CHECK( mdns_service_add("ESP32-UDP-Server", "_myservice", "_udp", CONFIG_UDP_PORT, serviceTxtData, 3) );
#if USE_TEXT_ITEM
	ESP_ERROR_CHECK( mdns_service_add("esp32-udp", service_type, "_udp", CONFIG_UDP_PORT, serviceTxtData, 3) );
#else
	ESP_ERROR_CHECK( mdns_service_add("esp32-udp", service_type, "_udp", CONFIG_UDP_PORT, NULL, 0) );
#endif
}

void udp_trans(void *pvParameters);
void udp_receive(void *pvParameters);
#endif // CONFIG_WIFI_MODE

#if CONFIG_ESPNOW_MODE
static void timer_espnow_cb(TimerHandle_t xTimer)
{
	example_espnow_event_t evt;
	evt.id = EXAMPLE_ESPNOW_PING_RQ;
	if (xQueueSend(xQueueESPNOWSend, &evt, 100) != pdTRUE) {
		ESP_LOGW(__FUNCTION__, "Send send queue fail");
	}
}

void espnow_task(void *pvParameter);
#endif //ESPNOW_MODE


void app_main(void)
{
	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

#if CONFIG_WIFI_MODE
	// Initialize WiFi
	wifi_init_sta();

	// Initialize mDNS
	initialise_mdns();
#endif // CONFIG_WIFI_MODE

	ESP_LOGI(TAG, "USB initialization");
	const tinyusb_config_t tusb_cfg = {};
#if 0
	const tinyusb_config_t tusb_cfg = {
		.device_descriptor = NULL,
		.string_descriptor = NULL,
		.external_phy = false,
		.configuration_descriptor = NULL,
	};
#endif

	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	tinyusb_config_cdcacm_t acm_cfg = {
		.usb_dev = TINYUSB_USBDEV_0,
		.cdc_port = TINYUSB_CDC_ACM_0,
		//.rx_unread_buf_sz = 64,
		//.rx_unread_buf_sz = 256,
		.rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
		.callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
		.callback_rx_wanted_char = NULL,
		.callback_line_state_changed = NULL,
		.callback_line_coding_changed = NULL
	};

	ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
	/* the second way to register a callback */
	ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
		TINYUSB_CDC_ACM_0,
		CDC_EVENT_LINE_STATE_CHANGED,
		&tinyusb_cdc_line_state_changed_callback));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
	acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
	ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
	ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
		TINYUSB_CDC_ACM_1,
		CDC_EVENT_LINE_STATE_CHANGED,
		&tinyusb_cdc_line_state_changed_callback));
#endif

	ESP_LOGI(TAG, "USB initialization DONE");

	// Create MessageBuffer
	xMessageBufferSend = xMessageBufferCreate(sizeof(PAYLOAD_t)*30);
	configASSERT( xMessageBufferSend );
	xMessageBufferRecv = xMessageBufferCreate(sizeof(PAYLOAD_t)*30);
	configASSERT( xMessageBufferRecv );
	xQueueESPNOWSend = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
	configASSERT( xQueueESPNOWSend );
	xQueueESPNOWRecv = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
	configASSERT( xQueueESPNOWRecv );

#if CONFIG_WIFI_MODE
	// Create Timer
	TimerHandle_t timerHandle = xTimerCreate("Trigger", 5000/portTICK_PERIOD_MS, pdTRUE, NULL, timer_udp_cb);
	configASSERT( timerHandle );
#endif

#if CONFIG_ESPNOW_MODE
	// Create Timer
	TimerHandle_t timerHandle = xTimerCreate("Trigger", 5000/portTICK_PERIOD_MS, pdTRUE, NULL, timer_espnow_cb);
	configASSERT( timerHandle );
#endif

	if (xTimerStart(timerHandle, 0) != pdPASS) {
		ESP_LOGE(TAG, "Unable to start Timer");
		while(1) { vTaskDelay(1); }
	} else {
		ESP_LOGI(TAG, "Success to start Timer");
	}

#if CONFIG_WIFI_MODE
	// Start Transmitter
	PARAMETER_t param1;
	param1.port = CONFIG_UDP_PORT;
	strcpy(param1.ipv4, "0.0.0.0"); /* not use */
	xTaskCreate(udp_trans, "TRANS", 1024*4, (void *)&param1, 2, NULL);

	// Start Receiver
	PARAMETER_t param;
	param.port = CONFIG_UDP_PORT;
	strcpy(param.ipv4, "0.0.0.0"); /* receive message from ANY */
	xTaskCreate(udp_receive, "RECV", 1024*4, (void *)&param, 2, NULL);

	PAYLOAD_t payload;
	while(1) {
		size_t received = xMessageBufferReceive(xMessageBufferRecv, &payload, sizeof(payload), portMAX_DELAY);
		if (received > 0) {
			//ESP_LOGI(TAG, "xMessageBufferReceive buffer=[%.*s]", payload.length, payload.buffer);
			ESP_LOGI(TAG, "Wireless-->USB [%.*s]", payload.length, payload.buffer);
			if (line_status) {
				tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)payload.buffer, payload.length);
				if (payload.length != 64) tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
			} else {
				ESP_LOGW(TAG, "USB not online");
			}
		}
	}
#endif // CONFIG_WIFI_MODE

#if CONFIG_ESPNOW_MODE
	// Start ESPNOW Take
	PARAMETER_t param1;
	strcpy(param1.espnow_pmk, CONFIG_ESPNOW_PMK);
	strcpy(param1.espnow_lmk, CONFIG_ESPNOW_LMK);
	param1.espnow_channel = CONFIG_ESPNOW_CHANNEL;
	param1.espnow_send_len = CONFIG_ESPNOW_SEND_LEN;
	param1.espnow_enable_long_range = false;
#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
	param1.espnow_enable_long_range = true;
#endif
	xTaskCreate(espnow_task, "ESPNOW", 1024*4, (void *)&param1, 4, NULL);

	example_espnow_event_t evt_recv;
	while(1) {
		xQueueReceive(xQueueESPNOWRecv, &evt_recv, portMAX_DELAY);
		ESP_LOGD(TAG, "xQueueReceive evt_recv.len=%d", evt_recv.len);
		//ESP_LOG_BUFFER_HEXDUMP(TAG, evt_recv.payload, evt_recv.len, ESP_LOG_INFO);
		ESP_LOGI(TAG, "Wireless-->USB [%.*s]", evt_recv.len, evt_recv.payload);
		if (line_status) {
			tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, evt_recv.payload, evt_recv.len);
			if (evt_recv.len != 64) tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
		} else {
			ESP_LOGW(TAG, "USB not online");
		}
	}

#endif
}
