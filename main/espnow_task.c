/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, can send and receive to each other.
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h" // for MACSTR
#include "esp_crc.h" // for esp_crc16_le()

#include "parameter.h"
#include "esp_now.h"
#include "espnow_task.h"

#define ESPNOW_MAXDELAY 512

extern QueueHandle_t xQueueESPNOWSend;
extern QueueHandle_t xQueueESPNOWRecv;

static const char *TAG = "ESPNOW";

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static esp_err_t espnow_init(PARAMETER_t param);
static void espnow_deinit(example_espnow_send_param_t *send_param);

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
	example_espnow_event_t evt;
	example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

	if (tx_info == NULL) {
		ESP_LOGE(__FUNCTION__, "Send cb arg error");
		return;
	}

	evt.id = EXAMPLE_ESPNOW_SEND_CB;
	memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
	send_cb->status = status;
	if (xQueueSend(xQueueESPNOWSend, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
		ESP_LOGW(__FUNCTION__, "xQueueSend fail");
	}
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
	ESP_LOGD(__FUNCTION__, "len=%d", len);
	example_espnow_event_t evt;
	example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
	uint8_t * mac_addr = recv_info->src_addr;

	if (mac_addr == NULL || data == NULL || len <= 0) {
		ESP_LOGE(__FUNCTION__, "Receive cb arg error");
		return;
	}

	evt.id = EXAMPLE_ESPNOW_RECV_CB;
	memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
	recv_cb->data = malloc(len);
	if (recv_cb->data == NULL) {
		ESP_LOGE(__FUNCTION__, "Malloc receive data fail");
		return;
	}
	memcpy(recv_cb->data, data, len);
	recv_cb->data_len = len;
	if (xQueueSend(xQueueESPNOWSend, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
		ESP_LOGW(__FUNCTION__, "xQueueSend fail");
		free(recv_cb->data);
	}
}

/* Parse received ESPNOW data. */
//int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
	example_espnow_data_t *buf = (example_espnow_data_t *)data;
	uint16_t crc, crc_cal = 0;

	if (data_len < sizeof(example_espnow_data_t)) {
		ESP_LOGE(__FUNCTION__, "Receive ESPNOW data too short, len:%d", data_len);
		return -1;
	}

	*state = buf->state;
	*seq = buf->seq_num;
	*magic = buf->magic;
	crc = buf->crc;
	buf->crc = 0;
	crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

	if (crc_cal == crc) {
		return buf->type;
	}

	ESP_LOGE(__FUNCTION__, "CRC fail. 0x%x-->0x%x", crc_cal, crc);
	return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param, uint8_t *payload, int payload_length)
{
	ESP_LOGD(__FUNCTION__, "payload_length=%d", payload_length);
	example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

	assert(send_param->len >= sizeof(example_espnow_data_t));

	buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
	buf->state = send_param->state;
	buf->seq_num = s_example_espnow_seq[buf->type]++;
	buf->crc = 0;
	buf->magic = send_param->magic;
	memcpy(buf->payload, payload, payload_length);
	buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

void espnow_task(void *pvParameters)
{
	PARAMETER_t *task_parameter = pvParameters;
	PARAMETER_t param;
	memcpy((char *)&param, task_parameter, sizeof(PARAMETER_t));
	ESP_LOGI(TAG, "Start:param.espnow_pmk=[%s]", param.espnow_pmk);
	ESP_LOGI(TAG, "Start:param.espnow_lmk=[%s]", param.espnow_lmk);
	ESP_LOGI(TAG, "Start:param.espnow_channel=[%d]", param.espnow_channel);
	ESP_LOGI(TAG, "Start:param.espnow_send_len=[%d]", param.espnow_send_len);
	ESP_LOGI(TAG, "Start:param.espnow_enable_long_range=[%d]", param.espnow_enable_long_range);

	example_espnow_event_t evt_send;
	uint8_t recv_state = 0;
	uint16_t recv_seq = 0;
	uint32_t recv_magic = 0;
	int ret;
	/*
	ESPNOW can carry payloads of up to 250 bytes
	This sample uses a 10 byte header.
	typedef struct {
		uint8_t type;		//Broadcast or unicast ESPNOW data.
		uint8_t state;		//Indicate that if has received broadcast ESPNOW data or not.
		uint16_t seq_num;	//Sequence number of ESPNOW data.
		uint16_t crc;		//CRC16 value of ESPNOW data.
		uint32_t magic;		//Magic number which is used to determine which device to send unicast ESPNOW data.
		uint8_t payload[0];	//Real payload of ESPNOW data.
	} __attribute__((packed)) example_espnow_data_t;
	*/
	uint8_t payload[240];
	int payload_length = param.espnow_send_len - sizeof(example_espnow_data_t);
	ESP_LOGI(TAG, "payload_length=%d", payload_length);

	// Initialize ESPNOW
	ESP_ERROR_CHECK(espnow_init(param));

	/* Initialize broadcast parameters. */
	example_espnow_send_param_t *broadcast = NULL;

	broadcast = malloc(sizeof(example_espnow_send_param_t));
	if (broadcast == NULL) {
		ESP_LOGE(TAG, "Malloc broadcast parameter fail");
		espnow_deinit(broadcast);
		vTaskDelete(NULL);
	}
	memset(broadcast, 0, sizeof(example_espnow_send_param_t));
	broadcast->unicast = false;
	broadcast->broadcast = true;
	broadcast->state = 0;
	//broadcast->magic = param.espnow_channel;
	broadcast->magic = 0;
	//broadcast->count = CONFIG_ESPNOW_SEND_COUNT;
	//broadcast->delay = CONFIG_ESPNOW_SEND_DELAY;
	broadcast->len = param.espnow_send_len;
	broadcast->buffer = malloc(param.espnow_send_len);
	if (broadcast->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc broadcast->buffer fail");
		espnow_deinit(broadcast);
		vTaskDelete(NULL);
	}
	memcpy(broadcast->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);

	/* Initialize sending parameters. */
	example_espnow_send_param_t *send_param = NULL;

	send_param = malloc(sizeof(example_espnow_send_param_t));
	if (send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		espnow_deinit(send_param);
		vTaskDelete(NULL);
	}
	memset(send_param, 0, sizeof(example_espnow_send_param_t));
	send_param->unicast = true;
	send_param->broadcast = false;
	send_param->state = 0;
	//send_param->magic = param.espnow_channel;
	send_param->magic = 0;
	//send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	//send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	send_param->len = param.espnow_send_len;
	send_param->buffer = malloc(param.espnow_send_len);
	if (send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send_param->buffer fail");
		espnow_deinit(broadcast);
		espnow_deinit(send_param);
		vTaskDelete(NULL);
	}
	memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);

	/* Start sending broadcast ESPNOW data. */
	example_espnow_data_prepare(send_param, payload, payload_length);
	if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
		ESP_LOGE(TAG, "Send error");
		espnow_deinit(broadcast);
		espnow_deinit(send_param);
		vTaskDelete(NULL);
	}

	while (xQueueReceive(xQueueESPNOWSend, &evt_send, portMAX_DELAY) == pdTRUE) {
		if (evt_send.id == EXAMPLE_ESPNOW_PING_RQ) {
			ESP_LOGD(TAG, "EXAMPLE_ESPNOW_PING_RQ");
			memset(payload, 0, sizeof(payload));
			example_espnow_data_prepare(broadcast, payload, payload_length);
			if (esp_now_send(broadcast->dest_mac, broadcast->buffer, broadcast->len) != ESP_OK) {
				ESP_LOGE(TAG, "broadcast send error");
				espnow_deinit(broadcast);
				espnow_deinit(send_param);
				vTaskDelete(NULL);
			}

		} else if (evt_send.id == EXAMPLE_ESPNOW_SEND_RQ) {
			ESP_LOGD(TAG, "EXAMPLE_ESPNOW_SEND_RQ");
			ESP_LOGD(TAG, "send_param->dest_mac "MACSTR, MAC2STR(send_param->dest_mac));
			if (memcmp(s_example_broadcast_mac, send_param->dest_mac, sizeof(send_param->dest_mac)) == 0) {
				ESP_LOGW(TAG, "Not Connected peer");
			} else {
				ESP_LOGD(TAG, "Send to "MACSTR, MAC2STR(send_param->dest_mac));
				//ESP_LOG_BUFFER_HEXDUMP(TAG, evt_send.payload, evt_send.len, ESP_LOG_INFO);
				if (evt_send.len > 240) {
					ESP_LOGE(TAG, "packet size [%d] too long. ESPNOW can carry payloads of up to 250 bytes", send_param->len);
				} else {
					ESP_LOGI(TAG, "USB-->Wireless [%.*s]", evt_send.len, (char*)evt_send.payload);
					memset(payload, 0, sizeof(payload));
					memcpy(payload, evt_send.payload, evt_send.len);
					example_espnow_data_prepare(send_param, payload, payload_length);
					//ESP_LOG_BUFFER_HEXDUMP(TAG, send_param->buffer, send_param->len, ESP_LOG_INFO);
					if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
						ESP_LOGE(TAG, "send_param send error");
						espnow_deinit(broadcast);
						espnow_deinit(send_param);
						vTaskDelete(NULL);
					}
				}
			}

		} else if (evt_send.id == EXAMPLE_ESPNOW_SEND_CB) {
			ESP_LOGD(TAG, "EXAMPLE_ESPNOW_SEND_CB");
			example_espnow_event_send_cb_t *send_cb = &evt_send.info.send_cb;
			ESP_LOGD(TAG, "Send data to ["MACSTR"], status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
			if (send_cb->status != 0) {
				ESP_LOGE(TAG, "Send fail to ["MACSTR"], status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
			}
			//bool is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

		} else if (evt_send.id == EXAMPLE_ESPNOW_RECV_CB) {
			ESP_LOGD(TAG, "EXAMPLE_ESPNOW_RECV_CB");
			example_espnow_event_recv_cb_t *recv_cb = &evt_send.info.recv_cb;

			ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
			ESP_LOGD(TAG, "ret=%d recv_seq=%d", ret, recv_seq);
			if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
				ESP_LOGD(TAG, "Receive %dth broadcast data from: ["MACSTR"]", recv_seq, MAC2STR(recv_cb->mac_addr));

				/* If MAC address does not exist in peer list, add it to peer list. */
				if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
					esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
					if (peer == NULL) {
						ESP_LOGE(TAG, "Malloc peer information fail");
						espnow_deinit(broadcast);
						espnow_deinit(send_param);
						vTaskDelete(NULL);
					}
					memset(peer, 0, sizeof(esp_now_peer_info_t));
					peer->channel = param.espnow_channel;
					peer->ifidx = ESPNOW_WIFI_IF;
					peer->encrypt = true;
					memcpy(peer->lmk, param.espnow_lmk, ESP_NOW_KEY_LEN);
					memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
					ESP_ERROR_CHECK( esp_now_add_peer(peer) );
					ESP_LOGW(TAG, "esp_now_add_peer ["MACSTR"]", MAC2STR(recv_cb->mac_addr));
					free(peer);
				} else {
					ESP_LOGD(TAG, "esp_now_is_peer_exist ["MACSTR"]", MAC2STR(recv_cb->mac_addr));
				}
				memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);

			} else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
				ESP_LOGD(TAG, "Receive %dth unicast data from: ["MACSTR"]", recv_seq, MAC2STR(recv_cb->mac_addr));
				example_espnow_data_t *buf = (example_espnow_data_t *)recv_cb->data;
				ESP_LOGD(TAG, "payload_length=%d", payload_length);
				//ESP_LOG_BUFFER_HEXDUMP(TAG, buf->payload, payload_length, ESP_LOG_INFO);
				example_espnow_event_t evt_recv;
				memcpy(evt_recv.payload, buf->payload, payload_length);
				evt_recv.len = payload_length;
				if (xQueueSend(xQueueESPNOWRecv, &evt_recv, 100) != pdTRUE) {
					ESP_LOGE(TAG, "xQueueSend fail");
				}

			} else {
				ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
			}
			free(recv_cb->data);
		} else {
			ESP_LOGE(TAG, "Callback type error: %d", evt_send.id);
		}
	}
}


static esp_err_t espnow_init(PARAMETER_t param)
{
	ESP_LOGI(TAG, "Start espnow_init");
	ESP_LOGI(TAG, "espnow_channel=%d", param.espnow_channel);
	ESP_LOGI(TAG, "espnow_enable_long_range=%d", param.espnow_enable_long_range);

	/* WiFi should start before using ESPNOW */
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_LOGI(__FUNCTION__, "WIFI_MODE_AP=%d", WIFI_MODE_AP);
	ESP_LOGI(__FUNCTION__, "WIFI_MODE_STA=%d", WIFI_MODE_STA);
	ESP_LOGI(__FUNCTION__, "ESPNOW_WIFI_MODE=%d", ESPNOW_WIFI_MODE);
	ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
	ESP_ERROR_CHECK( esp_wifi_start());

	/* Change WiFi primary channel */
	/* https://esp32.com/viewtopic.php?t=18291 */
	uint8_t primary;
	wifi_second_chan_t second;
	ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &second));
	ESP_LOGI(TAG, "current primary channel=%d", primary);
	primary = param.espnow_channel;
	ESP_ERROR_CHECK(esp_wifi_set_channel(primary, WIFI_SECOND_CHAN_NONE));
	ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &second));
	ESP_LOGI(TAG, "new primary channel=%d", primary);

	/* Set to long range */
	if (param.espnow_enable_long_range) {
		ESP_LOGW(TAG, "Set to long range");
		ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
	}

	/* Initialize ESPNOW and register sending and receiving callback function. */
	ESP_ERROR_CHECK( esp_now_init() );
	ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
	ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

	/* Set primary master key. */
	ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)param.espnow_pmk) );

	/* Add broadcast peer information to peer list. */
	esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
	if (peer == NULL) {
		ESP_LOGE(__FUNCTION__, "Malloc peer information fail");
		esp_now_deinit();
		return ESP_FAIL;
	}

	// esp_now_add_peer() to add the device to the paired device list before you send data to this device. 
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->channel = param.espnow_channel;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) );
	free(peer);
	ESP_LOGI(TAG, "Finish espnow_init");
	return ESP_OK;
}

static void espnow_deinit(example_espnow_send_param_t *param)
{
	if (param->buffer) free(param->buffer);
	if (param) free(param);
	//vQueueDelete(xQueueESPNOWSend);
	esp_now_deinit();
}

