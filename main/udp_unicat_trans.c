/*	UDP Unicast Transmitter

	This example code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/message_buffer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "mdns.h"
#include "lwip/sockets.h"

#include "parameter.h"

extern MessageBufferHandle_t xMessageBufferSend;

static const char *TAG = "SEND";

#define _DEBUG_ 0

#if _DEBUG_
/* these strings match mdns_ip_protocol_t enumeration */
static const char * ip_protocol_str[] = {"V4", "V6", "MAX"};
#endif

static void mdns_print_results(mdns_result_t *results, char * hostname, char * ip, uint16_t *port)
{
	mdns_result_t *r = results;
	mdns_ip_addr_t *a = NULL;
#if _DEBUG_
	int i = 1;
#endif
	while (r) {
#if _DEBUG_
		esp_netif_get_desc(r->esp_netif);
		printf("%d: Interface: %s, Type: %s, TTL: %"PRIu32"\n",
			i++, esp_netif_get_desc(r->esp_netif), ip_protocol_str[r->ip_protocol], r->ttl);

#endif
		if (r->instance_name) {
#if _DEBUG_
			printf("  PTR : %s.%s.%s\n", r->instance_name, r->service_type, r->proto);
#endif
		}
		if (r->hostname) {
#if _DEBUG_
			printf("  SRV : %s.local:%u\n", r->hostname, r->port);
#endif
			strcpy(hostname, r->hostname);
			*port = r->port;
		}
		if (r->txt_count) {
#if _DEBUG_
			printf("  TXT : [%zu] ", r->txt_count);
			for (int t = 0; t < r->txt_count; t++) {
				printf("%s=%s(%d); ", r->txt[t].key, r->txt[t].value ? r->txt[t].value : "NULL", r->txt_value_len[t]);
			}
			printf("\n");
#endif
		}
		a = r->addr;
		while (a) {
			if (a->addr.type == ESP_IPADDR_TYPE_V6) {
#if _DEBUG_
				printf("  AAAA: " IPV6STR "\n", IPV62STR(a->addr.u_addr.ip6));
#endif
			} else {
#if _DEBUG_
				printf("  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));
#endif
				sprintf(ip, IPSTR, IP2STR(&(a->addr.u_addr.ip4)));
			}
			a = a->next;
		}
		r = r->next;
	}
}

static int query_mdns_service(const char * service_name, const char * proto, char * hostname, char * ip, uint16_t *port)
{
	ESP_LOGI(__FUNCTION__, "Query PTR: %s.%s.local", service_name, proto);

	mdns_result_t * results = NULL;
	esp_err_t err = mdns_query_ptr(service_name, proto, 3000, 20,  &results);
	if(err){
		ESP_LOGE(__FUNCTION__, "Query Failed: %s", esp_err_to_name(err));
		return false;
	}
	if(!results){
		ESP_LOGW(__FUNCTION__, "No results found!");
		return false;
	}

	//char hostname[64];
	//char ip[16];
	//uint16_t port;
	mdns_print_results(results, hostname, ip, port);
	ESP_LOGI(__FUNCTION__, "hostname=[%s] ip=[%s] port=[%d]", hostname, ip, *port);
	mdns_query_results_free(results);
	return true;
}

int connect_peer(struct sockaddr_in *addr, uint16_t port, char *ipv4) {
	//struct sockaddr_in addr;
	//memset(addr, 0, sizeof(addr));
	addr->sin_family = AF_INET;
	addr->sin_port = htons(port);
	//addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); /* send message to 255.255.255.255 */
	//addr.sin_addr.s_addr = inet_addr("255.255.255.255"); /* send message to 255.255.255.255 */
	//addr.sin_addr.s_addr = inet_addr(param.ipv4);
	addr->sin_addr.s_addr = inet_addr(ipv4);

	/* create the socket */
	int fd;
	fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP ); // Create a UDP socket.
	LWIP_ASSERT("fd >= 0", fd >= 0);
	return fd;
}

// UDP Send Task
void udp_trans(void *pvParameters) {
	PARAMETER_t *task_parameter = pvParameters;
	PARAMETER_t param;
	memcpy((char *)&param, task_parameter, sizeof(PARAMETER_t));
	ESP_LOGI(TAG, "Start:param.port=%d param.ipv4=[%s]", param.port, param.ipv4);

#if 0
	/* set up socket connection */
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(param.port);
	//addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); /* send message to 255.255.255.255 */
	//addr.sin_addr.s_addr = inet_addr("255.255.255.255"); /* send message to 255.255.255.255 */
	addr.sin_addr.s_addr = inet_addr(param.ipv4);

	/* create the socket */
	int fd;
	int ret;
	fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP ); // Create a UDP socket.
	LWIP_ASSERT("fd >= 0", fd >= 0);
#endif

	// Socket stuff
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	int fd = 0;
	//int ret;
	int connected = false;

	// mDNS stuff
	char service_type[64];
	//sprintf(service_type, "_service_%d", CONFIG_UDP_PORT); //prepended with underscore
	sprintf(service_type, "_service_%d", param.port); //prepended with underscore
	char hostname[64];
	char ipv4[16];
	uint16_t port;
	

	PAYLOAD_t payload;
	TickType_t last_sendto = xTaskGetTickCount();
	while(1) {
		size_t received = xMessageBufferReceive(xMessageBufferSend, &payload, sizeof(payload), portMAX_DELAY);
		ESP_LOGD(TAG, "xMessageBufferReceive received=%d", received);
		if (received > 0) {
			ESP_LOGD(TAG, "payload.type=%d", payload.type);
			if (payload.type == PING) {
				if ((xTaskGetTickCount() - last_sendto) < 100) {
					ESP_LOGD(TAG, "Ignore ping requests");
					continue;
				}
				//int query = query_mdns_service("_myservice", "_udp", hostname, ipv4, &port);
				int query = query_mdns_service(service_type, "_udp", hostname, ipv4, &port);
				ESP_LOGD(TAG, "connected=%d query=%d", connected, query);
				if (query) {
					if (connected) {
						ESP_LOGI(TAG, "already connected with [%s]", hostname);
					} else {
						/* set up socket connection */
						fd = connect_peer(&addr, port, ipv4);
						ESP_LOGW(TAG, "new connection with [%s]", hostname);
						connected = true;
					}
				} else {
					if (connected) {
						ESP_LOGW(TAG, "lost contact with [%s]", hostname);
						int ret = lwip_close(fd);
						LWIP_ASSERT("ret == 0", ret == 0);
						fd = 0;
						connected = false;
					} else {
						ESP_LOGI(TAG, "waiting peer");
					}
				}
			} else if (payload.type == DATA) {
				if (connected) {
					//ESP_LOGI(TAG, "xMessageBufferReceive buffer=[%.*s]",payload.length, payload.buffer);
					ESP_LOGI(TAG, "USB-->Wireless [%.*s]",payload.length, payload.buffer);
					int ret = lwip_sendto(fd, payload.buffer, payload.length, 0, (struct sockaddr *)&addr, sizeof(addr));
					ESP_LOGD(TAG, "lwip_sendto payload.length=%d ret=%d", payload.length, ret);
					//LWIP_ASSERT("ret == payload.length", ret == payload.length);
					if (ret != payload.length) {
						ESP_LOGW(TAG, "lwip_sendto fail. payload.length=%d ret=%d", payload.length, ret);
					}
					last_sendto = xTaskGetTickCount();
				} else {
					ESP_LOGW(TAG, "not conneted peer");
				}
			}
		} else {
			ESP_LOGE(TAG, "xMessageBufferReceive fail");
			break;
		}
	}

	/* close socket. Don't reach here. */
#if 0
	ret = lwip_close(fd);
	LWIP_ASSERT("ret == 0", ret == 0);
#endif
	vTaskDelete( NULL );

}

