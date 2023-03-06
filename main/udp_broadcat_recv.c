/*	UDP Broadcast Receiver

	This example code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "parameter.h"

extern MessageBufferHandle_t xMessageBufferRecv;

static const char *TAG = "RECV";

// Bradcast Receive Task
void udp_receive(void *pvParameters) {
	PARAMETER_t *task_parameter = pvParameters;
	PARAMETER_t param;
	memcpy((char *)&param, task_parameter, sizeof(PARAMETER_t));
	ESP_LOGI(TAG, "Start:param.port=%d param.ipv4=[%s]", param.port, param.ipv4);

	/* set up socket connection */
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(param.port);
	//addr.sin_addr.s_addr = htonl(INADDR_ANY); /* receive message from ANY */
	//addr.sin_addr.s_addr = inet_addr("0.0.0.0"); /* receive message from ANY */
	addr.sin_addr.s_addr = inet_addr(param.ipv4);

	/* create the socket */
	int fd;
	int ret;
	fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP ); // Create a UDP socket.
	LWIP_ASSERT("fd >= 0", fd >= 0);

#if 0
	/* set option */
	int broadcast=1;
	ret = lwip_setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);
	LWIP_ASSERT("ret >= 0", ret >= 0);
#endif

	/* bind socket */
	ret = lwip_bind(fd, (struct sockaddr *)&addr, sizeof(addr));
	LWIP_ASSERT("ret >= 0", ret >= 0);

	struct sockaddr_in senderInfo;
	//char senderstr[16];
	PAYLOAD_t payload;

	/* read socket */
	while(1) {
		socklen_t senderInfoLen = sizeof(senderInfo);
		payload.length = lwip_recvfrom(fd, payload.buffer, sizeof(payload.buffer), 0, (struct sockaddr*)&senderInfo, &senderInfoLen);
		LWIP_ASSERT("payload.length > 0", payload.length > 0);
		payload.type = DATA;
		ESP_LOGD(TAG, "lwip_recv length=%d", payload.length);
		ESP_LOGD(TAG, "lwip_recv buffer=[%.*s]", payload.length, payload.buffer);
		//inet_ntop(AF_INET, &senderInfo.sin_addr, senderstr, sizeof(senderstr));
		//ESP_LOGI(TAG, "recvfrom : %s, port=%d", senderstr, ntohs(senderInfo.sin_port));
		size_t xBytesSent = xMessageBufferSend(xMessageBufferRecv, &payload, sizeof(payload), 1000/portTICK_PERIOD_MS);
		if( xBytesSent != sizeof(payload)) {
			ESP_LOGE(TAG, "xMessageBufferSend fail");
		}
	}

	/* close socket. Don't reach here. */
	ret = lwip_close(fd);
	LWIP_ASSERT("ret == 0", ret == 0);
	vTaskDelete( NULL );

}

