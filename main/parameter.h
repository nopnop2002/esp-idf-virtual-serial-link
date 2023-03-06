typedef struct {
	uint16_t port;
	char ipv4[20]; // xxx.xxx.xxx.xxx
	char espnow_pmk[20]; // The length of ESPNOW primary master must be 16 bytes.
	char espnow_lmk[20]; // The length of ESPNOW local master must be 16 bytes.
	int espnow_channel;
	int espnow_send_len;
	bool espnow_enable_long_range;
} PARAMETER_t;

#define MAX_PAYLOAD 128
#define PING 1
#define DATA 2

typedef struct {
	uint8_t type;
	size_t length;
	char buffer[MAX_PAYLOAD];
} PAYLOAD_t;
