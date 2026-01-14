typedef enum {
	PING,
	DATA
} type_t;

#define MAX_PAYLOAD 128
typedef struct {
	uint8_t type;
	size_t length;
	char buffer[MAX_PAYLOAD];
} PAYLOAD_t;
