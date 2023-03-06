# esp-idf-virtual-serial-link
Virtual serial communication using esp-idf.   

Long ago, most PCs had D-SUB serial ports.   
When PCs had D-SUB serial ports, serial communication between two PCs was very easy.   
Communication was possible with only one RS232C cross cable.   
```
+------------+                                              +------------+
| PC1        |              RS232C Cross-Cable              | PC2        |
|            |<-------------------------------------------->|            |
|            |                                              |            |
+------------+                                              +------------+
```

Before I knew it, the D-SUB serial port had disappeared from my PC, and it had changed a USB port.   
Using the USB port, serial communication between two PCs looks like this.   
```
+------------+     +------------+        +------------+     +------------+
| PC1        | USB | USB-UART   |  UART  | USB-UART   | USB | PC2        |
|            |     | Adapter    |        | Adapter    |     |            |
|            |     |            |        |            |     |            |
|            |<--->|        RX  |<-------| TX         |<--->|            |
|            |     |        TX  |------->| RX         |     |            |
+------------+     +------------+        +------------+     +------------+ 
```

ESP32S2/S3 has TinyUSB Serial Device function.   
It can communicate with the host via USB.   
By using this, serial communication can be made wireless.   
```
+------------+     +------------+        +------------+     +------------+
| PC1        | USB | ESP32S2/S3 |  WiFi  | ESP32S2/S3 | USB | PC2        |
|            |     |            |        |            |     |            |
|            |<--->|            |<------>|            |<--->|            |
|            |     |            |        |            |     |            |
+------------+     +------------+        +------------+     +------------+ 
```

ESP-NOW eliminates the need for a router.   
```
+------------+     +------------+        +------------+     +------------+
| PC1        | USB | ESP32S2/S3 | ESPNOW | ESP32S2/S3 | USB | PC2        |
|            |     |            |        |            |     |            |
|            |<--->|            |<------>|            |<--->|            |
|            |     |            |        |            |     |            |
+------------+     +------------+        +------------+     +------------+ 
```



# Limitation
The number of characters that can be read at once with tinyusb_cdcacm_read() is limited to 64 bytes.
Changing CONFIG_TINYUSB_CDC_RX_BUFSIZE does not work around this limitation.
If anyone knows a workaround, please let me know.
```
I (30304) tinyusb_cdc_rx_callback: CONFIG_TINYUSB_CDC_RX_BUFSIZE=256 rx_size=64
I (30304) tinyusb_cdc_rx_callback: Data from channel=0 rx_size=64
I (30304) tinyusb_cdc_rx_callback: 0x3fca6944   24 47 50 52 4d 43 2c 30  31 34 34 30 34 2c 41 2c  |$GPRMC,014404,A,|
I (30314) tinyusb_cdc_rx_callback: 0x3fca6954   33 35 34 30 2e 38 38 39  35 34 36 2c 4e 2c 31 33  |3540.889546,N,13|
I (30324) tinyusb_cdc_rx_callback: 0x3fca6964   39 34 36 2e 31 38 34 35  34 30 2c 45 2c 31 36 2e  |946.184540,E,16.|
I (30344) tinyusb_cdc_rx_callback: 0x3fca6974   33 2c 32 35 2e 33 2c 30  35 30 33 32 33 2c 2c 2c  |3,25.3,050323,,,|
I (30354) tinyusb_cdc_rx_callback: CONFIG_TINYUSB_CDC_RX_BUFSIZE=256 rx_size=6
I (30374) tinyusb_cdc_rx_callback: 0x3fca6944   41 2a 37 42 0d 0a                                 |A*7B..|
```


Data exceeding 64 bytes is also split at the receiving end.
```
I (291393) MAIN: Wireless-->USB [$GPRMC,021613,A,3540.909988,N,13946.196413,E,16.3,25.3,050323,,,]
I (291393) MAIN: Wireless-->USB [A*7A
]
```

# Software requirements
ESP-IDF V4.4/V5.0.   
ESP-IDF V5 is required when using ESP32-C2.   

# Hardware requirements
1. ESP32-S2/S3 Development board   
Because the ESP32-S2/S3 does support USB OTG.   

2. USB Connector   
I used this:   
![usb-connector](https://user-images.githubusercontent.com/6020549/124848149-3714ba00-dfd7-11eb-8344-8b120790c5c5.JPG)

```
ESP32-S2/S3 BOARD          USB CONNECTOR
                           +--+
                           | || VCC
    [GPIO 19]    --------> | || D-
    [GPIO 20]    --------> | || D+
    [  GND  ]    --------> | || GND
                           +--+
```

# Installation

```Shell
git clone https://github.com/nopnop2002/esp-idf-virtual-serial-port
cd esp-idf-virtual-serial-port
idf.py set-target {esp32s2/esp32s3}
idf.py menuconfig
idf.py flash
```


# Configuration
![config-top](https://user-images.githubusercontent.com/6020549/223055571-090c3f71-5c81-431f-ab56-da831d41f6bd.jpg)


### Configuration for ESPNOW Unicast
The two ESPs must use the same port.   
In order to perform UDP unicast communication, the IP address of the other party is required.   
This app uses mDNS to look up peer IP addresses with the same port number.   

![config-udp](https://user-images.githubusercontent.com/6020549/223055578-6942f911-0a10-473c-b35d-c4115c5fe233.jpg)




When providing two or more sets of communication circuits, it is necessary to change the port numbers.   

```
+------------+     +------------+        +------------+     +------------+
| PC1        | USB | ESP32S2/S3 |  WiFi  | ESP32S2/S3 | USB | PC2        |
|            |     |            | PORT#1 |            |     |            |
|            |<--->|            |<------>|            |<--->|            |
|            |     |            |        |            |     |            |
+------------+     +------------+        +------------+     +------------+ 

+------------+     +------------+        +------------+     +------------+
| PC3        | USB | ESP32S2/S3 |  WiFi  | ESP32S2/S3 | USB | PC4        |
|            |     |            | PORT#2 |            |     |            |
|            |<--->|            |<------>|            |<--->|            |
|            |     |            |        |            |     |            |
+------------+     +------------+        +------------+     +------------+ 
```

### Configuration for UDP Unicast
The two ESPs must use the same channel.   
In order to perform ESPNOW unicast communication, the MAC address of the other party is required.   
This app uses PING communication to ask for peer MAC addresses with the same channel number.   

![config-espnow](https://user-images.githubusercontent.com/6020549/223055585-5e8b91dc-e356-41ba-8d5a-6a35d8c25972.jpg)



When providing two or more sets of communication circuits, it is necessary to change the channel numbers.   
The range of the channel of paired devices is from 1 to 14.   
```
+------------+     +------------+        +------------+     +------------+
| PC1        | USB | ESP32S2/S3 | ESNOW  | ESP32S2/S3 | USB | PC2        |
|            |     |            |  CH#1  |            |     |            |
|            |<--->|            |<------>|            |<--->|            |
|            |     |            |        |            |     |            |
+------------+     +------------+        +------------+     +------------+ 

+------------+     +------------+        +------------+     +------------+
| PC3        | USB | ESP32S2/S3 | ESPNOW | ESP32S2/S3 | USB | PC4        |
|            |     |            |  CH#3  |            |     |            |
|            |<--->|            |<------>|            |<--->|            |
|            |     |            |        |            |     |            |
+------------+     +------------+        +------------+     +------------+ 
```

