// Archivo utilizado por el main para configurar la esp32 principal
#pragma once

#define UV_WIFI_SSID "internet"
#define UV_WIFI_PASSWORD "contraseña"

#define UV_MQTT_URI "mqtt://172.20.10.4:1883"

#define UV_TOPIC_TELEMETRY "uvgrow/nodo01/telemetria"
#define UV_TOPIC_EVENTS "uvgrow/nodo01/eventos"
#define UV_TOPIC_COMMANDS "uvgrow/nodo01/comandos"

#define UV_RELAY_PUMP_GPIO 25
#define UV_RELAY_FAN_GPIO 26

#define UV_PIR_GPIO 27

#define UV_ADC_LM35 4
#define UV_ADC_SOIL 5
#define UV_ADC_WATER 6
#define UV_ADC_MQ2 7
#define UV_ADC_LDR 0

#define UV_ENABLE_DHT11 1
#define UV_ENABLE_LM35 1
#define UV_ENABLE_SOIL 1
#define UV_ENABLE_WATER 1
#define UV_ENABLE_MQ2 1
#define UV_ENABLE_LDR 1
#define UV_ENABLE_PIR 1
#define UV_ENABLE_RFID 1
#define UV_ENABLE_REED 1
#define UV_REED_GPIO 16
#define UV_REED_INTERVAL_MS 100

#define UV_TELEMETRY_MS 60000
#define UV_SENSOR_MS 5000

#define UV_PUMP_MAX_MS 2000
#define UV_PUMP_COOLDOWN_MS 300000
