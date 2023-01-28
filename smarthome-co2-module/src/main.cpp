/*
  MIT License.
  Install MQTT lib to ESP8266.
  Tested on NodeMCU 1.0 (ESP12E)
*/

#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Passwords as scret, so will not publish them
// Define yours with #define in secrets.h

/*
#define MQTT_SERVER "host.com"
#define MQTT_SERVERPORT 1883
#define MQTT_USERNAME "username"
#define MQTT_PASSWORD "password"

#define WIFI_NETWORK "Network-name"
#define WIFI_PASSWORD "Network-password"
*/

#include "secrets.h"

const char *ssid = WIFI_NETWORK;
const char *password = WIFI_PASSWORD;

#define PIN_D6 12
#define PIN_D7 13
#define PIN_D8 15

#define OUPUT_RX PIN_D7
#define OUPUT_TX PIN_D8
#define OUPUT_LED PIN_D6

// Led signal, define threshold, where on and off

#define SIGNAL_CO2_HIGH 1000
#define SIGNAL_CO2_LOW  800

const byte s8_co2[8] = {0xfe, 0x04, 0x00, 0x03, 0x00, 0x01, 0xd5, 0xc5};
const byte s8_fwver[8] = {0xfe, 0x04, 0x00, 0x1c, 0x00, 0x01, 0xe4, 0x03};
const byte s8_id_hi[8] = {0xfe, 0x04, 0x00, 0x1d, 0x00, 0x01, 0xb5, 0xc3};
const byte s8_id_lo[8] = {0xfe, 0x04, 0x00, 0x1e, 0x00, 0x01, 0x45, 0xc3};
SoftwareSerial swSer(OUPUT_RX, OUPUT_TX, false); // RX, TX

byte buf[10];
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);
Adafruit_MQTT_Publish* senseair;

void MQTT_connect();
uint16_t readco2();

bool led_state = false;
char mqtt_topic[32];

void myread(int n)
{
    int i;
    memset(buf, 0, sizeof(buf));
    for (i = 0; i < n;)
    {
        if (swSer.available() > 0)
        {
            buf[i] = swSer.read();
            i++;
        }
        yield();
        delay(10);
    }
}

// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(byte *buf, int len)
{
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--)
        { // Loop over each bit
            if ((crc & 0x0001) != 0)
            {              // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else           // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

void setup()
{
    // Preapre status LED
    pinMode(OUPUT_LED, OUTPUT);
    digitalWrite(OUPUT_LED, LOW);

    // Prepare MQTT topic
    snprintf(mqtt_topic, 32, "petrows/%s/co2", WiFi.macAddress().c_str());

    Serial.begin(115200);
    swSer.begin(9600);
    delay(10);

    senseair = new Adafruit_MQTT_Publish(&mqtt, mqtt_topic);

    Serial.print("Connecting to ");
    Serial.println(WIFI_NETWORK);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

    int wifi_retry = 0;

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        digitalWrite(OUPUT_LED, led_state ? HIGH : LOW);

        led_state = !led_state;
        wifi_retry++;

        if (wifi_retry > 128) {
            Serial.println("WiFi not connected, restarting");
            ESP.restart();
        }
    }

    digitalWrite(OUPUT_LED, LOW);
    led_state = false;

    Serial.println();
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.print("Sensor ID: ");
    swSer.write(s8_id_hi, 8);
    myread(7);
    Serial.printf("%02x%02x", buf[3], buf[4]);
    swSer.write(s8_id_lo, 8);
    myread(7);
    Serial.printf("%02x%02x", buf[3], buf[4]);
    Serial.println("");

    swSer.write(s8_fwver, 8);
    myread(7);
    Serial.printf("Firmware: %d.%d", buf[3], buf[4]);
    Serial.println();
}

void loop()
{
    uint16_t co2;

    MQTT_connect();
    co2 = readco2();
    senseair->publish(co2);

    // Led control
    // We
    if (!led_state && co2 > 1000)
    {
        digitalWrite(OUPUT_LED, HIGH);
        led_state = true;
    }

    if (led_state && co2 < 800)
    {
        digitalWrite(OUPUT_LED, LOW);
        led_state = false;
    }

    delay(60 * 1000L);
}

uint16_t readco2()
{
    uint16_t crc, got, co2;

    swSer.write(s8_co2, 8);
    myread(7);
    co2 = (uint16_t)buf[3] * 256 + (uint16_t)buf[4];
    crc = ModRTU_CRC(buf, 5);
    got = (uint16_t)buf[5] + (uint16_t)buf[6] * 256;
    if (crc != got)
    {
        Serial.print("Invalid checksum.  Expect: ");
        Serial.print(crc, HEX);
        Serial.print("  Got: ");
        Serial.println(got, HEX);
    }
    else
    {
        Serial.print("CO2: ");
        Serial.println(co2);
        // Serial.printf("%02x %02x %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
    }
    return co2;
}

void MQTT_connect()
{
    int8_t ret;

    if (mqtt.connected())
        return;

    Serial.print("Connecting to MQTT... ");

    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0)
    {
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(5000); // wait 5 seconds
        retries--;
        if (retries == 0)
        {
            Serial.println("Cannot connect to MQTT.  System hangs.");
            ESP.restart();
        }
    }
    Serial.println("MQTT Connected!");
}
