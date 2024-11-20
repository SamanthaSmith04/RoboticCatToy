#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h> // Include the Adafruit NeoPixel library

// Replace the next variables with your SSID/Password combination
const char* ssid = "SSID";
const char* password = "PASSWORD";

// Add your MQTT Broker address, example:
const char* mqtt_server = "broker.hivemq.com";
const char* unique_identifier = "sunfounder-client-sdgvsda";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
int value = 0;


// LED STRIP PIN
const int LED_PIN = 4;

const int NUM_LEDS = 8;

int led_strip_lights[NUM_LEDS];

const char* X_JOYSTICK = "X_JOYSTICK";

const char* Y_JOYSTICK = "Y_JOYSTICK";

int x_pos = 0;

int y_pos = 0;

Adafruit_NeoPixel light_strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800); 


// When you connect to WIFI, only 36 39 34 35 32 33 pins can be used for analog reading.



void setup() {
  Serial.begin(115200);

  light_strip.begin();

  // default settings
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(LED_PIN, OUTPUT);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic "SF/LED", you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == X_JOYSTICK) {
    Serial.print("Changing x pos to ");
    x_pos = (int)message;
    Serial.print((char)x_pos);
  }
  else if (String(topic) == Y_JOYSTICK) {
    Serial.print("Changing y pos to ");
    y_pos = (int)message;
    Serial.print((char)y_pos);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(unique_identifier)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(X_JOYSTICK);
      client.subscribe(Y_JOYSTICK);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void update_led_strip() {
  if (x_pos > 0 && y_pos > 0) {
    for (int i = 0; i < NUM_LEDS; i++){
      
    }
  }
}