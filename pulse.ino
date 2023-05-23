#include <LiquidCrystal_I2C.h>
#include "MAX30100_PulseOximeter.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ESP8266WiFi.h>

LiquidCrystal_I2C lcd (0x27, 16, 2);

#define REPORTING_PERIOD_MS 500
#define PUB_PERIOD_MS 2000

#define WLAN_SSID "IbelieveWIcanFI"
#define WLAN_PASS "ylle5871"

#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME  "sid25ta"
#define AIO_KEY       "aio_OAfb88QfCeXeCKzsnVE7yObVY0GE"

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish HeartRate = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/HeartRate");
Adafruit_MQTT_Publish SpO2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/spo2");
Adafruit_MQTT_Publish Warning_SpO2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Warning_SpO2");
Adafruit_MQTT_Publish Warning_heartrate = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Warning_HeartRate");

PulseOximeter pox;
uint32_t tsLastReport = 0;
uint32_t pubLastReport = 0;

void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if (ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void onBeatDetected()
{
  Serial.println("Beat!");
}

void setup()
{
  Serial.begin(115200);

  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

  connect();

  Serial.print("Initializing pulse oximeter..");
  lcd.init ();
  lcd. backlight ();
  lcd.print("Initializing...");
  delay(3000);
  lcd.clear();

  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_27_1MA);

  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop()
{ 
  char low_hr[] = "Warning Low heart rate";
  char high_hr[] = "Warning High heart rate";
  char low_spo2[] = "Warning Low SpO2,visit doctor immediately";
  char crictical_spo2[] = "Warning critically low SpO2";
  
  pox.update();
  float heartrate = pox.getHeartRate();
  float spo2 = pox.getSpO2();

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    Serial.print("Heart rate:");
    Serial.print(pox.getHeartRate());

    Serial.print("bpm / SpO2:");
    Serial.print(pox.getSpO2());
    Serial.println("%");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BPM : ");
    lcd.print(pox.getHeartRate());

    lcd.setCursor(0, 1);
    lcd.print("SpO2: ");
    lcd.print(pox.getSpO2());
    lcd.print("%");

    tsLastReport = millis();

    if (heartrate >120) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Elevated");
      lcd.setCursor(0, 1);
      lcd.print("Heart rate");
    }

    else if (heartrate >140) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Warning High");
      lcd.setCursor(0, 1);
      lcd.print("Heart rate");
    }

    if (spo2 < 90) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Warning Low");
      lcd.setCursor(0, 1);
      lcd.print("SpO2%");
    }

    else if (spo2 < 70) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Visit Doctor");
      lcd.setCursor(0, 1);
      lcd.print("Immediately");
      if(!Warning_SpO2.publish(low_spo2)) {
      Serial.println(F("Failed"));
      } 
    }

    else if (spo2 < 60) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Critically Low");
      lcd.setCursor(0, 1);
      lcd.print("SpO2%");
      if(!Warning_SpO2.publish(crictical_spo2)) {
      Serial.println(F("Failed"));
      }
    }
  }

  if (millis()-pubLastReport>PUB_PERIOD_MS) {
    if (!HeartRate.publish(heartrate)) {
      Serial.println(F("Failed"));
    }
    if (!SpO2.publish(spo2)) {
      Serial.println(F("Failed"));
    }
    else {
      Serial.println(F("Sent!"));
    }
    pubLastReport = millis();
  }
}
