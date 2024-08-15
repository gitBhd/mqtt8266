/*
  Projet d'apprentissage d'un objet connecté (IoT)  pour réaliser une sonde de température
  ESP8266 + DHT22 + LED + MQTT + Home-Assistant
  Projets DIY (http://www.projetsdiy.fr) - Mai 2016
  Article du projet : http://www.projetsdiy.fr/esp8266-dht22-mqtt-projet-objet-connecte/
  Licence : MIT
*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);//clock,data petit OLED caré 0.49" 3v3
//d1 jaune d2 vert
Adafruit_BME280 bme; // use I2C interface
//#include "DHT.h"          // Librairie des capteurs DHT


#define SLEEPTIME 300//secondes

#define wifi_ssid "bhome"
#define wifi_password "AABBCCDDEE"

#define MQTT 
#define mqtt_server "192.168.0.222"
#define mqtt_user "hah"  //s'il a été configuré sur Mosquitto
#define mqtt_password "busimasterH1+" //idem

#define temperature_topic "sensor/blutemperature"  //Topic température
#define humidity_topic "sensor/bluhumidity"        //Topic humidité
#define pressure_topic "sensor/blupressure"        //Topic humidité
#define battery_topic "sensor/blubattery"        //Topic bat
//Buffer qui permet de décoder les messages MQTT reçus
char message_buff[100];

long lastMsg = 0;   //Horodatage du dernier message publié sur MQTT
long lastAna = 0;   
long lastRecu = 0;
bool debug = true;  //Affiche sur la console si True

float batVal=0;
float t=0;
int p=0;
int h =0;


//Création des objets
//DHT dht(DHTPIN, DHTTYPE);     
WiFiClient espClient;
PubSubClient client(espClient);

void toScreen(float t,int p, int h);
void setup_wifi() ;
void reconnect() ;
int bit=0;

IRAM_ATTR void bpPress()
{
  bit=1;
}



void setup() 
{
  Serial.begin(9600);     //Facultatif pour le debug


  u8g2.begin();
  u8g2_SetI2CAddress(u8g2.getU8g2(), 0x3c*2);  
  bme.begin(0x76); 
delay(1000);

 
  pinMode(14,INPUT);
 attachInterrupt(digitalPinToInterrupt(14),bpPress,HIGH);//le bp est tiré par le bas sur res externe
  t= bme.readTemperature();
     p=1.0315*(bme.readPressure()/ 100.0F);
   h=bme.readHumidity();
    toScreen(t,p,h);
    Serial.printf("%2.1f  %4d  %2d\r\n",t,p,h); 
  #ifdef MQTT
    setup_wifi();           //On se connecte au réseau wifi
    
    client.setServer(mqtt_server, 1883);    //Configuration de la connexion au serveur MQTT
   
  #endif
Serial.printf("Chip comeBack...\r\n");
 
}


void toScreen(float t,int p, int h)
{
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.drawHLine(0,0,63);
  u8g2.drawHLine(0,15,63);
  u8g2.drawHLine(0,31,63);
  u8g2.setFont(u8g2_font_fur11_tf);//u8g2_font_inb16_mr );  // choose a suitable font
  u8g2.setCursor(0, 14);
  
  u8g2.printf("T=%2.1fC",t);// print(a);
  u8g2.setFont(u8g2_font_lubR08_tf);
  u8g2.setCursor(0, 29);
  u8g2.printf("%3dmB%2d%%",p,h );// print(a);
  u8g2.sendBuffer();          // transfer internal memory to the display
  delay(1000*2);
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
}
void loop() 

{
  

  
 #ifdef MQTT
  if (!client.connected()) {    reconnect();  }  client.loop();
 #endif
  long now = millis();
  //Envoi d'un message par minute
 // if (now - lastMsg > 1000 * 10) {
  if (now - lastMsg > 1000 * 10) 
  {
    lastMsg = now;
  
 lastAna = now;
    int vIn=analogRead(0);
    float k=0.93;
    batVal=((float)(2*vIn))*3.3;
    batVal=(k*batVal)/1024;

 //   t=1;p=2;h=3;
   #ifdef MQTT
   
  client.publish(humidity_topic, String(h).c_str(), true);      //Et l'humidité
   client.publish(pressure_topic, String(p).c_str(), true);      //Et pressure
   client.publish(battery_topic, String(batVal).c_str(), true);      //Et batterie
   client.publish(temperature_topic, String(t).c_str(), true);   //Publie la température sur le topic temperature_topic
   Serial.printf("send MQTT data..\r\n");
   delay(30000);
   client.disconnect();
   Serial.printf("deepSleep 60s...\r\n");
   //u8g2.clearBuffer();   u8g2.sendBuffer();

  ESP.deepSleep( SLEEPTIME * 1000000 );
 #endif
  }
if (now - lastAna > 100 * 10) 
  {
    

     //Serial.printf("%d   %1.1f\r\n",vIn,r);
  }
 if(bit==1)
  {
    Serial.printf("BP PRESS\r\n");
    bit=0;
      t= bme.readTemperature();
     p=1.0315*(bme.readPressure()/ 100.0F);
   h=bme.readHumidity();
    toScreen(t,p,h);
    
    Serial.printf("%2.1f  %4d  %2d\r\n",t,p,h); 
    delay(1000*5);
  Serial.printf("deepSleep 6s...\r\n");
   u8g2.clearBuffer();
   u8g2.sendBuffer();
   ESP.deepSleep( SLEEPTIME * 1000000 );


  } 
  
}


//Connexion au réseau WiFi
void setup_wifi() 
{
  delay(10);
  Serial.println();
  Serial.print("Connexion a ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connexion WiFi etablie ");
  Serial.print("=> Addresse IP : ");
  Serial.print(WiFi.localIP());
}

//Reconnexion
void reconnect() 
{
  //Boucle jusqu'à obtenur une reconnexion
  #ifdef MQTT
  while (!client.connected()) {
    Serial.print("Connexion au serveur MQTT...");
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("OK");
    } else {
      Serial.print("KO, erreur : ");
      Serial.print(client.state());
      Serial.println(" On attend 5 secondes avant de recommencer");
      delay(5000);
    }
  }
  #endif
}