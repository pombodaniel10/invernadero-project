#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>  // Libreria sensor de Luz
#include <LiquidCrystal_I2C.h> // Libreria LCD
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

#define GREEN_LED D6
#define RED_LED D7
#define BLUE_LED D8

#define BAUD_RATE 115200
#define TEMP_MIN 15
#define HUM_SUELO 80
#define LUM_MIN 500

/*
#define PCF8591 (0x90 >> 1) // Dirreccion del bus I2C 1001 0000
#define PCF8591_ACTIVAR_DAC 0x40  //0100 0000
#define PCF8591_ADC0 0x00
#define PCF8591_ADC1 0x01
#define PCF8591_ADC2 0x02
#define PCF8591_ADC3 0x03
*/

const char *mqtt_server = "mqtt.thingspeak.com";

char mqttUserName[] = "TSArduinoMQTTDemo";  // Can be any name.
char mqttPass[] = "3LFV5BSW05P9VU3G";  // Change this your MQTT API Key from Account > MyProfile.
char writeAPIKey[] = "SV7Y0CX6PHT3CR9Y";    // Change to your channel Write API Key.
long channelID = 541211;

const char *str_status[] = {"WL_IDLE_STATUS",    "WL_NO_SSID_AVAIL",
                            "WL_SCAN_COMPLETED", "WL_CONNECTED",
                            "WL_CONNECT_FAILED", "WL_CONNECTION_LOST",
                            "WL_DISCONNECTED"};

// provide text for the WiFi mode
const char *str_mode[] = {"WIFI_OFF", "WIFI_STA", "WIFI_AP", "WIFI_AP_STA"};
bool disconnected = 0;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastConnectionTime = 0;
const unsigned long postingInterval = 10L * 500L; // Post data every 5 seconds.


float h; // Varible que recopila la humedad
float t; // Variable que recopila la temperatura

//HIGRÓMETRO FC-28
#define PINSUELO A0
int porcHs; // Variable que recopila la humedad del suelo
int porcHs2;
int porcHs3;

//Intesidad luminica
BH1750 lightMeter; // Se creo un objeto de tipo BH1750 llamado lightMeter(Medidor de luz)
uint16_t lux;

float presion;

float altitud;

//Valvula
#define PINVALVULA D3 // asignacion de pin como salida para valvula BLANCO
bool valvula_estado = false;

//Extractor
#define PINEXTRACTOR D4 // asignacion de pin como salida para extractor NEGRO
bool extractor_estado = false;

bool automatico = true;

LiquidCrystal_I2C lcd(0x3f,16,2); // Direccion LCD y Dimencion de LCD

void setup_wifi();
void sensarDatos(); // Función que obtiene los datos sensados por los sensores.
void mostrarDatos(); // Función que muestra los datos en el lcd
void controlValvula(int hs);
void controlExtractor(int t);
void codificarJSON(int h, int t, int porcHs, uint16_t lux,float presion, float altitud);
void reconnect();
//void callback(char *topic, byte *payload, unsigned int length);
void controlIL(uint16_t lux);
int leerPCF8591_Pin(unsigned int pin);

void setup() {
    Serial.begin(BAUD_RATE);

    //Iniciar LCD
    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0,0);

    Wire.begin();

    lightMeter.begin(); // Iniciar sensor de Luz

    //Pin de la valvula y del extractor establecidos como pines de salida
    pinMode(PINVALVULA, OUTPUT); //
    pinMode (PINEXTRACTOR, OUTPUT);

    //Pines de la tira RGB
    pinMode(RED_LED,OUTPUT);
    pinMode(BLUE_LED,OUTPUT);
    pinMode(GREEN_LED,OUTPUT);

    analogWriteFreq(500);
    analogWriteRange(100);

    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    setup_wifi();

    client.setServer(mqtt_server, 1883);
    //client.setCallback(callback);

}

void loop() {
  // Reconnect if MQTT client is not connected.
  if (!client.connected()){
    reconnect();
  }

  client.loop();   // Call the loop continuously to establish connection to the server.

  // If interval time has passed since the last connection, Publish data to ThingSpeak
  if (millis() - lastConnectionTime > postingInterval){
    sensarDatos();
    mostrarDatos();
    controlExtractor(t);
    controlValvula(porcHs);
    controlIL(lux);
  }
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.print("Conectandose a la red WiFi...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectandose");
  lcd.setCursor(0,1);
  lcd.print("a la red WiFi...");

  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  if (!wifiManager.autoConnect("Invernadero AP")) {
    Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    //ESP.reset();
    //delay(1000);
    disconnected = 1;
  }else{
    disconnected = 0;
  }

  Serial.println("WiFi connected");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("WiFi Conectado");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


}

/*
void callback(char *topic, byte *payload, unsigned int length) {

  String top = topic;

  if(top==TOPIC_AUTO){
    if(automatico==false&&(char)payload[0]=='1'){
      Serial.print("Modo automatico activado");
      automatico =  true;

      StaticJsonBuffer<200> jsonWrite;
      JsonObject &write = jsonWrite.createObject();
      write["auto"] = int(automatico);
      String JSON;
      write.printTo(JSON);
      const char *payload = JSON.c_str();
      client.publish("/v1.6/devices/invernadero/auto", payload);

    }else if(automatico==true&&(char)payload[0]=='0'){
      Serial.print("Modo automatico desactivado");
      automatico =  false;

      StaticJsonBuffer<200> jsonWrite;
      JsonObject &write = jsonWrite.createObject();
      write["auto"] = int(automatico);
      String JSON;
      write.printTo(JSON);
      const char *payload = JSON.c_str();
      client.publish("/v1.6/devices/invernadero/auto", payload);
    }

  }else if(top==TOPIC_EXTRACTOR&&automatico==false){

    if((char)payload[0]=='1' && extractor_estado==false && valvula_estado==false){
      digitalWrite (PINEXTRACTOR, HIGH ) ;
      extractor_estado = true;
      Serial.println("Extractor activado");
    }else if((char)payload[0]=='0' && extractor_estado==true){
      extractor_estado = false;
      digitalWrite (PINEXTRACTOR, LOW ) ;
      Serial.println("Extractor desactivado");
    }

  }else if(top==TOPIC_VALVULA&&automatico==false){

    if((char)payload[0]=='1' && valvula_estado==false && extractor_estado==false){
      digitalWrite (PINVALVULA, HIGH ) ;
      valvula_estado = true;
      Serial.println("Valvula activada");
      delay(3000);
      valvula_estado = false;
      digitalWrite (PINVALVULA, LOW ) ;
      Serial.println("Valvula desactivada");
    }else if((char)payload[0]=='0' && valvula_estado==true){
      valvula_estado = false;
      digitalWrite (PINVALVULA, LOW ) ;
      Serial.println("Valvula desactivada");
    }

  }

}
*/

void reconnect() {
  char clientID[10];

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");

    // Connect to the MQTT broker
    if (client.connect("invernaderoInedsor",mqttUserName,mqttPass))
    {
      Serial.println("connected");
    } else
    {
      Serial.print("failed, rc=");
      // Print to know why the connection failed.
      // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/*
int leerPCF8591_Pin(unsigned int pin){
    Wire.beginTransmission(PCF8591);
    Wire.write(pin);
    Wire.endTransmission();
    Wire.requestFrom(PCF8591, 2);
    Wire.read();  // Se omite el primero porque es el valor anterior
    int valor = (int) Wire.read();
    valor = (valor*100)/1023;
    valor = 100-valor;
    return valor;
}*/

void sensarDatos(){

    lux = lightMeter.readLightLevel(); // Intesidad luminica

    porcHs = analogRead(A0);
    porcHs = (porcHs*100)/1023;
    porcHs = 100-porcHs;
    //porcHs2 = leerPCF8591_Pin(PCF8591_ADC1);
    //porcHs3 = leerPCF8591_Pin(PCF8591_ADC2);

    presion = bme.readPressure() / 100.0F;
    altitud = bme.readAltitude(SEALEVELPRESSURE_HPA);
    t = bme.readTemperature();
    h = bme.readHumidity();

    if(disconnected==0){
        codificarJSON(h, t, porcHs,lux, presion, altitud);
    }

 }

 void codificarJSON(int h, int t, int porcHs, uint16_t lux, float presion, float altitud){

   // Create data string to send to ThingSpeak
  String data = String(
                  "field1=" + String(t, DEC)
                + "&field2=" + String(h, DEC)
                + "&field3=" + String(porcHs, DEC)
                + "&field4=" + String(lux, DEC)
                + "&field5=" + String(presion, DEC)
              );

  int length = data.length();
  char msgBuffer[length];
  data.toCharArray(msgBuffer,length+1);
  Serial.println(msgBuffer);

  // Create a topic string and publish data to ThingSpeak channel feed.
  String topicString ="channels/" + String( channelID ) + "/publish/"+String(writeAPIKey);
  length=topicString.length();
  char topicBuffer[length];
  topicString.toCharArray(topicBuffer,length+1);

  client.publish( topicBuffer, msgBuffer );

  lastConnectionTime = millis();

 }

void mostrarDatos(){
  lcd.clear();
  lcd.setCursor(0,0);

  Serial.println("Datos:");

  Serial.print("H: ");
  Serial.print(h);
  Serial.print(" %\t");
  lcd.print("HA:");
  lcd.print(h);
  lcd.print("% ");

  Serial.print("T: ");
  Serial.print(t);
  Serial.print(" °C\t");
  lcd.print("T:");
  lcd.print(t);
  lcd.print("C");

  Serial.print("Intesidad luminica: ");
  Serial.print(lux);
  Serial.print(" lx\t");
  lcd.setCursor(0,1);
  lcd.print("IL:");
  lcd.print(lux);
  lcd.print("Lx");

  Serial.print("HS: ");
  Serial.print(porcHs);
  Serial.print("\n\n");

  /*Serial.print("HS 2: ");
  Serial.print(porcHs2);
  Serial.print("\n\n");

  Serial.print("HS 3: ");
  Serial.print(porcHs3);
  Serial.print("\n\n"); */
  lcd.print("HS:");
  lcd.print(porcHs);
  lcd.print("%");

  Serial.print("Pressure = ");
  Serial.print(presion);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitud);
  Serial.println(" m");

}

void controlValvula(int hs){
  if(hs<HUM_SUELO && valvula_estado==false && extractor_estado==false){
    valvula_estado = true;
    digitalWrite (PINVALVULA, HIGH ) ; //valvula se enciende si la humedad es menor al 70%
    Serial.println("Valvula activada");
    delay(3000);
    valvula_estado = false;
    digitalWrite (PINVALVULA, LOW) ; //valvula se mantiene apagaga
    Serial.println("Valvula desactivada");
  }
}

void controlExtractor(int t){
  if(t>=TEMP_MIN && extractor_estado==false && valvula_estado==false){
    extractor_estado = true;
    digitalWrite (PINEXTRACTOR, HIGH);
    Serial.println("Extractor activado");
  }else if(t<=TEMP_MIN&extractor_estado==true){
    extractor_estado = false;
    digitalWrite (PINEXTRACTOR, LOW);
    Serial.println("Extractor desactivado");
  }
}

void controlIL(uint16_t lux){
    if(lux<LUM_MIN){
      analogWrite(RED_LED, 62);
      analogWrite(GREEN_LED,6);
      analogWrite(BLUE_LED, 148);
    }else{
      analogWrite(RED_LED, 0);
      analogWrite(BLUE_LED, 0);
      analogWrite(GREEN_LED,0);
    }
  }
