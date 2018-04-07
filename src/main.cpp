#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h> // Libreria necesaria para el sensor de Humedad y temperatura
#include <DHT_U.h> // Libreria sensor de Humedad y temperatura
#include "DHT.h" // Libreria sensor de Humedad y temperatura
#include <BH1750.h>  // Libreria sensor de Luz
#include <LiquidCrystal_I2C.h> // Libreria LCD
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>

#define BAUD_RATE 115200

const char *mqtt_server = "things.ubidots.com";

const char *dns = "invernadero-01";

const char *str_status[] = {"WL_IDLE_STATUS",    "WL_NO_SSID_AVAIL",
                            "WL_SCAN_COMPLETED", "WL_CONNECTED",
                            "WL_CONNECT_FAILED", "WL_CONNECTION_LOST",
                            "WL_DISCONNECTED"};

// provide text for the WiFi mode
const char *str_mode[] = {"WIFI_OFF", "WIFI_STA", "WIFI_AP", "WIFI_AP_STA"};
WiFiClient espClient;
PubSubClient client(espClient);
MDNSResponder mdns;
bool dnsConnection = false;
unsigned long startTime = millis();
void telnetHandle();

WiFiServer telnetServer(23);
WiFiClient serverClient;

// Senseor de humedad y temperatura DHT11
#define DHTPIN D5 // Se define el pin de lectura de datos
#define DHTTYPE DHT11  // Se define el tipo de sensor DHTs
DHT dht(DHTPIN, DHTTYPE); // Se crea el objeto dht de tipo DHT
int h; // Varible que recopila la humedad
int t; // Variable que recopila la temperatura

//HIGRÓMETRO FC-28
#define PINSUELO A0
int porcHs; // Variable que recopila la humedad del suelo

//Intesidad luminica
BH1750 lightMeter; // Se creo un objeto de tipo BH1750 llamado lightMeter(Medidor de luz)
uint16_t lux;

//Valvula
#define PINVALVULA D3 // asignacion de pin como salida para valvula BLANCO
bool valvula_estado = false;

//Extractor
#define PINEXTRACTOR D4 // asignacion de pin como salida para extractor NEGRO
bool extractor_estado = false;

LiquidCrystal_I2C lcd(0x3f,16,2); // Direccion LCD y Dimencion de LCD

void setup_wifi();
void sensarDatos(); // Función que obtiene los datos sensados por los sensores.
void mostrarDatos(); // Función que muestra los datos en el lcd
void riegoAutomatico(); // Función que activa las valvulas dependiendo de los valores sensados.
void codificarJSON(int h, int t, int porcHs, uint16_t lux);
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);


void setup() {
    Serial.begin(BAUD_RATE);

    //Iniciar LCD
    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0,0);

    setup_wifi();
    if (mdns.begin(dns, WiFi.localIP())) {
      dnsConnection = true;
    }
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    Wire.begin();

    dht.begin(); // Iniciar sensor DHT11
    lightMeter.begin(); // Iniciar sensor de Luz

    //Pin de la valvula y del extractor establecidos como pines de salida
    pinMode(PINVALVULA, OUTPUT); //
    pinMode (PINEXTRACTOR, OUTPUT);

    telnetServer.begin();
    telnetServer.setNoDelay(true);
    Serial.println("Please connect Telnet Client, exit with ^] and 'quit'");

    Serial.print("Free Heap[B]: ");
    Serial.println(ESP.getFreeHeap());
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

  if (!wifiManager.autoConnect("Invernadero AP")) {
    Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  Serial.println("WiFi connected");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("WiFi Conectado");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
}

void loop() {

  telnetHandle();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  sensarDatos();
  mostrarDatos();
  riegoAutomatico();
  delay(1000);

}

bool mensaje = false;
void telnetHandle() {
  if (telnetServer.hasClient()) {
    if (!serverClient || !serverClient.connected()) {
      if (serverClient) {
        serverClient.stop();
        Serial.println("Telnet Client Stop");
      }
      serverClient = telnetServer.available();
      Serial.println("New Telnet client");
      serverClient
          .flush(); // clear input buffer, else you get strange characters
    }
  }

  while (serverClient.available()) { // get data from Client
    Serial.write(serverClient.read());
  }

  if (!mensaje) { // run every 2000 ms
    startTime = millis();

    if (serverClient && serverClient.connected()) { // send data to Client
      serverClient.println("Conectado por telnet.");
      if (WiFi.status() == WL_CONNECTED) {
        serverClient.println("Conectado a: ");
        serverClient.println(WiFi.localIP());
      }
      if (client.connected()) {
        serverClient.println("Conectado a MQTT");
      }
      if (dnsConnection) {
        serverClient.println("Se logró establecer el dns");
      }
      mensaje = true;
    }
  }
  delay(10); // to avoid strange characters left in buffer
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect("invernaderoClient","A1E-ZB0GWTu0Cfe4LjMnDpDfgJTk0NCPp1","")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("datos-invernadero","Conectado al servidor MQTT.");
      // ... and resubscribe
      //client.subscribe("invernadero");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void codificarJSON(int h, int t, int porcHs, uint16_t lux){
  StaticJsonBuffer<200> jsonWrite;
  JsonObject &write = jsonWrite.createObject();

  write["temperatura"] = t;
  write["humedada"] = h;
  write["luminosidad"] = lux;
  write["humedads"] = porcHs;
  write["valvula"] = digitalRead(PINVALVULA);
  write["extractor"] = digitalRead(PINEXTRACTOR);

  String JSON;
  write.printTo(JSON);
  const char *payload = JSON.c_str();

  //Serial.println(payload);
  //Se codifica y se publica el JSON con los datos recogidos
  client.publish("/v1.6/devices/invernadero", payload);
}

void sensarDatos(){

    h=dht.readHumidity(); //Humedad del aire

    t=dht.readTemperature(); // Temperatura

    lux = lightMeter.readLightLevel(); // Intesidad luminica

    //porcHs= map(analogRead(0), 0, 1024, 100, 0); // Humedad del suelo
    //Serial.print(analogRead(0));
    porcHs = (analogRead(0) * 100) / 1024;
    porcHs = 100 - porcHs;

    codificarJSON(h, t, porcHs, lux);
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
  lcd.print("HS:");
  lcd.print(porcHs);
  lcd.print("%");

}

void controlValvula(int hs){
  if(hs<90 && valvula_estado==false && extractor_estado==false){
    valvula_estado = true;
    digitalWrite (PINVALVULA, HIGH ) ; //valvula se enciende si la humedad es menor al 90%
    Serial.println("Valvula activada");
  }else if(hs>=90 && valvula_estado){
    valvula_estado = false;
    digitalWrite (PINVALVULA, LOW) ; //valvula se mantiene apagaga
    Serial.println("Valvula desactivada");
  }
}

void controlExtractor(int t){
  if(t>28 && extractor_estado==false && valvula_estado==false){
    extractor_estado = true;
    digitalWrite (PINEXTRACTOR, HIGH);
    Serial.println("Extractor activado");
  }else if(t<=28&extractor_estado){
    extractor_estado = false;
    digitalWrite (PINEXTRACTOR, LOW);
    Serial.println("Extractor desactivado");
  }
}

void riegoAutomatico(){
  controlExtractor(t);
  controlValvula(porcHs);

}
