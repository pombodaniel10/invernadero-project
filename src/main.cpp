#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h> // Libreria necesaria para el sensor de Humedad y temperatura
#include <DHT_U.h> // Libreria sensor de Humedad y temperatura
#include <BH1750.h>  // Libreria sensor de Luz
#include <LiquidCrystal_I2C.h> // Libreria LCD
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

/* #include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C */

#define BAUD_RATE 115200
#define TEMP_MIN 31
#define HUM_SUELO 70
#define TOPIC_AUTO "/v1.6/devices/invernadero/auto/lv"
#define TOPIC_INVERNADERO "/v1.6/devices/invernadero"
#define TOPIC_EXTRACTOR "/v1.6/devices/invernadero/extractor/lv"
#define TOPIC_VALVULA "/v1.6/devices/invernadero/valvula/lv"

const char *mqtt_server = "things.ubidots.com";

const char *str_status[] = {"WL_IDLE_STATUS",    "WL_NO_SSID_AVAIL",
                            "WL_SCAN_COMPLETED", "WL_CONNECTED",
                            "WL_CONNECT_FAILED", "WL_CONNECTION_LOST",
                            "WL_DISCONNECTED"};

// provide text for the WiFi mode
const char *str_mode[] = {"WIFI_OFF", "WIFI_STA", "WIFI_AP", "WIFI_AP_STA"};
bool disconnected = 0;
WiFiClient espClient;
PubSubClient client(espClient);


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

bool automatico = true;

LiquidCrystal_I2C lcd(0x3f,16,2); // Direccion LCD y Dimencion de LCD

void setup_wifi();
void sensarDatos(); // Función que obtiene los datos sensados por los sensores.
void mostrarDatos(); // Función que muestra los datos en el lcd
void riegoAutomatico(); // Función que activa las valvulas dependiendo de los valores sensados.
void controlValvula(int hs);
void controlExtractor(int t);
void codificarJSON(int h, int t, int porcHs, uint16_t lux);
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);

float conteo;
bool first;

void setup() {
    Serial.begin(BAUD_RATE);

    //Iniciar LCD
    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0,0);

    Wire.begin();

    dht.begin(); // Iniciar sensor DHT11
    lightMeter.begin(); // Iniciar sensor de Luz

    //Pin de la valvula y del extractor establecidos como pines de salida
    pinMode(PINVALVULA, OUTPUT); //
    pinMode (PINEXTRACTOR, OUTPUT);

    /*bool status;

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }*/

    setup_wifi();

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

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

void loop() {

  if(WiFi.status()==WL_CONNECTED){
    disconnected = 0;
  }else{
    disconnected = 1;
  }

  if (!client.connected()&&disconnected==0) {
    reconnect();
  }

  if(disconnected==0){
      client.loop();
  }


  sensarDatos();
  mostrarDatos();
  if(automatico){
      riegoAutomatico();
  }
  delay(10000);

  if(first==true){
    conteo+=10;
  }
}

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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect("invernaderoClient","A1E-ZScJnTfGddjYY1sw2B64ppaBzORYss","")) {
      Serial.println("connected");

      StaticJsonBuffer<200> jsonWrite;
      JsonObject &write = jsonWrite.createObject();
      write["auto"] = int(automatico);
      String JSON;
      write.printTo(JSON);
      const char *payload = JSON.c_str();
      client.publish("/v1.6/devices/invernadero/auto", payload);
      // Once connected, publish an announcement...
      //client.publish("datos-invernadero","Conectado al servidor MQTT.");
      // ... and resubscribe
      client.subscribe(TOPIC_EXTRACTOR);
      client.subscribe(TOPIC_VALVULA);
      client.subscribe(TOPIC_AUTO);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sensarDatos(){
    if(dht.readHumidity()<=100){
      h=dht.readHumidity(); //Humedad del aire

      t=dht.readTemperature(); // Temperatura
    }

    lux = lightMeter.readLightLevel(); // Intesidad luminica

    porcHs = (analogRead(0) * 100) / 1024;
    porcHs = 100 - porcHs;

    if(disconnected==0){
        codificarJSON(h, t, porcHs, lux);
    }

 }

 void codificarJSON(int h, int t, int porcHs, uint16_t lux){
   StaticJsonBuffer<200> jsonWrite;
   JsonObject &write = jsonWrite.createObject();

   write["temperatura"] = t;
   write["humedada"] = h;
   write["luminosidad"] = lux;
   write["humedads"] = porcHs;
   if(automatico==true || automatico==false&&extractor_estado==false&&valvula_estado==false){
     write["valvula"] = digitalRead(PINVALVULA);
     write["extractor"] = digitalRead(PINEXTRACTOR);
   }else if(automatico==false&&extractor_estado==true){
     write["valvula"] = digitalRead(PINVALVULA);
   }else if(automatico==false&&valvula_estado==true){
     write["extractor"] = digitalRead(PINEXTRACTOR);
   }


   String JSON;
   write.printTo(JSON);
   const char *payload = JSON.c_str();

   //Serial.println(payload);
   //Se codifica y se publica el JSON con los datos recogidos
   client.publish(TOPIC_INVERNADERO, payload);
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

  /*
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %"); */



}

void riegoAutomatico(){
  controlExtractor(t);
  controlValvula(porcHs);
}

void controlValvula(int hs){
  if(hs<HUM_SUELO && valvula_estado==false && extractor_estado==false && (conteo>=60 || first==false) ){
    valvula_estado = true;
    digitalWrite (PINVALVULA, HIGH ) ; //valvula se enciende si la humedad es menor al 70%
    Serial.println("Valvula activada");
    delay(3000);
    valvula_estado = false;
    digitalWrite (PINVALVULA, LOW) ; //valvula se mantiene apagaga
    Serial.println("Valvula desactivada");
    conteo = 0;
    if(first==false){
      first = true;
    }
  }
}

void controlExtractor(int t){
  if(t>=TEMP_MIN && extractor_estado==false && valvula_estado==false){
    extractor_estado = true;
    digitalWrite (PINEXTRACTOR, HIGH);
    Serial.println("Extractor activado");
  }else if(t<=30&extractor_estado==true){
    extractor_estado = false;
    digitalWrite (PINEXTRACTOR, LOW);
    Serial.println("Extractor desactivado");
  }
}
