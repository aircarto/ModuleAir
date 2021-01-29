/*
  Code pour faire un GET sur le site de Aircarto
  Board: LOLIN(WEMOS)D1 R2 & mini
  Paul Vuarambon
  Aircarto
  Janvier 2021
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

/* Set these to your desired credentials. */
const char* ssid = "ModuleAir";
const char* password = "ModuleAir";

String data;
 

void setup() {
  delay(1000);
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);        //Prevents reconnection issue (taking too long to connect)
  delay(1000);
  WiFi.mode(WIFI_STA);        //This line hides the viewing of ESP as wifi hotspot

  
  
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");

  Serial.print("Connecting");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
}



void loop() {

  if (Serial.available()) {
    
  data = Serial.readString();
  data.trim();
  Serial.print("Donnée recues: ");
  Serial.println(data);

  delay(1000);


  HTTPClient http;    //Declare object of class HTTPClient
  
  String Link;
  
  Link = "http://data.aircarto-asso.fr/data-upload.php?data=" + data;
  
    
  Serial.print("Try to access: ");
  Serial.print(Link);
  delay(1000);
  
  http.begin(Link);     //Specify request destination
  
  int httpCode = http.GET();            //Send the request
  String payload = http.getString();    //Get the response payload

  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload

  http.end();  //Close connection
  
  delay(5000);  //GET Data at every 5 seconds
}

  
}
