/*
 *
 * Code Module Air
 * avec envoi des données sur le port de Série n°2 pour module WIFI
 *
 * Serial 3: SDS011
 * Serial 2: WIFI (esp8266 D1 mini)
 *
 * Sonde CO2 (MHZ16 ou MH-Z19b)
 * EN MODE UART sur Serial1
 *
 *
 * !! Attention ne pas oublier de modifier le nom du capteur !!!
 *
 * !! Attention: suivant le type d'écran Matrix inverser OE et LAT
 *
 * Librairies à intaller:
 *
    Adafruit GFX library (dispo dans le gestionnaire de librairies)
    RGB Matrix Panel (dispo dans le gestionnaire de librairies)
    SDS011 sensor library (dispo dans le gestionnaire de librairies)

    Pour la sonde C02 deux versions possible: MH-Z16 ou MH-19B:
    NDIRZ16 (dispo ici: https://github.com/airpaca/Module_Air/blob/master/20181218_AtmoSud_Module_Air/libraries/NDIRZ16-master.zip)
    ou
    MHZ9 (dispo ici: https://github.com/WifWaf/MH-Z19)

 *
 *
 *
 * MAJ du 3 sept: pour ne plus utiliser la fonction String, on envoie directement les données au format <devicenNum, CO2, PM10, PM2.5>
 * MAJ du 25 sept: Sonde CO2 en mode UART
 */

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <RGBmatrixPanel.h>
#include "logos.h"

#include <Wire.h>
#include <SPI.h>

#include <NDIRZ16.h> // CO2
#include <MHZ19.h>

#include <SDS011.h> // PM

#define OE 10
#define LAT 9
#define CLK 11
#define A A0
#define B A1
#define C A2
#define D A3

/*****************************PM10/2.5***********************************************/
#define TX 14
#define RX 15

SDS011 my_sds;
unsigned char displayTemp[8];
unsigned int Pm25 = 0;
unsigned int Pm10 = 0;
float P25 = 0;     // vrai calcul PM2.5
float P10 = 0;     // vrai calcul PM10
float P25_100 = 0; // vrai calcul PM2.5
float P10_100 = 0; // vrai calcul PM10
float meanP25 = 0; // moyenne PM2.5
float meanP10 = 0; // moyenne PM10
int meanCO2 = 0;   // moyenne CO2

/* CHOISIR NUM DU MODULE AIR **********************************************/

int deviceID = 475;

bool wifi = true;
bool lora = false;

/**********************************************/

RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false, 64);

char Valeur1[5];
char Valeur2[5];
char Valeur3[5];

float Nrouge = 5.1;
float Nvert = 5.1;

// Var I2C
char MtoS[20]; // MasterToSlave
char valo3[5];
char valno2[5];
char CO2[5];
char pm10[5];
char pm25[5];

// Parametres echelle / affichage
int pixel_VL = 48;
int Red;
int Green;
int Blue;

// code couleur de chaque classe de l'echelle
int RGB_0[] = {0, 240, 20};  // classe 0 à 1 (0VL à 0.2VL)
int RGB_1[] = {0, 240, 20};  // classe 1 à 2 (0.2VL à 0.4VL)
int RGB_2[] = {0, 230, 0};   // classe 2 à 3 (0.4VL à 0.6VL)
int RGB_3[] = {255, 255, 0}; // classe 3 à 4 (0.6VL à 0.8VL)
int RGB_4[] = {255, 170, 0}; // classe 4 à 5 (0.8VL à 1VL)
int RGB_5[] = {255, 0, 0};   // classe 5 à 6 (1VL à 2VL)
int RGB_6[] = {128, 0, 0};   // classe 6 (2VL)

// Valeur Limite a modifier
int VLISA = 75; // Horaire
char VISA[5];

// CO2 sur le port de série 1

// choisir sa sonde
NDIRZ16 mySensor = NDIRZ16(&Serial1);
// MHZ19 mySensor;

int Boucle = 1;
int BoucleLora = 1;

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data

boolean newData = false;

// ************************************************ SETUP *********************************************************

void setup()
{
  matrix.begin();        // SCREEN
  Serial.begin(115200);  // Moniteur de Serie
  Serial1.begin(9600);   // SONDE C02
  Serial2.begin(115200); // WEMOS D1 mini (ESP8266) WIFI
  Serial3.begin(9600);   // sonde PM
  my_sds.begin(RX, TX);

  Serial.println("**************");
  Serial.println("MODULE AIR");
  Serial.println("**************");
}

//************************************************************************************

void recvOneChar()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial2.available() > 0 && newData == false)
  {
    rc = Serial2.read();

    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars)
      {
        ndx = numChars - 1;
      }
    }
    else
    {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

//************************************************************************************

void showNewData()
{
  if (newData == true)
  {
    Serial.print("Received from WEMOS:  ");
    Serial.println(receivedChars);
    newData = false;
  }
}

//**************************************Calcul Ecran*********************************************

// Calcule Degrade de couleur en fonction de la valeur, et de la valeur limite du polluant.
int code_RGB2(int VL_pol, int pixel_VL, int Val_pol) // VL_pol = VL du polluant à Afficher, pixel_VL = numero du pixel qui marquera la VL sur la Matrice LED, Val_pol = Valeur renvoyée par la sonde ou l'API
{
  int pixel = Val_pol;
  // classe 0 à 1 (0VL à 0.2VL)
  if (((pixel_VL * 0) <= pixel) && (pixel <= (pixel_VL * 0.2)))
  {
    Red = RGB_0[0] + ((RGB_1[0] - RGB_0[0]) / (pixel_VL * 0.2 - pixel_VL * 0)) * (pixel - pixel_VL * 0); //                   RGB_0[1];
    Green = RGB_0[1] + ((RGB_1[1] - RGB_0[1]) / (pixel_VL * 0.2 - pixel_VL * 0)) * (pixel - pixel_VL * 0);
    Blue = RGB_0[2] + ((RGB_1[2] - RGB_0[2]) / (pixel_VL * 0.2 - pixel_VL * 0)) * (pixel - pixel_VL * 0);

    return Red;
    return Green;
    return Blue;

    // classe 1 à 2 (0.2VL à 0.4VL)
  }
  else if (((pixel_VL * 0.2) < pixel) && (pixel <= (pixel_VL * 0.4)))
  {
    Red = RGB_1[0] + ((RGB_2[0] - RGB_1[0]) / (pixel_VL * 0.4 - pixel_VL * 0.2)) * (pixel - pixel_VL * 0.2); //                   RGB_0[1];
    Green = RGB_1[1] + ((RGB_2[1] - RGB_1[1]) / (pixel_VL * 0.4 - pixel_VL * 0.2)) * (pixel - pixel_VL * 0.2);
    Blue = RGB_1[2] + ((RGB_2[2] - RGB_1[2]) / (pixel_VL * 0.4 - pixel_VL * 0.2)) * (pixel - pixel_VL * 0.2);

    return Red;
    return Green;
    return Blue;

    // classe 2 à 3 (0.4VL à 0.6VL)
  }
  else if (((pixel_VL * 0.4) < pixel) && (pixel <= (pixel_VL * 0.6)))
  {
    Red = RGB_2[0] + ((RGB_3[0] - RGB_2[0]) / (pixel_VL * 0.6 - pixel_VL * 0.4)) * (pixel - pixel_VL * 0.4); //                   RGB_0[1];
    Green = RGB_2[1] + ((RGB_3[1] - RGB_2[1]) / (pixel_VL * 0.6 - pixel_VL * 0.4)) * (pixel - pixel_VL * 0.4);
    Blue = RGB_2[2] + ((RGB_3[2] - RGB_2[2]) / (pixel_VL * 0.6 - pixel_VL * 0.4)) * (pixel - pixel_VL * 0.4);

    return Red;
    return Green;
    return Blue;

    // classe 3 à 4 (0.6VL à 0.8VL)
  }
  else if (((pixel_VL * 0.6) < pixel) && (pixel <= (pixel_VL * 0.8)))
  {
    Red = RGB_3[0] + ((RGB_4[0] - RGB_3[0]) / (pixel_VL * 0.8 - pixel_VL * 0.6)) * (pixel - pixel_VL * 0.6); //                   RGB_0[1];
    Green = RGB_3[1] + ((RGB_4[1] - RGB_3[1]) / (pixel_VL * 0.8 - pixel_VL * 0.6)) * (pixel - pixel_VL * 0.6);
    Blue = RGB_3[2] + ((RGB_4[2] - RGB_3[2]) / (pixel_VL * 0.8 - pixel_VL * 0.6)) * (pixel - pixel_VL * 0.6);

    return Red;
    return Green;
    return Blue;

    // classe 4 à 5 (0.8VL à 1VL)
  }
  else if (((pixel_VL * 0.8) < pixel) && (pixel < (pixel_VL * 1)))
  {
    Red = RGB_4[0] + ((RGB_5[0] - RGB_4[0]) / (pixel_VL * 1 - pixel_VL * 0.8)) * (pixel - pixel_VL * 0.8); //                   RGB_0[1];
    Green = RGB_4[1] + ((RGB_5[1] - RGB_4[1]) / (pixel_VL * 1 - pixel_VL * 0.8)) * (pixel - pixel_VL * 0.8);
    Blue = RGB_4[2] + ((RGB_5[2] - RGB_4[2]) / (pixel_VL * 1 - pixel_VL * 0.8)) * (pixel - pixel_VL * 0.8);

    return Red;
    return Green;
    return Blue;

    // classe 5 à 6 (1VL au delà et l'infini)
  }
  else if ((pixel_VL * 1) <= pixel)
  {
    Red = RGB_5[0] + ((RGB_6[0] - RGB_5[0]) / (pixel_VL * 2 - pixel_VL * 1)) * (pixel - pixel_VL * 1); //                   RGB_0[1];
    Green = RGB_5[1] + ((RGB_6[1] - RGB_5[1]) / (pixel_VL * 2 - pixel_VL * 1)) * (pixel - pixel_VL * 1);
    Blue = RGB_5[2] + ((RGB_6[2] - RGB_5[2]) / (pixel_VL * 2 - pixel_VL * 1)) * (pixel - pixel_VL * 1);

    return Red;
    return Green;
    return Blue;
  }
} // Fin de la fonction code_RGB2

// Calcule Carre de couleur en fonction de la valeur, et de la valeur limite du polluant.
int code_RGB(int VL_pol, int pixel_VL, int Val_pol) // VL_pol = VL du polluant à Afficher, pixel_VL = numero du pixel qui marquera la VL sur la Matrice LED, Val_pol = Valeur renvoyée par la sonde ou l'API
{
  double pixel = (pixel_VL * Val_pol);
  pixel = pixel / VL_pol;

  if (pixel > 64)
  {
    pixel = 64;
  }

  // classe 0 à 1 (0VL à 0.2VL)
  if (((pixel_VL * 0) <= pixel) && (pixel <= (pixel_VL * 0.2)))
  {
    Red = RGB_0[0] + ((RGB_1[0] - RGB_0[0]) / (pixel_VL * 0.2 - pixel_VL * 0)) * (pixel - pixel_VL * 0); //                   RGB_0[1];
    Green = RGB_0[1] + ((RGB_1[1] - RGB_0[1]) / (pixel_VL * 0.2 - pixel_VL * 0)) * (pixel - pixel_VL * 0);
    Blue = RGB_0[2] + ((RGB_1[2] - RGB_0[2]) / (pixel_VL * 0.2 - pixel_VL * 0)) * (pixel - pixel_VL * 0);

    return Red;
    return Green;
    return Blue;

    // classe 1 à 2 (0.2VL à 0.4VL)
  }
  else if (((pixel_VL * 0.2) < pixel) && (pixel <= (pixel_VL * 0.4)))
  {
    Red = RGB_1[0] + ((RGB_2[0] - RGB_1[0]) / (pixel_VL * 0.4 - pixel_VL * 0.2)) * (pixel - pixel_VL * 0.2); //                   RGB_0[1];
    Green = RGB_1[1] + ((RGB_2[1] - RGB_1[1]) / (pixel_VL * 0.4 - pixel_VL * 0.2)) * (pixel - pixel_VL * 0.2);
    Blue = RGB_1[2] + ((RGB_2[2] - RGB_1[2]) / (pixel_VL * 0.4 - pixel_VL * 0.2)) * (pixel - pixel_VL * 0.2);

    return Red;
    return Green;
    return Blue;

    // classe 2 à 3 (0.4VL à 0.6VL)
  }
  else if (((pixel_VL * 0.4) < pixel) && (pixel <= (pixel_VL * 0.6)))
  {
    Red = RGB_2[0] + ((RGB_3[0] - RGB_2[0]) / (pixel_VL * 0.6 - pixel_VL * 0.4)) * (pixel - pixel_VL * 0.4); //                   RGB_0[1];
    Green = RGB_2[1] + ((RGB_3[1] - RGB_2[1]) / (pixel_VL * 0.6 - pixel_VL * 0.4)) * (pixel - pixel_VL * 0.4);
    Blue = RGB_2[2] + ((RGB_3[2] - RGB_2[2]) / (pixel_VL * 0.6 - pixel_VL * 0.4)) * (pixel - pixel_VL * 0.4);

    return Red;
    return Green;
    return Blue;

    // classe 3 à 4 (0.6VL à 0.8VL)
  }
  else if (((pixel_VL * 0.6) < pixel) && (pixel <= (pixel_VL * 0.8)))
  {
    Red = RGB_3[0] + ((RGB_4[0] - RGB_3[0]) / (pixel_VL * 0.8 - pixel_VL * 0.6)) * (pixel - pixel_VL * 0.6); //                   RGB_0[1];
    Green = RGB_3[1] + ((RGB_4[1] - RGB_3[1]) / (pixel_VL * 0.8 - pixel_VL * 0.6)) * (pixel - pixel_VL * 0.6);
    Blue = RGB_3[2] + ((RGB_4[2] - RGB_3[2]) / (pixel_VL * 0.8 - pixel_VL * 0.6)) * (pixel - pixel_VL * 0.6);

    return Red;
    return Green;
    return Blue;

    // classe 4 à 5 (0.8VL à 1VL)
  }
  else if (((pixel_VL * 0.8) < pixel) && (pixel < (pixel_VL * 1)))
  {
    Red = RGB_4[0] + ((RGB_5[0] - RGB_4[0]) / (pixel_VL * 1 - pixel_VL * 0.8)) * (pixel - pixel_VL * 0.8); //                   RGB_0[1];
    Green = RGB_4[1] + ((RGB_5[1] - RGB_4[1]) / (pixel_VL * 1 - pixel_VL * 0.8)) * (pixel - pixel_VL * 0.8);
    Blue = RGB_4[2] + ((RGB_5[2] - RGB_4[2]) / (pixel_VL * 1 - pixel_VL * 0.8)) * (pixel - pixel_VL * 0.8);

    return Red;
    return Green;
    return Blue;

    // classe 5 à 6 (1VL au delà et l'infini)
  }
  else if ((pixel_VL * 1) <= pixel)
  {
    Red = RGB_5[0] + ((RGB_6[0] - RGB_5[0]) / (pixel_VL * 2 - pixel_VL * 1)) * (pixel - pixel_VL * 1); //                   RGB_0[1];
    Green = RGB_5[1] + ((RGB_6[1] - RGB_5[1]) / (pixel_VL * 2 - pixel_VL * 1)) * (pixel - pixel_VL * 1);
    Blue = RGB_5[2] + ((RGB_6[2] - RGB_5[2]) / (pixel_VL * 2 - pixel_VL * 1)) * (pixel - pixel_VL * 1);

    return Red;
    return Green;
    return Blue;
  }
} // Fin de la fonction code_RGB

void degrade()
{
  for (int i = 0; i < 65; i++) // degrade de couleur
  {
    code_RGB2(VLISA, pixel_VL, i);
    matrix.drawLine(i, 28, i, 31, matrix.Color888(Red, Green, Blue));
  }
  code_RGB2(VLISA, pixel_VL, pixel_VL);
  matrix.drawLine(pixel_VL, 26, pixel_VL, 27, matrix.Color888(Red, Green, Blue));
}

void ISA()
{
  int isa = 0;
  isa = atoi(receivedChars);
  Serial.println("Indice ISA: ");
  Serial.println(isa);
  matrix.setCursor(2, 0); // next line
  matrix.setTextSize(1);  // size 1 == 8 pixels high
  matrix.setTextColor(matrix.Color333(7, 7, 7));
  matrix.println("INDICE AIR");
  matrix.setCursor(5, 9); // next line
  matrix.setTextSize(2);  // size 2 == 16 pixels high
  matrix.setTextColor(matrix.Color888(0, 83, 229));
  matrix.println(isa);

  code_RGB(VLISA, pixel_VL, isa);

  if (isa < 100)
  {
    matrix.fillRect(32, 9, 31, 14, matrix.Color888(Red, Green, Blue));
  }
  else if (isa >= 100)
  {
    matrix.fillRect(42, 9, 41, 14, matrix.Color888(Red, Green, Blue));
  }

  degrade();

  int posc2 = (isa * 48) / (VLISA);
  Serial.print("Posc2 = ");
  Serial.println(posc2);
  matrix.drawPixel(posc2, 27, matrix.Color444(255, 255, 255));
  matrix.drawLine(posc2 - 1, 26, posc2 + 1, 26, matrix.Color444(255, 255, 255));
  matrix.drawLine(posc2 - 2, 25, posc2 + 2, 25, matrix.Color444(255, 255, 255));
  delay(8000);

  blackScreen();
}

//************************************************************************************

void PolExt()
{
  int isa_2 = 0;
  isa_2 = atoi(receivedChars);
  if (isa_2 != 0)
  {
    Serial.println("Affichage de l'air extérieur");
    AirExt();
    delay(5000);
    blackScreen();

    ISA();
  }
  else
  {
  }
}

/********************************************SondeCO2********************************************/

void MHZ16DIR() // Lecture + message de prévention du CO2
{
  if (mySensor.measure())
  { // mesure CO2
    matrix.fillRect(0, 8, 64, 24, matrix.Color444(0, 0, 0));
    // CO2
    blackScreen();
    matrix.setCursor(2, 0); // next line
    matrix.setTextSize(1);  // size 1 == 8 pixels high
    matrix.setTextColor(matrix.Color888(0, 82, 228));
    matrix.println(" CO ppm");

    // 2 de CO2 en indice
    matrix.drawLine(20, 3, 21, 3, matrix.Color888(0, 82, 228));
    matrix.drawLine(22, 4, 22, 4, matrix.Color888(0, 82, 228));
    matrix.drawLine(21, 5, 21, 5, matrix.Color888(0, 82, 228));
    matrix.drawLine(20, 6, 20, 6, matrix.Color888(0, 82, 228));
    matrix.drawLine(20, 7, 22, 7, matrix.Color888(0, 82, 228));

    // Petite maison
    petiteMaison();

    matrix.setCursor(5, 9); // next line
    matrix.setTextSize(2);  // size 2 == 16 pixels high

    // Serial.print("CO2 :");
    // Serial.println(mySensor.ppm);

    if (mySensor.ppm <= 800)
    { // CO2 niv bon
      matrix.fillRect(47, 9, 17, 14, matrix.Color444(0, 15, 0));
      matrix.setCursor(21, 25); // next line
      matrix.setTextSize(1);    // size 1 == 8 pixels high
      matrix.setTextColor(matrix.Color333(0, 7, 0));
      matrix.println("BIEN");
      matrix.setCursor(6, 9);
      matrix.setTextSize(2);
      matrix.setTextColor(matrix.Color333(7, 7, 7));
      matrix.println(mySensor.ppm);
    } // fin CO2 niv bon

    if (mySensor.ppm > 800 && mySensor.ppm <= 1700)
    { // CO2 niv moyen
      matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 15, 0));
      matrix.setCursor(7, 25); // next line
      matrix.setTextSize(1);   // size 1 == 8 pixels high
      matrix.setTextColor(matrix.Color333(7, 7, 0));
      matrix.println("AERER SVP");
      matrix.setTextColor(matrix.Color333(7, 7, 7));
      matrix.setCursor(0, 9);
      matrix.setTextSize(2);
      matrix.println(mySensor.ppm);
    } // fin CO2 niv moyen

    if (mySensor.ppm > 1700)
    { // CO2 niv mauvais
      matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 0, 0));
      matrix.setCursor(3, 25); // next line
      matrix.setTextSize(1);   // size 1 == 8 pixels high
      matrix.setTextColor(matrix.Color333(7, 0, 0));
      matrix.println("AERER VITE");
      matrix.setTextColor(matrix.Color333(7, 7, 7));
      matrix.setCursor(0, 9);
      matrix.setTextSize(2);
      matrix.println(mySensor.ppm);
    } // fin CO2 niv mauvais
    snprintf(CO2, sizeof CO2, "%lu", (unsigned long)mySensor.ppm);
    delay(8000);
    blackScreen();
  } // fin mesure CO2
}

/********************************************SondePM********************************************/

// Lecture PM
void SDS011PM()
{
  uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;
  while (Serial3.available() > 0)
  {
    mData = Serial3.read();
    delay(2); // wait until packet is received

    if (mData == 0xAA) // head1 ok
    {
      mPkt[0] = mData;
      mData = Serial3.read();

      if (mData == 0xc0) // head2 ok
      {
        mPkt[1] = mData;
        mCheck = 0;
        for (i = 0; i < 6; i++) // data recv and crc calc
        {
          mPkt[i + 2] = Serial3.read();
          delay(2);
          mCheck += mPkt[i + 2];
        }
        mPkt[8] = Serial3.read();
        delay(1);
        mPkt[9] = Serial3.read();
        if (mCheck == mPkt[8]) // crc ok
        {
          Serial3.flush();

          Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3] << 8);
          Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5] << 8);
          if (Pm25 > 9999)
            Pm25 = 9999;
          if (Pm10 > 9999)
            Pm10 = 9999;

          P25 = (float(Pm25) / 10);
          P10 = (float(Pm10) / 10);

          blackScreen();
          matrix.setCursor(0, 0); // next line
          matrix.setTextSize(1);  // size 1 == 8 pixels high
          matrix.setTextColor(matrix.Color888(0, 82, 228));
          matrix.println("PM10");
          ugm3();
          petiteMaison();

          matrix.setCursor(5, 9); // next line
          matrix.setTextSize(2);  // size 2 == 16 pixels high

          if (P10 <= 15)
          { // PM10 niv IDEAL -> MAJ: IDEAL devient BON
            matrix.fillRect(47, 9, 17, 14, matrix.Color444(0, 15, 0));

            // matrix.setCursor(18, 25); // next line
            // matrix.setTextSize(1);    // size 1 == 8 pixels high
            // matrix.setTextColor(matrix.Color333(0, 7, 0));
            // matrix.println("IDEAL");

            matrix.setCursor(22, 25); // next line
            matrix.setTextSize(1);    // size 1 == 8 pixels high
            matrix.setTextColor(matrix.Color333(0, 7, 0));
            matrix.println("BON");

            if (P10 >= 5 && P10 < 10)
            {
              matrix.setCursor(6, 9); // placement du curseur differents >5 et <10
              matrix.setTextSize(2);
              matrix.setTextColor(matrix.Color333(7, 7, 7));
              matrix.println(P10, 1);
            }
            else if (P10 < 5)
            {
              matrix.setCursor(0, 9);
              matrix.setTextSize(2);
              matrix.setTextColor(matrix.Color333(7, 7, 7));
              matrix.println("< 5"); // affichage borné inf à 5
            }
            else
            {
              matrix.setCursor(0, 9);
              matrix.setTextSize(2);
              matrix.setTextColor(matrix.Color333(7, 7, 7));
              matrix.println(P10, 1);
            }
          } // fin PM10 niv bon

          if (P10 > 75)
          { // CO2 niv mauvais
            // matrix.setTextColor(matrix.Color444(15, 0, 0));
            matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 0, 0));
            matrix.setCursor(3, 25); // next line
            matrix.setTextSize(1);   // size 1 == 8 pixels high
            matrix.setTextColor(matrix.Color333(7, 0, 0));
            matrix.println("MAUVAIS");
            matrix.setTextColor(matrix.Color333(7, 7, 7));
            matrix.setCursor(0, 9);
            matrix.setTextSize(2);
            if (P10 > 99)
            {
              matrix.println(P10, 0);
            }
            else
            {
              matrix.println(P10, 1);
            }
          } // fin PM10 niv mauvais

          if (P10 > 15 && P10 <= 30)
          { // CO2 niv BON  -> MAJ: BON devient MOYEN
            // matrix.setTextColor(matrix.Color444(15, 15, 0));
            matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 15, 0));

            // matrix.setCursor(22, 25); // next line
            // matrix.setTextSize(1);    // size 1 == 8 pixels high
            // matrix.setTextColor(matrix.Color333(7, 7, 0));
            // matrix.println("BON");

            matrix.setCursor(15, 25); // next line
            matrix.setTextSize(1);    // size 1 == 8 pixels high
            matrix.setTextColor(matrix.Color333(7, 7, 0));
            matrix.println("MOYEN");

            matrix.setTextColor(matrix.Color333(7, 7, 7));
            matrix.setCursor(0, 9);
            matrix.setTextSize(2);
            matrix.println(P10, 1);
          }

          if (P10 > 30 && P10 <= 75)
          { // CO2 niv moyen  --> MAJ moyen devient DEGRADE
            // matrix.setTextColor(matrix.Color444(15, 15, 0));
            matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 15, 0));

            // matrix.setCursor(15, 25); // next line
            // matrix.setTextSize(1);    // size 1 == 8 pixels high
            // matrix.setTextColor(matrix.Color333(7, 7, 0));
            // matrix.println("MOYEN");

            matrix.setCursor(3, 25); // next line
            matrix.setTextSize(1);   // size 1 == 8 pixels high
            matrix.setTextColor(matrix.Color333(7, 7, 0));
            matrix.println("DEGRADE");

            matrix.setTextColor(matrix.Color333(7, 7, 7));
            matrix.setCursor(0, 9);
            matrix.setTextSize(2);
            matrix.println(P10, 1);
          }

          delay(8000);
          blackScreen();

          matrix.setCursor(0, 0); // next line
          matrix.setTextSize(1);  // size 1 == 8 pixels high
          matrix.setTextColor(matrix.Color888(0, 82, 228));
          matrix.println("PM2");
          matrix.setCursor(20, 0);
          matrix.drawPixel(18, 6, matrix.Color888(0, 82, 228));
          matrix.println("5");

          ugm3();
          petiteMaison();

          if (P25 < 10)
          { // PM2,5 niv IDEAL -> MAJ: IDEAL devient BON

            matrix.fillRect(47, 9, 17, 14, matrix.Color444(0, 15, 0));

            // matrix.setCursor(18, 25); // next line
            // matrix.setTextSize(1);    // size 1 == 8 pixels high
            // matrix.setTextColor(matrix.Color333(0, 7, 0));
            // matrix.println("IDEAL");

            matrix.setCursor(22, 25); // next line
            matrix.setTextSize(1);    // size 1 == 8 pixels high
            matrix.setTextColor(matrix.Color333(0, 7, 0));
            matrix.println("BON");

            if (P25 < 3)
            {
              matrix.setCursor(0, 9);
              matrix.setTextSize(2);
              matrix.setTextColor(matrix.Color333(7, 7, 7));
              matrix.println("< 3"); // affichage borné inf à 3
            }
            else
            {
              matrix.setCursor(6, 9);
              matrix.setTextSize(2);
              matrix.setTextColor(matrix.Color333(7, 7, 7));
              matrix.println(P25, 1);
            }
          } // fin PM2,5 niv bon

          if (P25 > 50)
          { // PM2,5 niv mauvais
            // matrix.setTextColor(matrix.Color444(15, 0, 0));
            matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 0, 0));
            matrix.setCursor(3, 25); // next line
            matrix.setTextSize(1);   // size 1 == 8 pixels high
            matrix.setTextColor(matrix.Color333(7, 0, 0));
            matrix.println("MAUVAIS");
            matrix.setTextColor(matrix.Color333(7, 7, 7));
            matrix.setCursor(0, 9);
            matrix.setTextSize(2);
            if (P25 > 99)
            {
              matrix.println(P25, 0);
            }
            else
            {
              matrix.println(P25, 1);
            }
          } // fin PM2,5 niv mauvais

          if (P25 >= 10 && P25 <= 20)
          { // PM2,5 niv BON -> MAJ: BON devient MOYEN
            // matrix.setTextColor(matrix.Color444(15, 15, 0));
            matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 15, 0));
            matrix.setTextSize(1); // size 1 == 8 pixels high

            // matrix.setCursor(22, 25); // next line
            // matrix.setTextColor(matrix.Color333(7, 7, 0));
            // matrix.println("BON");

            matrix.setCursor(15, 25); // next line
            matrix.setTextSize(1);    // size 1 == 8 pixels high
            matrix.setTextColor(matrix.Color333(7, 7, 0));
            matrix.println("MOYEN");

            matrix.setTextColor(matrix.Color333(7, 7, 7));
            matrix.setCursor(0, 9);
            matrix.setTextSize(2);
            matrix.println(P25, 1);
          }

          if (P25 > 20 && P25 <= 50)
          { // PM2,5 niv moyen --> MAJ moyen devient DEGRADE
            // matrix.setTextColor(matrix.Color444(15, 15, 0));
            matrix.fillRect(47, 9, 17, 14, matrix.Color444(15, 15, 0));
            matrix.setTextSize(1); // size 1 == 8 pixels high

            // matrix.setCursor(15, 25); // next line
            // matrix.setTextColor(matrix.Color333(7, 7, 0));
            // matrix.println("MOYEN");

            matrix.setCursor(3, 25); // next line
            matrix.setTextColor(matrix.Color333(7, 7, 0));
            matrix.println("DEGRADE");

            matrix.setTextColor(matrix.Color333(7, 7, 7));
            matrix.setCursor(0, 9);
            matrix.setTextSize(2);
            matrix.println(P25, 1);
          }
          return;
        }
      }
    }
  }
  if (Serial3.available() == 0)
  {
    matrix.setCursor(32, 15); // next line
    matrix.setTextSize(1);    // size 1 == 8 pixels high
    matrix.setTextColor(matrix.Color333(7, 0, 0));
    matrix.println("prbl sonde PM");
    delay(5000);
    blackScreen();
  }
}

// *************************************** LOOP **************************************
// **************************************************************************************

void loop()
{
  blackScreen();

  Serial.print("Module Air numéro: ");
  Serial.println(deviceID);
  Serial.print("Boucle Logo: ");
  Serial.println(Boucle);
  Serial.print("Boucle Lora: ");
  Serial.println(BoucleLora);

  if (Boucle == 99)
  {
    Serial.println("Ecran AtmoSud");
    LogoAtmoSud();
    delay(5000);
    blackScreen();
  }

  if (Boucle == 99)
  {
    Serial.println("Ecran Aircarto");
    LogoAirCarto();
    delay(5000);
    blackScreen();
  }

  if (Boucle == 1)
  {
    Serial.println("Ecran Module Air");
    LogoModuleAir();
    delay(5000);
    blackScreen();
    Boucle = 0;
  }

  //  if (Boucle == 1)
  // {
  //   Serial.println("Ecran Gemenos");
  //   LogoGemenos();
  //   delay(5000);
  //   blackScreen();
  // }

  // Pas besoin d'afficher le logo "air interieur" si on n'affiche pas les données "extérieur"
  /*
   AirInterieur();
  delay(5000);
  blackScreen();
  */

  Serial.println("Ecran CO2");
  MHZ16DIR();

  Serial.println("Ecran PM");
  SDS011PM();
  delay(8000); // 7500
  blackScreen();

  Serial.print("CO2 :");
  Serial.println(mySensor.ppm);
  Serial.print("PM25: ");
  Serial.println(P25);
  Serial.print("PM10: ");
  Serial.println(P10);

  // ********************************
  // WIFI
  // *******************************

  // pour envoyer les data au module WIFI: Ne pas utiliser de String
  // on envoie les données séparée par une virgule au format <devicenNum, CO2, PM10, PM2.5>
  // la boucle dure 30 sec

  // WIFI send every 30 sec

  if (wifi)
  {

    Serial2.print("<");
    Serial2.print(deviceID);
    Serial2.print(",");
    Serial2.print(mySensor.ppm);
    Serial2.print(",");
    Serial2.print(P25);
    Serial2.print(",");
    Serial2.print(P10);
    Serial2.print(">");
    Serial2.print("\r\n");
    Serial.println("Data envoyée sur le port de Série 2");
  }
  // ********************************
  // LORA
  // ********************************

  // en lora on veut envoyer toute les 5 min (10 x 0,5min)
  // on fait donc une moyenne des 10 dernières mesures et on envoie
  if (lora)
  {
    // moyenne des mesures
    meanP10 = meanP10 + P10;
    meanP25 = meanP25 + P25;
    meanCO2 = meanCO2 + mySensor.ppm;

    // send every 5 min (9 boucles)

    if (BoucleLora == 10)
    {
      meanP10 = meanP10 / BoucleLora;
      meanP25 = meanP25 / BoucleLora;
      meanCO2 = meanCO2 / BoucleLora;

      Serial2.print("<");
      Serial2.print(deviceID);
      Serial2.print(",");
      Serial2.print(meanCO2);
      Serial2.print(",");
      Serial2.print(meanP25);
      Serial2.print(",");
      Serial2.print(meanP10);
      Serial2.print(">");
      Serial2.print("\r\n");
      Serial.println("Data envoyée sur le port de Série 2 pour LORA");
      BoucleLora = 0;
      meanP10 = 0;
      meanP25 = 0;
      meanCO2 = 0;
    }

    // on incrémenta la Boucle pour le LORA
    BoucleLora = BoucleLora + 1;
  }
  // ********* FIN LORA

  // REPONSE DE L'ESP 32 et Affichage polluants Ext
  //  recvOneChar(); //écoute le port de série 2 pour une réponse
  //  showNewData(); // affiche la réponse
  //  PolExt(); // Affichage écrans polluants extérieur

  // on incrémenta la Boucle pour altérner les logos
  Boucle = Boucle + 1;

  Serial.println("FIN DE LA BOUCLE LOOP ------- ");
  Serial.println("");
}
