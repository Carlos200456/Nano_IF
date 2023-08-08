/* Nano Universal Interface by Meditech s.a. 

   Sketch > Include Library > Manage Libraries...
   Include the following Libraries:
   U8g2 Library

 This code is not in the public domain.

 http://www.meditech.com.ar

 */
#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "..\lib\TimerTwo.h"
#include "..\lib\TimerTwo.cpp"
#include "Functions.h"

// Opcionales de la Interface
#define OLED          // Con Display OLED
#define ROTA          // Con control de Rotacion de Imagen por botones
#define MAGNY         // Con control de Magnificacion por botones (Panel Plano)

#define Scopia 2
#define Cine 3
#define XRay 4
#define PulseIn 5                      // For Integris 3000 Input x-Ray Pulse in Toshiba KV button state
#define AEC 6
#define Mag 7
#define DEBUG 8                        // for Debug pourpouses print input data
#define TurnRight 10
#define ResetRotation 11
#define TurnLeft 12
#define T0 A0
#define T1 A1
#define FocoFino A2
#define AEC_Analog A6

#ifdef OLED
// OLED 0.96"
// U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// OLED 1.3"
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
#endif

String inputString = "";               // a string to hold incoming data
String Tipo ="";
String Signo ="";
String Magnitud ="";
String ACK ="ACK";
String NACK ="NACK";
bool DataReady = false;             // whether the string is complete
bool error = false;                 // whether the string have errors
char TipoIF = 0;                       // X-Ray Pulse and T0, T1 behavior
int inputAEC = 0;
int outputAEC = 0;
int AEC_Limit_In = 0;
int AEC_Limit_UP = 0;
int AEC_Limit_DW = 0;
int AEC_Limit_UP_Cine = 0;
int AEC_Limit_DW_Cine = 0;
int AEC_Analod_Read = 0;
int AEC_Lock_Volt = 0;
int Offset = 0;
float Gain = 0;
float AEC_Voltage = 0;
int IrisActive = 0;
int KVDWActive = 0;
int KVUPActive = 0;
int KVSTActive = 0;
int TT_AEC_Limit_Low = 96;                
int TT_AEC_Limit_Hi = 160;

unsigned int count = 0;
unsigned int XRayPeriod = 80;
unsigned int XRayTime = 10;
unsigned int SuperPulso = 0;
unsigned int InterDelay = 0;
unsigned int InterDelayDefault = 1000;
bool XRayOn = 0;              // Start time of X-Ray
bool NoPaso = true;
bool AEC_Lock = false;
unsigned char AEC_Lock_Qy = 0;
bool Pulse_State = false;
bool Last_Pulse_State = false;
bool debugbool = false;


char SCin = 0;
char CIin = 0;
char Magin = 0;
char MagPass = 0;
unsigned long lastDebounceTimeSC = 0;  // the last time the output SC was toggled
unsigned long debounceDelaySC = 20;    // the debounce time; increase if the output flickers
unsigned long lastDebounceTimeCI = 0;  // the last time the output CI was toggled
unsigned long debounceDelayCI = 20;    // the debounce time; increase if the output flickers
unsigned long lastDebounceTimeMag = 0; // the last time the output Mag was toggled
unsigned long debounceDelayMag = 50;   // the debounce time; increase if the output flickers
unsigned long Presed = 0;
int buttonStateSC;                     // the current reading from the input pin
int buttonStateCI;                     // the current reading from the input pin
int buttonStateMag;                    // the current reading from the input pin
int lastButtonStateSC = 0;           // the previous reading from the input pin SC
int lastButtonStateCI = 0;           // the previous reading from the input pin CI
int lastButtonStateMag = 0;          // the previous reading from the input pin Mag
int eeAddress; //EEPROM address to start reading from
int IrisServo = 90;
int LimUp = 0;
int LimDn = 0;
unsigned char linea = 4;
unsigned char RX_Number = 0;
unsigned char Cursor = 0;

Servo servo_iris;                       //initialize a servo object for the connected servo

void setup() {
  // initialize serial:
  Serial.begin(57600);
  // reserve 20 bytes for the inputString:
  inputString.reserve(20);
  TipoIF = ReadEEPROM(4);             // TipoIF = 0 ==> Normal, TipoIF = 1 ==> Integris 3000, TipoIF = 2 ==> Toshiba, TipoIF = 3 ==> BH 5000

  #ifdef OLED
  u8x8.begin();  // initialize with the I2C
  u8x8.setPowerSave(0);
  // init done
  u8x8.setFont(u8x8_font_pxplustandynewtv_r);  // u8x8_font_torussansbold8_r, u8x8_font_chroma48medium8_r, u8x8_font_victoriamedium8_r, u8x8_font_pxplustandynewtv_r
  u8x8.draw1x2String(0,0,"Meditech s.a.");
  u8x8.setCursor(0,2);             // Column, Row
  u8x8.print("Nano Interface 2");
  if (TipoIF == 1){
    u8x8.setCursor(0,3);           // Column, Row
    u8x8.print("Integris 3000");
  }
  if (TipoIF == 2){
    u8x8.setCursor(0,3);           // Column, Row
    u8x8.print("Toshiba");
  }
  if (TipoIF == 3){
    u8x8.setCursor(0,3);           // Column, Row
    u8x8.print("BH 5000");
    InterDelayDefault = 2000;      // Retardo entre Pedales para evitar cuelgue del Generador
  }
  u8x8.setCursor(0,4);             // Column, Row
  u8x8.print("Version 4.Git");     // SOFTWARE VERSION ---------------------------<<<<<<<<<<<<<<<<
  delay(2000);
  u8x8.clearDisplay();
  #endif

  if (TipoIF == 1){
    debounceDelaySC = 50;     // the debounce time; increase if the output flickers
    debounceDelayCI = 500;    // the debounce time; increase if the output flickers
  }

  pinMode(Scopia, INPUT_PULLUP);
  pinMode(Cine, INPUT_PULLUP);
  pinMode(DEBUG, INPUT_PULLUP);
  pinMode(Mag, INPUT_PULLUP);
  pinMode(TurnLeft, INPUT_PULLUP);
  pinMode(TurnRight, INPUT_PULLUP);
  pinMode(ResetRotation, INPUT_PULLUP);
  pinMode(AEC_Analog, INPUT);
  pinMode(XRay, OUTPUT);
  pinMode(AEC, OUTPUT);
  pinMode(T0, OUTPUT);
  pinMode(T1, OUTPUT);
  pinMode(FocoFino, OUTPUT);
  pinMode(PulseIn, INPUT_PULLUP);     // For Integris 3000 Input x-Ray Pulse
  LimUp = ReadEEPROM(0);
  LimDn = ReadEEPROM(1);
  AEC_Limit_UP = ReadEEPROM(2);
  AEC_Limit_DW = ReadEEPROM(3);
  AEC_Limit_UP_Cine = ReadEEPROM(5);
  AEC_Limit_DW_Cine = ReadEEPROM(6);
  Offset = ReadEEPROM(7);
  Gain = float ((float) (ReadEEPROM(8)) / 10);
  AEC_Lock_Volt = ReadEEPROM(9);
  TT_AEC_Limit_Low = ReadEEPROM(10);                
  TT_AEC_Limit_Hi = ReadEEPROM(11);

  IrisActive = 500;                 // Activar el Servo por 500ms para que llegue a la posicion "IrisServo"

  outputAEC = map(0, 0, 255, AEC_Limit_DW, AEC_Limit_UP);             // Valor de arranque del AEC Problemas con Philips es necesario que arranque de abajo
  Timer2.EnableTimerInterrupt(Xray, 1000);                            // Interrupt every 1 milliseconds cuando hay que controlar los pulsos
  if (TipoIF == 3){                       // Reset BH-5000 y IDITV
    digitalWrite(FocoFino, HIGH);
    delay(1000);
    digitalWrite(FocoFino, LOW);
  }
  debugbool = !digitalRead(DEBUG);
}


// ------------------ Interupt for Pulse Generation -----------------------------------------
void Xray(void){
  if ((!buttonStateSC || !buttonStateCI) && XRayOn && !debugbool) {        // Demora para empesar a pulsar en Fluoro รณ Cine
    if (TipoIF == 1){                                 // Integris 3000
        digitalWrite (XRay, !digitalRead(PulseIn));   // Repetir el pulso del Generador Invertido para la Camara 
    } else {
      if (!KVSTActive) {
        if (count == 0) {
          digitalWrite (XRay, HIGH);
        }
      }
      // if ((TipoIF == 3) && (count == 2)) digitalWrite (T1, LOW);   // Control de la señal de ABC en BH 5000
      
      count++;
      
      if (count > XRayTime ) {
        digitalWrite (XRay, LOW);
      }
      // if ((TipoIF == 3) && (count > (XRayTime + 10))) digitalWrite (T1, HIGH);
      if (count == 5) AEC_Analod_Read = analogRead(AEC_Analog);     // Lee el Valor de AEC durante el Pulso
      if (count >= XRayPeriod) count = 0;
    } 
  } else {
    count = 0;
  }
  if (IrisActive) IrisActive -= 1;
  if (KVDWActive) KVDWActive -= 1;
  if (KVUPActive) KVUPActive -= 1;
  if (KVSTActive) KVSTActive -= 1;
  if (SuperPulso) SuperPulso -= 1;
  if (InterDelay) InterDelay -= 1;
}


void loop() {
  if (debugbool) {
    // digitalWrite (T1, LOW);                   // Anula la restriccion de ABC en DEBUG para poder calibrar
    AEC_Analod_Read = analogRead(AEC_Analog); // Lee el valor de ABC en DEBUG para poder calibrar
  } 
  // Read the string when a newline arrives:
  if (DataReady) {
    Tipo = inputString.substring(0,1);
    Signo = inputString.substring(1,2);
    Magnitud = inputString.substring(2);

    #ifdef OLED
    u8x8.clearLine(linea);
    u8x8.setCursor(0,linea);              // Column, Row
    u8x8.print(inputString);
    u8x8.setCursor(8,linea);              // Column, Row
    u8x8.print(RX_Number);
    linea += 1;
    RX_Number += 1;
    if (linea == 8) linea = 4;
    #endif
    
    if (Tipo == "Y"){                     // X-Ray FPS., valid values 1 - 30
      if (Signo == "U"){
        if ((1 <= Magnitud.toInt())&&( Magnitud.toInt() <= 30)){
          XRayPeriod = (1 / float (Magnitud.toInt())) * 1000;
          if (XRayTime > XRayPeriod){
            XRayTime = XRayPeriod - 15;
          }
          if ((XRayPeriod - XRayTime) < 15){
            XRayTime = XRayPeriod - 15;
          }
        } else error = true;
      }
      goto jmp;
    }
    
    if (Tipo == "I"){                   // X-Ray On Time in ms., valid values 1 - 999
      if (Signo == "U"){
        if ((1 <= Magnitud.toInt())&&( Magnitud.toInt() <= 999)){
          XRayTime = Magnitud.toInt();
          if (XRayTime > XRayPeriod){
            XRayTime = XRayPeriod - 15;
            error = true;
          }
          if ((XRayPeriod - XRayTime) < 15){
            XRayTime = XRayPeriod - 15;
            error = true;
          }
        } else error = true;
      }
      if(TipoIF == 0){             // Interface Normal Meditech      
        switch (XRayTime) {
          case 6:
            digitalWrite(T0, LOW);
            digitalWrite(T1, LOW);
            digitalWrite(FocoFino, HIGH);
            break;
          case 20:
            digitalWrite(T0, HIGH);
            digitalWrite(T1, LOW);
            digitalWrite(FocoFino, LOW);
            break;
          case 21:
            digitalWrite(T0, HIGH);
            digitalWrite(T1, LOW);
            digitalWrite(FocoFino, HIGH);
            break;
          case 40:
            digitalWrite(T0, LOW);
            digitalWrite(T1, HIGH);
            digitalWrite(FocoFino, LOW);
            break;
          case 41:
            digitalWrite(T0, LOW);
            digitalWrite(T1, HIGH);
            digitalWrite(FocoFino, HIGH);
            break;
          case 80:
            digitalWrite(T0, HIGH);
            digitalWrite(T1, HIGH);
            digitalWrite(FocoFino, LOW);
            break;
          case 81:
            digitalWrite(T0, HIGH);
            digitalWrite(T1, HIGH);
            digitalWrite(FocoFino, HIGH);
            break;
          default:
            digitalWrite(T0, LOW);
            digitalWrite(T1, LOW);
            digitalWrite(FocoFino, LOW);
        }
      }
      if(TipoIF == 3){             // Interface BH-5000 Hi / Low Gain II
        if(XRayTime > 20) digitalWrite(T1, LOW); else digitalWrite(T1, HIGH);
      }
      goto jmp;
    }

    if (Tipo == "A"){                   // AEC Value, valid values 0 - 255
      if ((0 <= Magnitud.toInt())&&( Magnitud.toInt() <= 255)){
        inputAEC = Magnitud.toInt();
        if (inputAEC > 255) inputAEC = 255;
        if (inputAEC < 0) inputAEC = 0;
      } else error = true;
      goto jmp;
    }

    if (Tipo == "L"){                     // Activar el Lock de Kv.
      if (Signo == "K"){
        if (Magnitud.toInt() == 0){
          AEC_Lock = false;
        } else {
          AEC_Lock = true;
          AEC_Lock_Qy = (unsigned char)Magnitud.toInt();
        }
      }
      goto jmp;
    }

    if (Tipo == "D"){                     // Activar el Modo Debug
      if (Signo == "B"){
        if (Magnitud.toInt() == 0){
          debugbool = false;
        } else {
          debugbool = true;
        }
      }
      goto jmp;
    }

    if ((Tipo == "R") && (TipoIF == 3) && debugbool){      // Rayos en BH-5000
      if (Signo == "X"){
        SuperPulso = Magnitud.toInt();
      }
      goto jmp;
    }

    if (Tipo == "S"){                   // Servo Iris value, valid values  0 - 31
      if ((0 <= Magnitud.toInt())&&( Magnitud.toInt() <= 31)){
        IrisServo = Magnitud.toInt();
        if (IrisServo > 31) IrisServo = 31;
        if (IrisServo < 0 ) IrisServo = 0;
        IrisServo = map(IrisServo, 0, 31, LimDn, LimUp);
        IrisActive = 800;                // Tiempo para que el iris llegue a la posicion correcta, verificar con maximo recorrido
        // Servo_to_Pos(IrisServo);      //command to rotate the servo to the specified IrisServo angle
      } else error = true;
      goto jmp;
    }

    if ((Tipo == "C")&&(debugbool)){     // Command for Iris Calibration
      if (Signo == "U"){
        IrisServo += 1;
        Serial.print(IrisServo);
        IrisActive = 1500;
      }
      if (Signo == "D"){
        IrisServo -= 1;
        Serial.print(IrisServo);
        IrisActive = 1500;
      }
      goto jmp;
    }

    if ((Tipo == "B")&&(debugbool)){    // Command for AEC Calibration
      if (Signo == "U"){
        AEC_Limit_In += Magnitud.toInt();
      }
      if (Signo == "D"){
        AEC_Limit_In -= Magnitud.toInt();
      }
      if (AEC_Limit_In > 255) AEC_Limit_In = 255;
      if (AEC_Limit_In < 0) AEC_Limit_In = 0;
      goto jmp;
    }

    if ((Tipo == "W")&&(debugbool)){         // -------------------- Servo Iris Limits Calibration --------------------
      if (Signo == "U") {
        WriteEEPROM(0, IrisServo);
        delay(50);
        LimUp = ReadEEPROM(0);
      }
      if (Signo == "D") {
        WriteEEPROM(1, IrisServo);
        delay(50);
        LimDn = ReadEEPROM(1);
      }
      goto jmp;
    }

    if ((Tipo == "T")&&(debugbool)){         // -------------------- Interface Mode Program --------------------
      if (Signo == "T") WriteEEPROM(4, 2);             // Toshiba ==> T0 y T1 suben y bajan los KV en Cine, PulseIn KV button state
      if (Signo == "I") WriteEEPROM(4, 1);             // Integris 3000 el pulso de Rayos X es externo
      if (Signo == "N") WriteEEPROM(4, 0);             // Interface Normal Meditech
      if (Signo == "B") WriteEEPROM(4, 3);             // Interface BH 5000
      delay(50);
      WriteEEPROM(0, 120);
      delay(50);
      WriteEEPROM(1, 60);
      delay(50);
      WriteEEPROM(2, 255);
      delay(50);
      WriteEEPROM(3, 0);
      delay(50);
      WriteEEPROM(5, 255);
      delay(50);
      WriteEEPROM(6, 0);
      delay(50);
      WriteEEPROM(7, 417);
      delay(50);
      WriteEEPROM(8, 403);
      delay(50);
      WriteEEPROM(9, 128);
      delay(50);
      WriteEEPROM(10, 96);
      delay(50);
      WriteEEPROM(11, 160);
      delay(50);
      LimUp = ReadEEPROM(0);
      LimDn = ReadEEPROM(1);
      AEC_Limit_UP = ReadEEPROM(2);
      AEC_Limit_DW = ReadEEPROM(3);
      TipoIF = ReadEEPROM(4);
      AEC_Limit_UP_Cine = ReadEEPROM(5);
      AEC_Limit_DW_Cine = ReadEEPROM(6);
      Offset = ReadEEPROM(7);
      Gain = float (ReadEEPROM(8));
      Gain = Gain / 10;
      AEC_Lock_Volt = ReadEEPROM(9);
      TT_AEC_Limit_Low = ReadEEPROM(10);                
      TT_AEC_Limit_Hi = ReadEEPROM(11);
      delay(500);
      software_Reset();
      goto jmp;
    }

    if ((Tipo == "V")&&(debugbool)){      // -------------------- Offset y Gain Voltimeter --------------------
      if (Signo == "O") WriteEEPROM(7, Magnitud.toInt());
      if (Signo == "G") WriteEEPROM(8, Magnitud.toInt());
      delay(50);
      Offset = ReadEEPROM(7);
      Gain = float ((float) (ReadEEPROM(8)) / 10);
      goto jmp;
    }

    if ((Tipo == "Z")&&(debugbool)){                   // Command to Write AEC Calibration
      if (Signo == "U") {
        WriteEEPROM(2, AEC_Limit_In);
        delay(50);
        AEC_Limit_UP = ReadEEPROM(2);
      }
      if (Signo == "D") {
        WriteEEPROM(3, AEC_Limit_In);
        delay(50);
        AEC_Limit_DW = ReadEEPROM(3);
      }
      goto jmp;
    }
    
    if ((Tipo == "X")&&(debugbool)){                   // Command to Write AEC Calibration
      if (Signo == "U") {
        WriteEEPROM(5, AEC_Limit_In);
        delay(50);
        AEC_Limit_UP_Cine = ReadEEPROM(5);
      }
      if (Signo == "D") {
        WriteEEPROM(6, AEC_Limit_In);
        delay(50);
        AEC_Limit_DW_Cine = ReadEEPROM(6);
      }
      goto jmp;
    }

    if ((Tipo == "K")&&(debugbool)){                   // Command to Write AEC Lock Calibration
      if (Signo == "L") {
        WriteEEPROM(9, AEC_Limit_In);
        delay(50);
        AEC_Lock_Volt = ReadEEPROM(9);
      }
      if (Signo == "D") {
        WriteEEPROM(10, AEC_Limit_In);
        delay(50);
        TT_AEC_Limit_Low = ReadEEPROM(10);
      }
      if (Signo == "U") {
        WriteEEPROM(11, AEC_Limit_In);
        delay(50);
        TT_AEC_Limit_Hi = ReadEEPROM(11);
      }
      goto jmp;
    }
    
    error = true;      // Si el comando de entrada es desconocido setear error
    
    jmp:
    
    if (debugbool){
      Serial.print("RX: ");
      Serial.print(inputString);
      Serial.print("Tipo: ");
      Serial.print(Tipo);
      Serial.print(" ,Signo: ");
      Serial.print(Signo);
      Serial.print(" ,Magnitud: ");
      Serial.println(Magnitud);
      delay(20);
      Serial.print("IF Tipo: ");
      Serial.println(TipoIF, DEC);
      delay(20);
      Serial.print("AEC Limit: ");
      Serial.println(AEC_Limit_In, DEC);
      delay(20);
      Serial.print("AEC Limit UP: ");
      Serial.println(AEC_Limit_UP, DEC);
      delay(20);
      Serial.print("AEC Limit DW: ");
      Serial.println(AEC_Limit_DW, DEC);
      delay(20);
      Serial.print("AEC Limit UP Cine: ");
      Serial.println(AEC_Limit_UP_Cine, DEC);
      delay(20);
      Serial.print("AEC Limit DW Cine: ");
      Serial.println(AEC_Limit_DW_Cine, DEC);
      delay(20);
      Serial.print("AEC Lock Volt: ");
      Serial.println(AEC_Lock_Volt, DEC);
      delay(20);
      Serial.print("T1 Limit Low(Subir): ");
      Serial.println(TT_AEC_Limit_Low, DEC);
      delay(20);
      Serial.print("T0 Limit Hi(Bajar): ");
      Serial.println(TT_AEC_Limit_Hi, DEC);
      delay(20);
      Serial.print("Servo Iris: ");
      Serial.println(IrisServo, DEC);
      delay(20);
      Serial.print("Servo Limit Up: ");
      Serial.println(LimUp, DEC);
      delay(20);
      Serial.print("Servo Limit Dn: ");
      Serial.println(LimDn, DEC);
      delay(20);
      Serial.print("Int Time: ");
      Serial.println(XRayTime, DEC);
      delay(20);
      Serial.print("Period: ");
      Serial.println(XRayPeriod, DEC);
      delay(20);
      Serial.print("Offset: ");
      Serial.println(Offset, DEC);
      delay(20);
      Serial.print("Gain: ");
      Serial.println(Gain, 2);
    }

    #ifdef OLED
    if (linea == 4) Cursor = 7; else Cursor = linea -1;
    if (error) {
      u8x8.setCursor(12,Cursor);   // Column, Row
      u8x8.print("E");
    } else {
      u8x8.setCursor(12,Cursor);   // Column, Row
      u8x8.print(" ");
    }
    #endif

    if (error) {
      Serial.println(NACK);
    } else {
      Serial.println(ACK);
    }
    
    error = false;
    // clear the string:
    inputString = "";
    DataReady = false;
  }

  if (TipoIF == 3){           // BH 5000 Accionamiento Se;al RQ_SN_X con retardo mediante Pin T0 = A0
    if (debugbool) {          // SuperPulso de Rayos X para Adaptacion BH-5000
      if (SuperPulso) {
        digitalWrite (XRay, HIGH);
        digitalWrite(T0, HIGH);
      } else {
        digitalWrite (XRay, LOW);
        digitalWrite(T0, LOW);
      } 
    } else {
      if ((!buttonStateSC || !buttonStateCI) && !KVSTActive) digitalWrite(T0, HIGH);
      else {
      digitalWrite(T0, LOW);
      }
    }
  }


  Servo_to_Pos(IrisServo);      //command to rotate the servo to the specified IrisServo angle

  #ifdef OLED
  if (debugbool){
    u8x8.setCursor(15,4);   // Column, Row
    u8x8.print("D");
  } else {
    u8x8.setCursor(15,4);   // Column, Row
    u8x8.print(" ");
  }
  #endif

  if (debugbool) {
    outputAEC = AEC_Limit_In;
  } else {
    if (!buttonStateCI) outputAEC = map(inputAEC, 0, 255, AEC_Limit_DW_Cine, AEC_Limit_UP_Cine); else outputAEC = map(inputAEC, 0, 255, AEC_Limit_DW, AEC_Limit_UP);
  }

  if (AEC_Lock && (AEC_Lock_Qy == 0)){
    outputAEC = AEC_Lock_Volt;
    #ifdef OLED
      u8x8.drawString(14, 0, "L"); 
    #endif
  }

  // --------- Detector de Pulsos de RX de Cine ------------------
  if (!buttonStateCI){
    Pulse_State = digitalRead(XRay);
    if (digitalRead(PulseIn) && (TipoIF == 2)) KVSTActive = 10;
  }
  
  if (Pulse_State != Last_Pulse_State) {
    // if the state has changed, decrement the counter
    if ((outputAEC < TT_AEC_Limit_Low)&&(!buttonStateCI)){
      if (!Pulse_State) KVUPActive = 10;
    }
    if ((outputAEC > TT_AEC_Limit_Hi)&&(!buttonStateCI)){
      if (!Pulse_State) KVDWActive = 10;
    }
    if (Pulse_State) {
      // if the current state is HIGH then the pin went from off to on:
      if (AEC_Lock_Qy) AEC_Lock_Qy--;
     }
  }
  Last_Pulse_State = Pulse_State;
  
  if (TipoIF == 2){   // Toshiba
    if (KVUPActive) digitalWrite(T1, HIGH); else digitalWrite(T1, LOW);
    if (KVDWActive) digitalWrite(T0, HIGH); else digitalWrite(T0, LOW);
    if (KVSTActive) digitalWrite(FocoFino, HIGH); else digitalWrite(FocoFino, LOW);
  }
  if (TipoIF == 1){                                      // Integris 3000 Accionamiento del pedal de escopia con retardo mediante Pin FocoFino = A2
    if (!buttonStateSC) digitalWrite(FocoFino, HIGH);
    else {
      if (!KVSTActive)digitalWrite(FocoFino, LOW);
    }
  }

  if (TipoIF != 1){
    if ((TipoIF == 3) && (XRayPeriod > 200)) analogWrite(AEC, 10); else analogWrite(AEC, char(outputAEC));
    // AEC_Analod_Read = analogRead(AEC_Analog);  // Ahora Lee en la interupcion de XRay por el corte de AEC
    AEC_Voltage = float ((AEC_Analod_Read - Offset) / Gain);
    #ifdef OLED
    u8x8.setCursor(0,2);   // Column, Row
    u8x8.print("ABC: ");
    u8x8.print(AEC_Voltage, 2);
    u8x8.print(" V    ");
    #endif
  }
// Fluoro Input ------------------------------
  if (!InterDelay) SCin = digitalRead(Scopia);
  if (SCin != lastButtonStateSC) {
    // reset the debouncing timer
    lastDebounceTimeSC = millis();
  }
  if ((millis() - lastDebounceTimeSC) > debounceDelaySC) {        // Demora para empesar a pulsar en Cine
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // if the button state has changed:
    if (SCin != buttonStateSC) {
      buttonStateSC = SCin;
      // only toggle the FluoroOn if the new Scopia button state is LOW
      if ((buttonStateSC == HIGH) && (buttonStateCI == HIGH)) {
        if (TipoIF == 1) KVSTActive = 50;                        // Demora para cortar Scopia en Roadmapping
        Serial.println("FluoroOff");
        inputAEC = AEC_Lock_Volt;                               // Salida de AEC al valor elegido al finalizar Fluoro
        #ifdef OLED
        u8x8.clearLine(0);
        u8x8.clearLine(1);
        u8x8.draw1x2String(0,0,"FluoroOff");
        #endif
        PulsoExtra(); 
        InterDelay = InterDelayDefault;
      }else {
        if (buttonStateCI){
          if (TipoIF == 3) KVSTActive = 500;                        // Demora para habilitar RQ_SN_X en BH 5000 en Fluoroscopia
          Serial.println("FluoroOn");
          #ifdef OLED
          u8x8.clearLine(0);
          u8x8.clearLine(1);
          u8x8.draw1x2String(0,0,"FluoroOn");
          #endif
          Presed = millis();
         // delay (2);
        }
      }
    }
  }
  lastButtonStateSC = SCin;

  
// Cine Input ------------------------------
  if (!InterDelay) CIin = digitalRead(Cine);
  if (CIin != lastButtonStateCI) {
    // reset the debouncing timer
    lastDebounceTimeCI = millis();
  }
  if ((millis() - lastDebounceTimeCI) > debounceDelayCI) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // if the button state has changed:
    if (CIin != buttonStateCI) {
      buttonStateCI = CIin;
      // only toggle the CineOn if the new Cine button state is LOW
      if ((buttonStateSC == HIGH) && (buttonStateCI == HIGH)) {
        Serial.println("CineOff");
        inputAEC = AEC_Lock_Volt;                               // Salida de AEC al valor elegido al finalizar Cine
        AEC_Lock = false;
        #ifdef OLED
        u8x8.clearLine(0);
        u8x8.clearLine(1);
        u8x8.draw1x2String(0,0,"CineOff");
        #endif
        PulsoExtra();
        InterDelay = InterDelayDefault;
      }else {
        if (buttonStateSC){
          if (TipoIF == 3) KVSTActive = 1000;                        // Demora para habilitar RQ_SN_X en BH 5000 en Cine
          Serial.println("CineOn");
          #ifdef OLED
          u8x8.clearLine(0);
          u8x8.clearLine(1);
          u8x8.draw1x2String(0,0,"CineOn");
          #endif
          Presed = millis();
        }
      }
    }
  }
  lastButtonStateCI = CIin;

  if ((millis() - Presed) > 250) XRayOn = true;        // Demora para empesar a pulsar en Fluoro รณ Cine

// Mag Input ------------------------------
  #ifdef MAGNY
  Magin = digitalRead(PulseIn);
  if (Magin != lastButtonStateMag) {
    // reset the debouncing timer
    lastDebounceTimeMag = millis();
  }
  if ((millis() - lastDebounceTimeMag) > debounceDelayMag) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // if the button state has changed:
    if (Magin != buttonStateMag) {
      buttonStateMag = Magin;
      // only toggle the CineOn if the new Cine button state is LOW
      if (buttonStateMag == HIGH) {

        }else {
          MagPass += 1;
          switch (MagPass){
            case 1:{
              Serial.println("Mag1");
              #ifdef OLED
              u8x8.clearLine(0);
              u8x8.clearLine(1);
              u8x8.draw1x2String(0,0,"Mag1");
              #endif
              delay (100);
            break;
            }
            case 2:{
              Serial.println("Mag2");
              #ifdef OLED
              u8x8.clearLine(0);
              u8x8.clearLine(1);
              u8x8.draw1x2String(0,0,"Mag2");
              #endif
              delay (100);
            break;
            }
            case 3:{
              Serial.println("Overview");
              #ifdef OLED
              u8x8.clearLine(0);
              u8x8.clearLine(1);
              u8x8.draw1x2String(0,0,"Overview");   // Column, Row
              #endif
              delay (100);
              MagPass = 0;
            break;
            }
          }
      }
    }
  }
  lastButtonStateMag = Magin;
  #endif

  #ifdef OLED
  if (digitalRead(XRay)) {
    u8x8.drawString(15, 0, "P"); 
  } else {
    u8x8.drawString(15, 0, " "); 
  }
  #endif

// ------------------ Image Rotation Control -------------------------
  #ifdef ROTA
  if (!digitalRead(TurnLeft)){
    Serial.println("UDMLEK");
    #ifdef OLED
    u8x8.clearLine(0);
    u8x8.clearLine(1);
    u8x8.draw1x2String(0,0,"UDMLEK");
    #endif
    delay (50);
  }
  if (!digitalRead(TurnRight)){
    Serial.println("UDMRIK");
    #ifdef OLED
    u8x8.clearLine(0);
    u8x8.clearLine(1);
    u8x8.draw1x2String(0,0,"UDMRIK");
    #endif
    delay (50);
  }
  if (!digitalRead(ResetRotation)){
    Serial.println("UDMREK");
    #ifdef OLED
    u8x8.clearLine(0);
    u8x8.clearLine(1);
    u8x8.draw1x2String(0,0,"UDMREK");
    #endif
    delay (60);
  }
  #endif
}   // -------------- End of main Loop ---------------------


/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      DataReady = true;
    }
  }
}

int ReadEEPROM(int i){
  int Data;
  eeAddress = sizeof(int) * i; 
  //Get the int data from the EEPROM at position 'eeAddress'
  EEPROM.get(eeAddress, Data);
  return Data;
}

void WriteEEPROM(int i, int Data){
  eeAddress = sizeof(int) * i; 
  EEPROM.put(eeAddress, Data);
}

void PulsoExtra(){
  XRayOn = false;
  delay (XRayPeriod - XRayTime);
  digitalWrite (XRay, HIGH);
  delay (XRayTime);
  digitalWrite (XRay, LOW);
  if (TipoIF == 3) digitalWrite (T1, HIGH);
}

void Servo_to_Pos(int i){
  if (IrisActive && NoPaso){
    servo_iris.attach(9);       // attach the signal pin of servo to Pin9 of Arduino Nano
    NoPaso = false;
  }
  servo_iris.write(i);        // command to rotate the servo
  if (!IrisActive){
    servo_iris.detach();        // adetach the signal of servo
    NoPaso = true;
  }
}

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
asm volatile ("  jmp 0");  
}  
