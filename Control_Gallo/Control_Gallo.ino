#include <SPI.h>

//---------------------------------------------

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

//------------------------------------ DEFINICIONES --------------------------------
//Para comunicaciones SPI
// for Due/Mega/Leonardo use the hardware SPI pins (which are different)
#define _sclk 52
#define _mosi 51
#define _miso 50
#define ssdriver2 49

#define DEBUG

//Para comunicaciones Bluetoot
#define    STX          0x02
#define    ETX          0x03
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)



#define SONAR_TRIGGER_PIN_CENTRO     2
#define SONAR_ECHO_PIN_CENTRO        3
#define SONAR_TRIGGER_PIN_IZQ     4
#define SONAR_ECHO_PIN_IZQUIERDA        5
#define SONAR_TRIGGER_PIN_DERECHA     6
#define SONAR_ECHO_PIN_DERECHA        7
#define SONAR_TRIGGER_PIN_TRASERO    8
#define SONAR_ECHO_PIN_TRASERO      9 
//------------------------------------ VARIABLES -----------------------------------
//Para comunicaciones SPI
uint8_t SPIresult;
//Para Driver 2
int VM1=1;              //+/- 4096 12 bits o  +/-1023 10 bits
int VM2=1;
float Vmax= 4095;
int variable_estado=1;

//Para comunicacion Bluetooth
int incomingByte = 0;                                   // for incoming serial data
byte Enable=0;
byte Prog1=0;
byte Prog2=0;
byte Prog3=0;
byte Prog4=0;
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
float dir=0,vel=0;                                      // para calcular velocidades con joystic
float auxf1=0;                                          // auxiliar de calculos float
String displayStatus = "xxxx";                          // message to Android device

unsigned long long_pulso;
int distancia_centro=0,distancia_izq=0,distancia_der=0,distancia_trasera=0;

//------------------------------------ CONFIGURACION -------------------------------
void setup() 
{



  
  //Serial para depuracion
  #ifdef DEBUG
      Serial.begin(9600);
      while (!Serial);
      Serial.flush();
      Serial.println("Checando comunicacion con MEGA8!"); 
      Serial.flush();
  #endif
  //Serial para bluetooth a 115000 baudios
  Serial1.begin(115200);
  //SPI para comunicacion con perifericos esclavos
  SPI.begin();
  
  // initalize the  data ready and chip select pins:
  digitalWrite(ssdriver2, HIGH); //LOW
  pinMode(ssdriver2, OUTPUT); //INPUT, INPUT_PULLUP
  
  
  
  while(Serial1.available())  Serial1.read();         // empty RX buffer


   
   pinMode(SONAR_TRIGGER_PIN_CENTRO, OUTPUT);
   pinMode(SONAR_ECHO_PIN_CENTRO, INPUT);
   pinMode(SONAR_TRIGGER_PIN_IZQ, OUTPUT);
   pinMode(SONAR_ECHO_PIN_IZQUIERDA, INPUT);
   pinMode(SONAR_TRIGGER_PIN_DERECHA, OUTPUT);
   pinMode(SONAR_ECHO_PIN_DERECHA, INPUT);
  
   pinMode(SONAR_TRIGGER_PIN_TRASERO, OUTPUT);
   pinMode(SONAR_ECHO_PIN_TRASERO, INPUT);
}



//********************************* PROGRAMA PRINCIPAL *****************************
void loop(void) 
{
  //Comunicacion Bluetooth ----------------------
  
  if(recibeBluetoothData()) //Si se recibio un comando BlueTooth
  {
    sendBlueToothData(); 
  }
  

if (variable_estado==0)
{
distancia_medida();
if (distancia_centro<50 || distancia_izq<50 || distancia_der<50  )
   {

  VM2=0;
  VM1=0;
  EnviaVelocidad();  
  }
else 
{
    

if ((distancia_izq<200 && distancia_izq>50 )|| (distancia_der<200 &&distancia_der>50))
{
    VM1=distancia_izq*20;
    VM2=distancia_der*20;
EnviaVelocidad();  
}
}

   
}



}


//*********************************** FUNCIONES ************************************
//---------------------------- RECIBE COMANDOS BLUETOOTH ---------------------------
bool recibeBluetoothData()
{
  if (Serial1.available() > 0) 
  {
    delay(2);
    cmd[0] =  Serial1.read();  
    if(cmd[0] == STX)  
    {
      int i=1;      
      while(Serial1.available())  
      {
        delay(1);
        cmd[i] = Serial1.read();
        if(cmd[i]>127 || i>7)                 return false;     // Communication error
        if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
        i++;
      }
      
      //Se recibieron datos del Joystic
      if(i==7)          // Joystic: 6 Bytes  ex: < STX "200" "180" ETX >
      {
        int joyX = (cmd[1]-48)*100 + (cmd[2]-48)*10 + (cmd[3]-48);       // obtain the Int from the ASCII representation
        int joyY = (cmd[4]-48)*100 + (cmd[5]-48)*10 + (cmd[6]-48);
        joyX = joyX - 200;                                                  // Offset to avoid
        joyY = joyY - 200;                                                  // transmitting negative numbers
        if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return false;      // commmunication error
        
        //Calcular la velocidad de los motores con punto flotante
        dir=joyX;  vel=joyY;
        dir=dir/100; vel=vel/100; //normalizar
        if(dir>0) //derecha
        {
             auxf1=Vmax*vel;
             VM1=(int)auxf1;
             auxf1=Vmax*vel*(1-dir);
             VM2=(int)auxf1;
        }
        else    //izq
        {
             auxf1=Vmax*vel;
             VM2=(int)auxf1;
             auxf1=Vmax*vel*(dir+1);
             VM1=(int)auxf1;
        }
/*        
        //Calcular la velocidad de los motores con enteros        Evaluar cual es la mejor opción
        if(joyX>0) //derecha
        {
             VM1=40*joyY;
             VM2=40*((joyY*(100-joyX))/100);
        }
        else    //izq
        {
             VM1=40*((joyY*(joyX+100))/100);
             VM2=40*joyY;
        }
*/
        //Actualizar la velocidad de los motores
        //if(Enable==1)  
       if (variable_estado==1)
        EnviaVelocidad();  
        
        #ifdef DEBUG
          //Serial.print("Joystick position:  ");
          //Serial.print(joyX);  
          //Serial.print(", ");  
          //Serial.println(joyY); 
          Serial.print("Velocidad de motores VM1,VM2:  ");
          Serial.print(VM1);  
          Serial.print(", ");  
          Serial.println(VM2); 
        #endif
      }
      
      //Se recibieron datos de botones
      else if(i==2)              // Boton: 3 Bytes  ex: < STX "C" ETX >
      {      
        //getButtonState(cmd[1]);
        switch (cmd[1]) 
        {
          // -----------------  BUTTON #1  -----------------------
          case 'A':
           
            variable_estado=1;
            
            buttonStatus |= B000001;        // ON
            Enable=1;     
            #ifdef DEBUG
              Serial.println("\n** Button_1: ON **");
              displayStatus = "LED <ON>";
              Serial.println(displayStatus);
            #endif
            //digitalWrite(ledPin, HIGH);
            break;
          case 'B':
            variable_estado=0;
            buttonStatus &= B111110;        // OFF
            Enable=0;  
            #ifdef DEBUG
              Serial.println("\n** Button_1: OFF **");
              displayStatus = "LED <OFF>";
              Serial.println(displayStatus);
            #endif  
            //digitalWrite(ledPin, LOW);
            break;
          // -----------------  BUTTON #2  -----------------------
          case 'C':
            buttonStatus |= B000010;        // ON
            // your code...      
            #ifdef DEBUG
              Serial.println("\n** Button_2: ON **");
              displayStatus = "Button2 <ON>";
              Serial.println(displayStatus);
            #endif  
            break;
          case 'D':
            buttonStatus &= B111101;        // OFF
            // your code...      
            #ifdef DEBUG
              Serial.println("\n** Button_2: OFF **");
              displayStatus = "Button2 <OFF>";
              Serial.println(displayStatus);
            #endif  
            break;
          // -----------------  BUTTON #3  -----------------------
          case 'E':
            buttonStatus |= B000100;        // ON
            // your code...      
            #ifdef DEBUG
              Serial.println("\n** Button_3: ON **");
              displayStatus = "Motor #1 enabled"; // Demo text message
              Serial.println(displayStatus);
            #endif
            break;
          case 'F':
            buttonStatus &= B111011;      // OFF
            // your code...
            #ifdef DEBUG      
              Serial.println("\n** Button_3: OFF **");
              displayStatus = "Motor #1 stopped";
              Serial.println(displayStatus);
            #endif
            break;
          // -----------------  BUTTON #4  -----------------------
          case 'G':
            buttonStatus |= B001000;       // ON
            // your code...            
            sendInterval = FAST;
            
            #ifdef DEBUG
              Serial.println("\n** Button_4: ON **");
              displayStatus = "Datafield update <FAST>";
              Serial.println(displayStatus);
            #endif 
            break;
          case 'H':
            buttonStatus &= B110111;    // OFF
            // your code...
            sendInterval = SLOW;
            
            #ifdef DEBUG      
              Serial.println("\n** Button_4: OFF **");
              displayStatus = "Datafield update <SLOW>";
              Serial.println(displayStatus);
            #endif
            break;
          // -----------------  BUTTON #5  -----------------------
          case 'I':           // configured as momentary button
            //      buttonStatus |= B010000;        // ON
            // your code...      
            #ifdef DEBUG      
              Serial.println("\n** Button_5: ++ pushed ++ **");
              displayStatus = "Button5: <pushed>";
            #endif  
            break;
//        case 'J':
//          buttonStatus &= B101111;        // OFF
//          // your code...      
//          break;
          // -----------------  BUTTON #6  -----------------------
          case 'K':
            buttonStatus |= B100000;        // ON
            // your code...      
            #ifdef DEBUG      
              Serial.println("\n** Button_6: ON **");
              displayStatus = "Button6 <ON>"; // Demo text message
            #endif  
            break;
          case 'L':
            buttonStatus &= B011111;        // OFF
            // your code...  
            #ifdef DEBUG         
              Serial.println("\n** Button_6: OFF **");
              displayStatus = "Button6 <OFF>";
            #endif  
            break;
        }
      }
      return true;
    }
  }
  return false;  
}

//---------------------------- ENVIA DATOS AL BLUETOOTH -----------------------------
void sendBlueToothData()  
{
  String bStatus = "";
  
  static long previousMillis = 0;                             
  long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
    previousMillis = currentMillis; 

// Data frame transmitted back from Arduino to Android device:
// < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
// < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    Serial1.print((char)STX);                                             // Start of Transmission
    for(int i=0; i<6; i++)  
    {
      if(buttonStatus & (B100000 >>i))      bStatus += "1";
      else                                  bStatus += "0";
    }
    Serial1.print(bStatus);                  Serial1.print((char)0x1);   // buttons status feedback
    Serial1.print("0.1122");                 Serial1.print((char)0x4);   // datafield #1
    Serial1.print("1.5777");                 Serial1.print((char)0x5);   // datafield #2
    Serial1.print(displayStatus);                                         // datafield #3
    Serial1.print((char)ETX);                                             // End of Transmission
  }  
}

//---------------------------- ENVIA VELOCIDAD A DRIVER 2 --------------------------
//Formato SPI: 'V',VM1H,VM1L,VM2H,VM2L
void EnviaVelocidad(void)
{
  #ifdef DEBUG
      Serial.println(VM1,DEC);
      Serial.println(VM2,DEC);
      Serial.println(" ");
  #endif
      
  digitalWrite(ssdriver2, LOW);
  SPIresult = SPI.transfer('V');
  digitalWrite(ssdriver2, HIGH);
  #ifdef DEBUG
      Serial.println(SPIresult); 
  #endif
      
  digitalWrite(ssdriver2, LOW);
  SPIresult = SPI.transfer(VM1>>8);
  digitalWrite(ssdriver2, HIGH);
  #ifdef DEBUG
      Serial.println(SPIresult); //    Serial.print(SPIresult,HEX); 
  #endif
      
  digitalWrite(ssdriver2, LOW);
  SPIresult = SPI.transfer(VM1&0x00FF);
  digitalWrite(ssdriver2, HIGH);
  #ifdef DEBUG
      Serial.println(SPIresult); 
  #endif
      
  digitalWrite(ssdriver2, LOW);
  SPIresult = SPI.transfer(VM2>>8);
  digitalWrite(ssdriver2, HIGH);
  #ifdef DEBUG
      Serial.println(SPIresult); 
  #endif
      
  digitalWrite(ssdriver2, LOW);
  SPIresult = SPI.transfer(VM2&0x00FF);
  digitalWrite(ssdriver2, HIGH);
  #ifdef DEBUG    
      Serial.println(SPIresult);       
      Serial.println(" ");
      Serial.println("Fin de comando de velocidad"); 
  #endif
}


void distancia_medida()
{
   
   digitalWrite(SONAR_TRIGGER_PIN_CENTRO, HIGH);
   delayMicroseconds(10);
   digitalWrite(SONAR_TRIGGER_PIN_CENTRO, LOW);
   delayMicroseconds(1);
   long_pulso = pulseIn(SONAR_ECHO_PIN_CENTRO, HIGH);
   distancia_centro=(long_pulso / 58);
   
   
   digitalWrite(SONAR_TRIGGER_PIN_IZQ, HIGH);
   delayMicroseconds(10);
   digitalWrite(SONAR_TRIGGER_PIN_IZQ, LOW);
   delayMicroseconds(1);
   long_pulso = pulseIn(SONAR_ECHO_PIN_IZQUIERDA, HIGH);
   distancia_izq=(long_pulso / 58);
   
   
   digitalWrite(SONAR_TRIGGER_PIN_DERECHA, HIGH);
   delayMicroseconds(10);
   digitalWrite(SONAR_TRIGGER_PIN_DERECHA, LOW);
   delayMicroseconds(1);
   long_pulso = pulseIn(SONAR_ECHO_PIN_DERECHA, HIGH);
   distancia_der=(long_pulso / 58);
 
 
   digitalWrite(SONAR_TRIGGER_PIN_TRASERO, HIGH);
   delayMicroseconds(10);
   digitalWrite(SONAR_TRIGGER_PIN_TRASERO, LOW);
   delayMicroseconds(1);
   long_pulso = pulseIn(SONAR_ECHO_PIN_TRASERO, HIGH);
   distancia_trasera=(long_pulso / 58);
   
}


