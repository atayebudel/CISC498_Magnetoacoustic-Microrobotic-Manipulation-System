#include <ACS712.h>
#include "SerialTransfer.h"


SerialTransfer myTransfer;

const float vcc = 5.0;
const int adcMax = 1023;
const int sens = 185;  // 5A version

//Coil 1 : +Y Brown 
const int Coil1_DIR_R = 28; 
const int Coil1_DIR_L = 29;

//Coil 2 : +X Purple
const int Coil2_DIR_R = 30; 
const int Coil2_DIR_L = 31; 

//Coil 3 : -Y Green
const int Coil3_DIR_R = 32; 
const int Coil3_DIR_L = 33; 

//Coil 4: -X Blue
const int Coil4_DIR_R = 46; 
const int Coil4_DIR_L = 48; 

//Coil 5 : +Z  Yellow
const int Coil5_DIR_R = 44; 
const int Coil5_DIR_L = 42; 

//Coil 6 : -Z Orange
const int Coil6_DIR_R = 38; 
const int Coil6_DIR_L = 40; 


ACS712 sensor1(A0, vcc, adcMax, sens); //+Y
ACS712 sensor2(A1, vcc, adcMax, sens); //+X
ACS712 sensor3(A2, vcc, adcMax, sens); //-Y
ACS712 sensor4(A3, vcc, adcMax, sens); //-X
ACS712 sensor5(A4, vcc, adcMax, sens); //+Z
ACS712 sensor6(A5, vcc, adcMax, sens); //-Z


//Store data from arduino to send to python
struct current_data {
  float Coil1_current;  //+Y
  float Coil2_current;  //+X
  float Coil3_current;  //-Y
  float Coil4_current;  //-X
  float Coil5_current;  //+Z
  float Coil6_current;  //-Z
};

current_data Current_Data; //create instance
  
void setup()
{  

  
  Serial.begin(115200);
  myTransfer.begin(Serial);

  sensor1.autoMidPoint();
  sensor2.autoMidPoint();
  sensor3.autoMidPoint();
  sensor4.autoMidPoint();
  sensor5.autoMidPoint();
  sensor6.autoMidPoint();

   //Coil1 INPUT
  pinMode(Coil1_DIR_R, INPUT);  
  pinMode(Coil1_DIR_L, INPUT); 
 
  //Coil2 INPUT
  pinMode(Coil2_DIR_R, INPUT);  
  pinMode(Coil2_DIR_L, INPUT); 
  
  //Coil3 INPUT
  pinMode(Coil3_DIR_R, INPUT);  
  pinMode(Coil3_DIR_L, INPUT); 

  //Coil4 INPUT
  pinMode(Coil4_DIR_R, INPUT);  
  pinMode(Coil4_DIR_L, INPUT); 

  //Coil5 Input
  pinMode(Coil5_DIR_R, INPUT);  
  pinMode(Coil5_DIR_L, INPUT); 
 
  //Coil6 INPUT
  pinMode(Coil6_DIR_R, INPUT);  
  pinMode(Coil6_DIR_L, INPUT); 
}


float smoothRead(ACS712 &sensor, int samples = 20) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += sensor.mA_DC();
    }
    return (sum / samples) / 1000.0;  // convert to A
}


void loop()                  
{  
        
        
        
        // Coil 1
        if (digitalRead(Coil1_DIR_R) == HIGH) {
            Current_Data.Coil1_current = smoothRead(sensor1); // positive current
        } 
        else if (digitalRead(Coil1_DIR_L) == HIGH) {
            Current_Data.Coil1_current = -smoothRead(sensor1); // negative current
        } 
        else {
            Current_Data.Coil1_current = smoothRead(sensor1); // no drive
        }



        // Coil 2
        if (digitalRead(Coil2_DIR_R) == HIGH) {
            Current_Data.Coil2_current = smoothRead(sensor2); // positive current
        } 
        else if (digitalRead(Coil2_DIR_L) == HIGH) {
            Current_Data.Coil2_current = -smoothRead(sensor2); // negative current
        } 
        else {
            Current_Data.Coil2_current = smoothRead(sensor2); // no drive
        }

        // Coil 3
        if (digitalRead(Coil3_DIR_R) == HIGH) {
            Current_Data.Coil3_current = smoothRead(sensor3); // positive current
        } 
        else if (digitalRead(Coil3_DIR_L) == HIGH) {
            Current_Data.Coil3_current = -smoothRead(sensor3); // negative current
        } 
        else {
            Current_Data.Coil3_current = smoothRead(sensor3); // no drive
        }


        // Coil 4
        if (digitalRead(Coil4_DIR_R) == HIGH) {
            Current_Data.Coil4_current = smoothRead(sensor4); // positive current
        } 
        else if (digitalRead(Coil4_DIR_L) == HIGH) {
            Current_Data.Coil4_current = -smoothRead(sensor4); // negative current
        } 
        else {
            Current_Data.Coil4_current = smoothRead(sensor4); // no drive
        }


        // Coil 5
        if (digitalRead(Coil5_DIR_R) == HIGH) {
            Current_Data.Coil5_current = smoothRead(sensor5); // positive current
        } 
        else if (digitalRead(Coil5_DIR_L) == HIGH) {
            Current_Data.Coil5_current = -smoothRead(sensor5); // negative current
        } 
        else {
            Current_Data.Coil5_current = smoothRead(sensor5); // no drive
        }

        // Coil 6
        if (digitalRead(Coil6_DIR_R) == HIGH) {
            Current_Data.Coil6_current = smoothRead(sensor6); // positive current
        } 
        else if (digitalRead(Coil6_DIR_L) == HIGH) {
            Current_Data.Coil6_current = -smoothRead(sensor6); // negative current
        } 
        else {
            Current_Data.Coil6_current = smoothRead(sensor6); // no drive
        }
        


        //Serial.print(Current_Data.Coil1_current); Serial.print(" ");
        //Serial.print(Current_Data.Coil2_current); Serial.print(" ");
        //Serial.print(Current_Data.Coil3_current); Serial.print(" ");
        //Serial.print(Current_Data.Coil4_current); Serial.print(" ");
        //Serial.print(Current_Data.Coil5_current); Serial.print(" ");
        //Serial.println(Current_Data.Coil6_current);
               
          
         //SEND TO PYTHON
    
         uint16_t sendSize = 0;
         sendSize = myTransfer.txObj(Current_Data, sendSize);
         myTransfer.sendData(sendSize);  

         


    
}
  
