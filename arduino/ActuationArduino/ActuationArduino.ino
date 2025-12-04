

// PWM output signal to coils is much smoother at 0x02. although you would reduce the rotating field resolution if you swithc at a lower PWM cycle. not sure whats better.
#include <AD9850.h>
#include <Wire.h>
#include "SerialTransfer.h"


SerialTransfer myTransfer;


float action[10]; //an array to store incoming data from python

#define PI 3.1415926535897932384626433832795

//actions
float Bx;
float By;
float Bz;
float alpha;
float gamma;
float rolling_frequency;
float psi;
float gradient_status;
float equal_field_status;
float acoustic_frequency;
float nullvalue;




int phase = 0; 



//other field vals
float Bx_roll;
float By_roll;
float Bz_roll;

float Bx_uniform;
float By_uniform;
float Bz_uniform;

float BxPer;
float ByPer;
float BzPer;
float c; 
float magnitude;


//other constants
float tim;
float t;
float omega;

float Bx_final;
float By_final;
float Bz_final;


bool acoustic_on = false;




//Coil 1 : +Y Brown
const int Coil1_PWMR = 2;
const int Coil1_PWML = 3;
const int Coil1_ENR = 26; //26
const int Coil1_ENL = 27; //27  
const int Coil1_DIR_R = 35; //26
const int Coil1_DIR_L = 37; //27   333

//Coil 2 : +X Purple
const int Coil2_PWMR = 44; 
const int Coil2_PWML = 45;
const int Coil2_ENR = 24; //24
const int Coil2_ENL = 25; //25
const int Coil2_DIR_R = 39; //26
const int Coil2_DIR_L = 41; //27   333

//Coil 3 : -Y Green
const int Coil3_PWMR = 6;
const int Coil3_PWML = 7;
const int Coil3_ENR = 22;
const int Coil3_ENL = 23;
const int Coil3_DIR_R = 43; //26
const int Coil3_DIR_L = 47; //27   333

//Coil 4: -X Blue
const int Coil4_PWMR = 10;
const int Coil4_PWML = 9;
const int Coil4_ENR = 32;
const int Coil4_ENL = 33;
const int Coil4_DIR_R = 49; //26
const int Coil4_DIR_L = 52; //27   333

//Coil 5 : +Z  Yellow
const int Coil5_PWMR = 12;
const int Coil5_PWML = 11;
const int Coil5_ENR = 30;
const int Coil5_ENL = 31;
const int Coil5_DIR_R = 48; //26
const int Coil5_DIR_L = 50; //27   333

//Coil 6 : -Z Orange
const int Coil6_PWMR = 8;
const int Coil6_PWML = 46; // 
const int Coil6_ENR = 28;
const int Coil6_ENL = 29;
const int Coil6_DIR_R = 51; //26
const int Coil6_DIR_L = 53; //27   333


//AD9850 Acoustic Module
const int W_CLK_PIN = 34;
const int FQ_UD_PIN = 36;
const int DATA_PIN = 38;
const int RESET_PIN = 40;


  
void setup()
{


  cli();
  TCCR1B = (TCCR1B & 0b11111000) | 0x02; //31.37255 [kHz] pin 12,11
  TCCR2B = (TCCR2B & 0b11111000) | 0x02; //31.37255 [kHz] pin 10,9
  TCCR3B = (TCCR3B & 0b11111000) | 0x02; //31.37255 [kHz] pin 5,3,2
  TCCR4B = (TCCR4B & 0b11111000) | 0x02; //31.37255 [kHz] pin 8,7,6
  TCCR5B = (TCCR5B & 0b11111000) | 0x02; //31.37255 [kHz] pin 44,45,46   
  sei();
  
  Serial.begin(115200);
  myTransfer.begin(Serial);

    //start acoustic module
  DDS.begin(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);
  DDS.calibrate(124999500);




   //Coil1 Ouptut
  pinMode(Coil1_PWMR, OUTPUT);  
  pinMode(Coil1_PWML, OUTPUT);
  pinMode(Coil1_ENR, OUTPUT);  
  pinMode(Coil1_ENL, OUTPUT); 
  pinMode(Coil1_DIR_R, OUTPUT);  
  pinMode(Coil1_DIR_L, OUTPUT); 
  


  //Coil2 Output
  pinMode(Coil2_PWMR, OUTPUT);  
  pinMode(Coil2_PWML, OUTPUT);
  pinMode(Coil2_ENR, OUTPUT);  
  pinMode(Coil2_ENL, OUTPUT); 
  pinMode(Coil2_DIR_R, OUTPUT);  
  pinMode(Coil2_DIR_L, OUTPUT); 
  


  //Coil3 Output
  pinMode(Coil3_PWMR, OUTPUT);  
  pinMode(Coil3_PWML, OUTPUT);
  pinMode(Coil3_ENR, OUTPUT);  
  pinMode(Coil3_ENL, OUTPUT); 
  pinMode(Coil3_DIR_R, OUTPUT);  
  pinMode(Coil3_DIR_L, OUTPUT); 


  //Coil4 Output
  pinMode(Coil4_PWMR, OUTPUT);  
  pinMode(Coil4_PWML, OUTPUT);
  pinMode(Coil4_ENR, OUTPUT);  
  pinMode(Coil4_ENL, OUTPUT); 
  pinMode(Coil4_DIR_R, OUTPUT);  
  pinMode(Coil4_DIR_L, OUTPUT); 


  //Coil5 Output
  pinMode(Coil5_PWMR, OUTPUT);  
  pinMode(Coil5_PWML, OUTPUT);
  pinMode(Coil5_ENR, OUTPUT);  
  pinMode(Coil5_ENL, OUTPUT);
  pinMode(Coil5_DIR_R, OUTPUT);  
  pinMode(Coil5_DIR_L, OUTPUT); 
 

  //Coil6 Output
  pinMode(Coil6_PWMR, OUTPUT);  
  pinMode(Coil6_PWML, OUTPUT);
  pinMode(Coil6_ENR, OUTPUT);  
  pinMode(Coil6_ENL, OUTPUT);
  pinMode(Coil6_DIR_R, OUTPUT);  
  pinMode(Coil6_DIR_L, OUTPUT); 
  


  digitalWrite(Coil1_ENR, HIGH);
  digitalWrite(Coil1_ENL, HIGH);
  digitalWrite(Coil1_DIR_R, LOW);
  digitalWrite(Coil1_DIR_L, LOW);

  digitalWrite(Coil2_ENR, HIGH);
  digitalWrite(Coil2_ENL, HIGH);
  digitalWrite(Coil2_DIR_R, LOW);
  digitalWrite(Coil2_DIR_L, LOW);

  digitalWrite(Coil3_ENR, HIGH);
  digitalWrite(Coil3_ENL, HIGH);
  digitalWrite(Coil3_DIR_R, LOW);
  digitalWrite(Coil3_DIR_L, LOW);

  digitalWrite(Coil4_ENR, HIGH);
  digitalWrite(Coil4_ENL, HIGH);
  digitalWrite(Coil4_DIR_R, LOW);
  digitalWrite(Coil4_DIR_L, LOW);

  digitalWrite(Coil5_ENR, HIGH);
  digitalWrite(Coil5_ENL, HIGH);
  digitalWrite(Coil5_DIR_R, LOW);
  digitalWrite(Coil5_DIR_L, LOW);

  digitalWrite(Coil6_ENR, HIGH);
  digitalWrite(Coil6_ENL, HIGH);
  digitalWrite(Coil6_DIR_R, LOW);
  digitalWrite(Coil6_DIR_L, LOW);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set1(float DC1){
  DC1 = constrain(DC1, -1.0, 1.0);
  if (DC1 > 0){
    digitalWrite(Coil1_DIR_R, HIGH);
    digitalWrite(Coil1_DIR_L, LOW);
    analogWrite(Coil1_PWMR,abs(DC1)*255);
    analogWrite(Coil1_PWML,0);
    
  }
  else if (DC1 < 0){
    digitalWrite(Coil1_DIR_R, LOW);
    digitalWrite(Coil1_DIR_L, HIGH);
    analogWrite(Coil1_PWMR,0);
    analogWrite(Coil1_PWML,abs(DC1)*255);
    
  }
  else {
    digitalWrite(Coil1_DIR_R, LOW);
    digitalWrite(Coil1_DIR_L, LOW);
    analogWrite(Coil1_PWMR,0);
    analogWrite(Coil1_PWML,0); 
    
    
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set2(float DC2){
  DC2 = constrain(DC2, -1.0, 1.0);
  if (DC2 > 0){
    digitalWrite(Coil2_DIR_R, HIGH);
    digitalWrite(Coil2_DIR_L, LOW);
    analogWrite(Coil2_PWMR,abs(DC2)*255);
    analogWrite(Coil2_PWML,0);
    
  }
  else if (DC2 < 0){
    digitalWrite(Coil2_DIR_R, LOW);
    digitalWrite(Coil2_DIR_L, HIGH);
    analogWrite(Coil2_PWMR,0);
    analogWrite(Coil2_PWML,abs(DC2)*255);
    
  }
  else {
    digitalWrite(Coil2_DIR_R, LOW);
    digitalWrite(Coil2_DIR_L, LOW);
    analogWrite(Coil2_PWMR,0);
    analogWrite(Coil2_PWML,0);
    
  }
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set3(float DC3){
  DC3 = constrain(DC3, -1.0, 1.0);
  if (DC3 > 0){
    digitalWrite(Coil3_DIR_R, HIGH);
    digitalWrite(Coil3_DIR_L, LOW);
    analogWrite(Coil3_PWMR,abs(DC3)*255);
    analogWrite(Coil3_PWML,0);
    
  }
  else if (DC3 < 0){
    digitalWrite(Coil3_DIR_R, LOW);
    digitalWrite(Coil3_DIR_L, HIGH);
    analogWrite(Coil3_PWMR,0);
    analogWrite(Coil3_PWML,abs(DC3)*255);
    
  }
  else {
    digitalWrite(Coil3_DIR_R, LOW);
    digitalWrite(Coil3_DIR_L, LOW);
    analogWrite(Coil3_PWMR,0);
    analogWrite(Coil3_PWML,0);
    
    
  }

  

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set4(float DC4){
  DC4 = constrain(DC4, -1.0, 1.0);
  if (DC4 > 0){
    digitalWrite(Coil4_DIR_R, HIGH);
    digitalWrite(Coil4_DIR_L, LOW);
    analogWrite(Coil4_PWMR,abs(DC4)*255);
    analogWrite(Coil4_PWML,0);

  }
  else if (DC4 < 0){
    digitalWrite(Coil4_DIR_R, LOW);
    digitalWrite(Coil4_DIR_L, HIGH);
    analogWrite(Coil4_PWMR,0);
    analogWrite(Coil4_PWML,abs(DC4)*255);
    
  }
  else {
    digitalWrite(Coil4_DIR_R, LOW);
    digitalWrite(Coil4_DIR_L, LOW);
    analogWrite(Coil4_PWMR,0);
    analogWrite(Coil4_PWML,0);
    
   
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set5(float DC5){
  DC5 = constrain(DC5, -1.0, 1.0);
  if (DC5 > 0){
    digitalWrite(Coil5_DIR_R, HIGH);
    digitalWrite(Coil5_DIR_L, LOW);
    analogWrite(Coil5_PWMR,abs(DC5)*255);
    analogWrite(Coil5_PWML,0);
    
  }
  else if (DC5 < 0){
    digitalWrite(Coil5_DIR_R, LOW);
    digitalWrite(Coil5_DIR_L, HIGH);
    analogWrite(Coil5_PWMR,0);
    analogWrite(Coil5_PWML,abs(DC5)*255);
    
  }
  else {
    digitalWrite(Coil5_DIR_R, LOW);
    digitalWrite(Coil5_DIR_L, LOW);
    analogWrite(Coil5_PWMR,0);
    analogWrite(Coil5_PWML,0);


    
  }


 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set6(float DC6){
  DC6 = constrain(DC6, -1.0, 1.0);
  if (DC6 > 0){
    digitalWrite(Coil6_DIR_R, HIGH);
    digitalWrite(Coil6_DIR_L, LOW);
    analogWrite(Coil6_PWMR,abs(DC6)*255);
    analogWrite(Coil6_PWML,0);
    
  }
  else if (DC6 < 0){
    digitalWrite(Coil6_DIR_R, LOW);
    digitalWrite(Coil6_DIR_L, HIGH);
    analogWrite(Coil6_PWMR,0);
    analogWrite(Coil6_PWML,abs(DC6)*255);
    
  }
  else {
    digitalWrite(Coil6_DIR_R, LOW);
    digitalWrite(Coil6_DIR_L, LOW);
    analogWrite(Coil6_PWMR,0);
    analogWrite(Coil6_PWML,0);
    

  }
 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
    //RECEIVE FROM PYTHON
 
    if (myTransfer.available() >= sizeof(action)) {
          uint16_t message = 0;
          
      
          // Now read into the action array
          message = myTransfer.rxObj(action, message);

          //for (int i = 0; i < 10; i++) {
           //   Serial.print(" ");
           //   Serial.print("action[");
           //   Serial.print(i);
           //   Serial.print("] = ");
           //   Serial.println(action[i], 6);  // 6 decimal places for clarity
           //  }

           //  Serial.print("Bytes read: ");
           //  Serial.println(message);
    }

   // assign incoming actions to variables
   Bx_uniform = action[0];
   By_uniform = action[1];
   Bz_uniform = action[2];   
   alpha = action[3];
   gamma = action[4];
   rolling_frequency = action[5]; 
   psi = action[6]; 
   gradient_status = action[7];
   equal_field_status = action[8];
   acoustic_frequency = action[9];


   
   
    // handle acoustic waveform output
   if (acoustic_frequency > 0) {
        if (!acoustic_on) DDS.setfreq(acoustic_frequency,phase);
              acoustic_on = true;
    } else {
        if (acoustic_on) DDS.down();
            acoustic_on = false;
    }
   
   // convert to an angular velocity 
   omega = 2*PI*rolling_frequency;
   

    //define a time variable to continously sample from
   t = micros() / 1e6;

   //field generation logic
   if (omega == 0){
       Bx_roll = 0;
       By_roll = 0;
       Bz_roll = 0;
      }
   else {
      //working rotating field equations 7/1/25
      Bx_roll = - (cos(alpha) * cos(gamma)  * cos(omega*t))     +     (sin(alpha) * sin(omega*t)) ; 
      By_roll = - (sin(alpha) * cos(gamma)  * cos(omega*t))     -     (cos(alpha) * sin(omega*t)); 
      Bz_roll = sin(gamma) * cos(omega*t);

       // condition for perpendicular field (psi cannot be 90)
       // condition for perpendicular field (psi cannot be 90)
      if (psi < PI/2){
          c = 1/tan(psi);
          BxPer = c* cos(alpha) * sin(gamma);
          ByPer = tan(alpha) * BxPer;
          BzPer = BxPer * (1/cos(alpha)) * (1/tan(gamma));  
          }
      else{
          c = 0;
          BxPer = 0;
          ByPer = 0;
          BzPer = 0;
      }
       
       // superimpose the rolling field with the perpendicular field
      Bx_roll = (Bx_roll + BxPer) / (1+c);
      By_roll = (By_roll + ByPer) / (1+c);
      Bz_roll = (Bz_roll + BzPer) / (1+c);
      
      }
   //need to add unform field with rotating field and normalize
   //cc = sqrt(Bx_roll*Bx_roll + By_roll*By_roll + Bz_roll*Bz_roll)
   Bx = (Bx_uniform + Bx_roll); /// (1+cc);
   By = (By_uniform + By_roll); /// (1+cc);
   Bz = (Bz_uniform + Bz_roll); //// (1+cc); 
    

   // condition to prevent divide by zero error when total Bx, By, Bz are off aka zeroed
   if (Bx == 0 and By == 0 and Bz ==0){
        Bx_final = 0;
        By_final = 0;
        Bz_final = 0;
   }
   // otherwise I need to normalize the superpoistion of the rotating field with the uniform field
   else{
       magnitude = max(sqrt(Bx_uniform*Bx_uniform + By_uniform*By_uniform + Bz_uniform*Bz_uniform), 
                       sqrt(Bx_roll*Bx_roll + By_roll*By_roll + Bz_roll*Bz_roll));


       Bx_final = magnitude * (Bx / sqrt(Bx*Bx + By*By + Bz*Bz));
       By_final = magnitude * (By / sqrt(Bx*Bx + By*By + Bz*Bz));
       Bz_final = magnitude * (Bz / sqrt(Bx*Bx + By*By + Bz*Bz));


       //equal field status for helmholtz coil design
       if (equal_field_status == 1){
           Bx_final = Bx_final;
           By_final = By_final * .6;
           Bz_final = Bz_final * .3;
       }
          
       
   }
   

  // if gradient status = 1: output the the corresponding gradient field
   if (gradient_status != 0){
      //y gradient
      if (By_final > 0){
        set1(By_final);
        
      }
      else if (By_final < 0){
        set3(By_final);
      }
      else{ //if By==0
        set1(0);
        set3(0);
      }

      //x gradient
      if (Bx_final > 0){
        set2(Bx_final);
      }
      else if (Bx_final < 0){
        set4(Bx_final);
      }
      else{ //if Bx==0
        set2(0);
        set4(0);
      }

      //z gradient
      if (Bz_final > 0){
        set5(Bz_final);
      }
      else if (Bz_final < 0){
        set6(Bz_final);
      }
      else{ //if Bz==0
        set5(0);
        set6(0);
      }
   }

   // if gradient status = 0: output the correspending uniform field
   else {
      set1(By_final);
      set2(Bx_final);
      set3(-By_final);
      set4(-Bx_final);
      set5(Bz_final);
      set6(-Bz_final);
    
   }

      

  

    }
  
