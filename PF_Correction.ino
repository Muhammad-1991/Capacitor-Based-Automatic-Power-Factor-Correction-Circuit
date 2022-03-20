/*
  Capacitor Based Automatic Power Factor Correction 

  Caculates the time difference b/w Volatage and Current and coresponding 
  phase difference and power factor. Adds or removes capacitors in the circuit
  to make power factor as close to unity as possible. 

  The circuit:
  - digital 0: Serial Comm
  - digital 1: Serial Comm
  - digital 2: To Potential Transformer Output(as interrupt)
  - digital 3: To Current Transformer Output(as interrupt)
  - digital 11: To Relay for Including/excluding Capacitor
  - digital 12: To Relay for Including/excluding Capacitor
  - digital 13: To Relay for Including/excluding Capacitor

  created 21 Mar 2022
  by Muhammad Irfan

  This example code is in the public domain.

  For details, please visit
  https://github.com/Muhammad-1991/Capacitor-Based-Automatic-Power-Factor-Correction-Circuit
*/

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define PT_D       2                        //Interrupt Pin for PT Signal
#define CT_D       3                        //Interrupt Pin for CT Signal

#define CAP_BANK1 11                        //Pin for Including/Excluding Capacitor 1 
#define CAP_BANK2 12                        //Pin for Including/Excluding Capacitor 2
#define CAP_BANK3 13                        //Pin for Including/Excluding Capacitor 3

volatile unsigned long  pulseInTime_PT = 0;             //Variable to store time in uS of PT Signal
volatile unsigned long  pulseInTime_CT = 0;             //Variable to store time in uS of CT Signal
volatile bool           waveRecorded_PT = false;        //Variable to determine whether PT signal has been recorded or not
volatile bool           waveRecorded_CT = false;        //Variable to determine whether CT signal has been recorded or not
float                   timeDifferenceAv = 0;           //Variable for storing Average time difference b/w PT and CT Signals

int         divider = 0;              //Variable is being used to take mean value of time difference b/w PT and CT signals
const float freq = 50.0;              //System/Grid Frequency in Hz (either 50 or 60).
float       thetaDeg = 0;             //Phase between Voltage and Current in Degrees
float       thetaRad = 0;             //Phase between Voltage and Current in Radians
float       runningPF = 1;

int         capIn = 0;                //Variable to store current number of Capacitors included in the system 

int     CAP_IN[8] = {};               //Array to store 8 feasible number of capacitors-combinations 
float   POWER_FACTOR[8] = {};         //Array to store 8 respective power-factors
int     index = 0;                    //For Indexing 
float   optimalPF = 0;                //Most optimal POWER FACTOR resulted for the load 
bool    PF_Calculated = false;        //Variable to find whether Optimal PF is being calculated for the load 

U8G2_KS0108_128X64_F u8g2(U8G2_R0, 8, 9, 10, 5, 16, 19, 6, 7, /*enable=*/ 18, /*dc=*/ 17, /*cs0=*/ 14, /*cs1=*/ 15, /*cs2=*/ U8X8_PIN_NONE, /* reset=*/  U8X8_PIN_NONE);   // Set R/W to low!

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////ISR FOR RECORDING THE TIME, THE PT SIGNAL ARRIVED/////////////////////////////
void potentialTransformer()                       ///////////////////////////////////////////////////////////
{                                                 //Once the Signal-time for potential transformer is recorded
  pulseInTime_PT = micros();                      //The interrupt to PT_D pin is detached and this event is indicated  
  detachInterrupt(digitalPinToInterrupt(PT_D));   //by waveRecorded_PT variable as being TRUE
  waveRecorded_PT = true;                         //----------SAME FOR CURRENT TRANSFORMER-----------------//
}                                                 ///////////////////////////////////////////////////////////

///////////////////////////////ISR FOR RECORDING THE TIME, THE CT SIGNAL ARRIVED/////////////////////////////
void currentTransformer()
{
  pulseInTime_CT = micros();
  detachInterrupt(digitalPinToInterrupt(CT_D));
  waveRecorded_CT = true;
}

void optimalPowerFactor(int, float);        //Function for Finding Optimal Power Factor and Related Number of Capacitors
void pfCorrection();                        //Function for Changing Capacitance in the System Depending on Whether Current is leading or Lagging

void setup() {

  Serial.begin(115200);
  
  pinMode(PT_D, INPUT);                   //Define PT_D Pin as input
  pinMode(CT_D, INPUT);                   //Define CT_D Pin as input

  pinMode(CAP_BANK1, OUTPUT);             //
  pinMode(CAP_BANK2, OUTPUT);             //Define CAP_BANK1 as output 
  pinMode(CAP_BANK3, OUTPUT);             //

  digitalWrite(CAP_BANK1, LOW);           //Initially no capacitor is included in the system
  digitalWrite(CAP_BANK2, LOW);           //
  digitalWrite(CAP_BANK3, LOW);           //These Pins or connected to relays as in Circuit Diagram
  
  attachInterrupt(digitalPinToInterrupt(PT_D), potentialTransformer, RISING);       //Attach Interrupts to PT_D pin 
  attachInterrupt(digitalPinToInterrupt(CT_D), currentTransformer, RISING);         //Attach Interrupts to CT_D pin 

  pinMode(9, OUTPUT);
  digitalWrite(9, 0);  // default output in I2C mode for the SSD1306 test shield: set the i2c adr to 0
  u8g2.begin();
                     
}

void loop() {

  if(waveRecorded_PT == true && waveRecorded_CT == true )                       ////////////////////////////////////////////////////
  {                                                                             // Once time for both PT_D and CT_D is recorded, these   
    waveRecorded_PT = false;                                                    //statements will execute. Initially setting both waveRecorded_CT  
    waveRecorded_CT = false;                                                    //and waveRecorded_PT as false. Then measuring the time difference 
                                                                                //b/t PT_D and CT_D signals. The inner if statement ensures that both 
        long timeDifference = pulseInTime_PT - pulseInTime_CT;                  //PT_D and CT_D corresponds to the same wave. Time defference is  
                                                                                //averaged over 10 signals recorded accurately. 
        if( timeDifference > -5000 && timeDifference < 5000 )                   //
        {                                                                       //
          timeDifferenceAv = timeDifferenceAv + timeDifference;                 //
          divider++;                                                            //
        }                                                                       //
                                                                                //
    attachInterrupt(digitalPinToInterrupt(PT_D), potentialTransformer, RISING); //
    attachInterrupt(digitalPinToInterrupt(CT_D), currentTransformer, RISING);   //
  }                                                                             ////////////////////////////////////////////////////
  
  
  if(divider > 9 )                                                              //This statement will execute once 10 accurate time difference are recorded
  {
      
      timeDifferenceAv = timeDifferenceAv / divider;      //Take Mean Value of pulse durations
      
      timeDifferenceAv = timeDifferenceAv / 1000000.00;   //Convert microseconds into seconds

      Serial.println("//////////////////////////////");

      thetaDeg = timeDifferenceAv * freq * 360.0;      //Calculate angle in degrees
    
      Serial.print("Phase Difference in Deg: ");
      Serial.println(thetaDeg);
    
      thetaRad = thetaDeg * 0.0174533 ;               //Calculate angle in radians
    
      runningPF = cos(thetaRad);

      Serial.print("Power Factor Running: ");
      Serial.println(runningPF);
      Serial.print("capIn: ");
      Serial.println(capIn);
      Serial.print("PF_Calculated: ");
      Serial.println(PF_Calculated);
      Serial.println("//////////////////////////////");
      
      timeDifferenceAv = 0;
      divider = 0;  

      if( (runningPF < optimalPF * 0.975 || runningPF > optimalPF * 1.025) && PF_Calculated == true ) // This statment checks whether the system is running at Optimal PF
        PF_Calculated = false;                                                                        //If power factor does not match the optimal PF the variable PF_Calculated
                                                                                                      //is assigned a FALSE value, meaning that new Optimal PF needs be calculated        
                                                                                                      
      if(PF_Calculated == false)
      {
        detachInterrupt(digitalPinToInterrupt(PT_D));
        detachInterrupt(digitalPinToInterrupt(CT_D));
        optimalPowerFactor(capIn, runningPF);                                       
      }           
   }
}

//////////////////////////////////////////////////////////////////////////
///////POWER FACTOR //////

void optimalPowerFactor(int cap_In, float PF)
{
  
  CAP_IN[index] = cap_In;
  POWER_FACTOR[index++] = PF;
  
  if(index > 7)
  {
    index = 0;
    int INDEX = 0;
    PF_Calculated = true;
    
    for(int i = 0; i < 8; i++)
    {
      if( POWER_FACTOR[0] < POWER_FACTOR[i] )
      {
        POWER_FACTOR[0] = POWER_FACTOR[i];
        INDEX = i;
      }    
    }

    optimalPF = POWER_FACTOR[0];
    capIn = CAP_IN[INDEX];

    pfCorrection();
  
    Serial.println("______________________________");
    Serial.println(optimalPF);
    Serial.println(capIn);
    Serial.println("______________________________"); 

    /////////////////////////////////////////////////////////////////////////////////////////
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
    u8g2.setCursor(2,10);                // set cursor position 
    u8g2.print("Optimal Power Factor");
    u8g2.setCursor(50,25);
    u8g2.print(optimalPF);                       // write power factor to the internal memory
    u8g2.sendBuffer();
    /////////////////////////////////////////////////////////////////////////////////////////

    attachInterrupt(digitalPinToInterrupt(PT_D), potentialTransformer, RISING);
    attachInterrupt(digitalPinToInterrupt(CT_D), currentTransformer, RISING); 
    
  } else
        {
            if( thetaDeg < 0 )
               capIn++;
            else if(thetaDeg > 0)
               capIn--;
          
            if( capIn < 0 )
               capIn = 0;
            else if( capIn > 7)
               capIn = 7;

            /////////////////////////////////////////////////////////////////////////////////////////
            u8g2.clearBuffer();                   // clear the internal memory
            u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
            u8g2.setCursor(2,10);                // set cursor position
        
            u8g2.print("Running Power Factor");
            u8g2.setCursor(50,25);
            u8g2.print(runningPF);                       // write power factor to the internal memory
            u8g2.sendBuffer();
            /////////////////////////////////////////////////////////////////////////////////////////
     
            pfCorrection();
        }
}

void pfCorrection()
{ 
  switch(capIn)
  {
  case 0:
    digitalWrite(CAP_BANK1, LOW);
    digitalWrite(CAP_BANK2, LOW);
    digitalWrite(CAP_BANK3, LOW);
    break;
  case 1:
    digitalWrite(CAP_BANK1, HIGH);
    digitalWrite(CAP_BANK2, LOW);
    digitalWrite(CAP_BANK3, LOW); 
    break;
  case 2:
    digitalWrite(CAP_BANK1, LOW);
    digitalWrite(CAP_BANK2, HIGH);
    digitalWrite(CAP_BANK3, LOW); 
    break;
  case 3:
    digitalWrite(CAP_BANK1, HIGH);
    digitalWrite(CAP_BANK2, HIGH);
    digitalWrite(CAP_BANK3, LOW); 
    break;
  case 4:
    digitalWrite(CAP_BANK1, LOW);
    digitalWrite(CAP_BANK2, LOW);
    digitalWrite(CAP_BANK3, HIGH); 
    break;
  case 5:
    digitalWrite(CAP_BANK1, HIGH);
    digitalWrite(CAP_BANK2, LOW);
    digitalWrite(CAP_BANK3, HIGH); 
    break;
  case 6:
    digitalWrite(CAP_BANK1, LOW);
    digitalWrite(CAP_BANK2, HIGH);
    digitalWrite(CAP_BANK3, HIGH); 
    break;
  case 7:
    digitalWrite(CAP_BANK1, HIGH);
    digitalWrite(CAP_BANK2, HIGH);
    digitalWrite(CAP_BANK3, HIGH); 
    break;
  default:
    break;
  }    
    attachInterrupt(digitalPinToInterrupt(PT_D), potentialTransformer, RISING);
    attachInterrupt(digitalPinToInterrupt(CT_D), currentTransformer, RISING);   
}
