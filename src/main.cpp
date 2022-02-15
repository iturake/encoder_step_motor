/*eksikler: -Home pozisyonu
            -Stop pozisyonu
            -Pozitif negatif yon karari
*/
//----------------------------------------------------------------------------------------------//
#include <Arduino.h>
//enkoder pini
#define ANGLE_PIN 2 
#define INDEX_PIN 3 
//enkoder değer
double index; 
double angle;
//interrupt enkoder
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
//aci hesabı için değer
unsigned long duration;
//alinan deger
int Received_angle;
//motorun attığı tur sayısı 
int counter=0;

void indexInterrupt()
{
  if (digitalRead(INDEX_PIN) == HIGH) {
    // start measuring
    pulseInTimeBegin = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}

#include <AccelStepper.h>

//User-defined values
long receivedSteps = 0; //Number of steps
long receivedSpeed = 0; //Steps / second
long receivedAcceleration = 0; //Steps / second^2
char receivedCommand;
//-------------------------------------------------------------------------------
int directionMultiplier; // = 1: positive direction, = -1: negative direction
bool newData, runallowed = false; // booleans for new data from serial, and runallowed flag
AccelStepper stepper(2, 8, 9);// direction Digital 9 (CCW), pulses Digital 8 (CLK)

// Private Function
void RunTheMotor();
void RotateRelative();
void Encoder();
void PrintCommands();
void checkSerial();
int degreeCalculator(int counter, double encoderDegree);
void GoHome();




void setup() {

  Serial.begin(9600);
  pinMode(INDEX_PIN, INPUT);
  pinMode(ANGLE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INDEX_PIN), indexInterrupt,CHANGE);
 
//Setting for stepper motor
stepper.setMaxSpeed(1600); //SPEED = Steps / second
stepper.setAcceleration(3200); //ACCELERATION = Steps /(second)^2
 
stepper.disableOutputs(); //disable outputs
//-------------------------------------------
  cli();
  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */

  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 3999;
  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
  TCCR1B |= (1 << WGM12);
  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  /* Timer1 kesmesi aktif hale getirildi */

  sei();
  /* Timer1 kesmesinin çalışabilmesi için tüm kesmeler aktif hale getirildi */
    
}
ISR(TIMER1_COMPA_vect){
   duration = pulseIn(ANGLE_PIN, HIGH);
       angle = (duration/9.739499999999999/100)*361 -1;
    Serial.println(degreeCalculator(counter,angle));
}

void loop() {
  
  checkSerial(); //data alındı
  if (directionMultiplier==1) //Gelen yön pozitif(P) ise motor o an sahip olduğu açı değerinden daha az bir degere hareket edecek.
  {
    if (degreeCalculator(counter,angle)>Received_angle)
    { 
      receivedSteps = 500; //Number of steps
      receivedSpeed = 60000; //Steps / second
      receivedAcceleration = 800; //Steps / second^2   
      RotateRelative();
      RunTheMotor(); //function to handle the motor 
    }
  }
  else if (directionMultiplier==-1) //Gelen yön negatif(N) ise motor o an sahip olduğu açı değerinden daha fazla bir degere hareket edecek.
  {
    if (degreeCalculator(counter,angle)<Received_angle)
    { 
      receivedSteps = 500; //Number of steps
      receivedSpeed = 60000; //Steps / second
      receivedAcceleration = 800; //Steps / second^2   
      RotateRelative();
      RunTheMotor(); //function to handle the motor 
    }
  }
  Encoder(); 
  

 
}

void Encoder()
{
  
  //Measuring pulse width from encounter
  if (newPulseDurationAvailable) {
    newPulseDurationAvailable = false;
    unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;
    Serial.print("Counter: ");
    Serial.println(counter);
    Serial.print("Pulse : ");
    Serial.println(pulseDuration);

    if(pulseDuration>0){
      if (directionMultiplier==-1)
      {
        counter=counter+1;
      }
      else if (directionMultiplier==1)
      {
        counter=counter-1;
      }     
      
      }
  }
}


void RotateRelative()
{
    //We move X steps from the current position of the stepper motor in a given direction.
    //The direction is determined by the multiplier (+1 or -1)
   
    runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepper.setMaxSpeed(receivedSpeed); //set speed
    stepper.move(directionMultiplier * receivedSteps); //set relative distance and direction
}

void RunTheMotor() //function for the motor
{
    if (runallowed == true)
    {
        stepper.enableOutputs(); //enable pins
        stepper.run(); //step the motor (this will step the motor by 1 step at each loop)  
    }
    else //program enters this part if the runallowed is FALSE, we do not do anything
    {
        stepper.disableOutputs(); //disable outputs
        return;
    }
}

void PrintCommands()
{
  //Printing the results to serial monitor
  Serial.print("Angle:");
  Serial.println(angle);
  Serial.print("Index:");
  Serial.println(index);

}

int degreeCalculator(int counter, double encoderDegree) //enkdoerdan aci degeri konum aci degerine cevrildi
{
  return(counter*30+(encoderDegree*30/360));
}

void checkSerial() //function for receiving the commands
{  
    if (Serial.available() > 0) //if something comes from the computer
    {
        receivedCommand = Serial.read(); // pass the value to the receivedCommad variable
        newData = true; //indicate that there is a new data by setting this bool to true
 
        if (newData == true) //we only enter this long switch-case statement if there is a new command from the computer
        {
            switch (receivedCommand) //we check what is the command
            {
 
            case 'P': //P uses the move() function of the AccelStepper library, which means that it moves relatively to the current position.              
               
                Received_angle = Serial.parseFloat(); //value for the steps
                //receivedSpeed = Serial.parseFloat(); //value for the speed
                directionMultiplier = 1; //We define the direction             
                Serial.println("Negative direction."); //print the action
                //example: P2000 400 - 2000 steps (5 revolution with 400 step/rev microstepping) and 400 steps/s speed
                //In theory, this movement should take 5 seconds
                break;         
 
            case 'N': //N uses the move() function of the AccelStepper library, which means that it moves relatively to the current position.      
               
                Received_angle = Serial.parseFloat(); //value for the steps
                //receivedSpeed = Serial.parseFloat(); //value for the speed 
                directionMultiplier = -1; //We define the direction
                Serial.println("Positive direction."); //print action
                //example: N2000 400 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
                //In theory, this movement should take 5 seconds
                break;

            case 'S': // Stops the motor
               
                Received_angle=degreeCalculator(counter,angle);
                Serial.println("Stopped."); //print action
                break;
               
            case 'H': //H: Homing
 
                runallowed = true;     
                Serial.println("Homing"); //Print the message
                GoHome();// Run the function
                break;
              
       
           
            }
        }
        //after we went through the above tasks, newData is set to false again, so we are ready to receive new commands again.
        newData = false;       
    }
}

void GoHome()
{
  directionMultiplier=1;
  Received_angle=0;
}