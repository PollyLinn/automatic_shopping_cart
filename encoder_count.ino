#define encoder_A 2   //yellow
#define encoder_B 3    //white
#define PWM 5
#define IN3 6
#define IN4 7
volatile long encoderCount = 0;

int interval = 1;

int motor_mode = 1;
long previousMicros = 0; 
long currentMicros = 0; 

int last_motor_mode = 1;
int counter_mode = 0;

void setup(){
  Serial.begin(9600); 
  pinMode(PWM,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(encoder_A, INPUT);
  pinMode(encoder_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_A), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_B), ISR_EncoderB, CHANGE);
  previousMicros = micros();
}

void loop(){
//    currentMicros = micros();
//    if (currentMicros - previousMicros > interval) {
//    previousMicros = currentMicros;
    check_phase(encoder_A);
    check_phase(encoder_B);
//    if(motor_mode != last_motor_mode)
//    { 
//      last_motor_mode = motor_mode; 
//    }
//    else{
//      last_motor_mode = motor_mode;    
//    }
//}
    Serial.print("encoderCount: ");    Serial.println(encoderCount);
}

void ISR_EncoderA() {
  if (digitalRead(encoder_A) == HIGH) {
    if (digitalRead(encoder_B) == LOW) {
        motor_mode = 1;               //1 0
        encoderCount++;
       } 
    else {
        motor_mode = 2;               //1 1
        encoderCount--;
        } 
  }
  else
  {
    if (digitalRead(encoder_B) == HIGH) {
        motor_mode = 3;               //0 1
        encoderCount++;
      } 
    else {
        motor_mode = 4;               //0 0
        encoderCount--;
       }
    }
 }

void ISR_EncoderB() {
  if (digitalRead(encoder_B) == HIGH) {
    if (digitalRead(encoder_A) == HIGH) {
        motor_mode = 2;               //1 1 
        encoderCount++;       
    }
    else {
        motor_mode = 3;               //0 1
        encoderCount--;     
    }
  }
  else {
    if (digitalRead(encoder_A) == LOW) {
        motor_mode = 4;               //0 0
        encoderCount++;
      }          
    else {
        motor_mode = 1;               //1 0
        encoderCount--;
     }         
     }
}



int check_phase(int phase)
{
    int last_phase = digitalRead(phase);
    int phase_counter = 0;
     int origin_phase = 0;

    for(int i=0;i<4;i++)
   {
     if(phase == last_phase)
     {
         phase_counter++;
    }
     last_phase = phase;
   }
   if(phase_counter != 4)
   {
     phase = origin_phase;
   }
   else{
     phase = last_phase;
   } 
}
