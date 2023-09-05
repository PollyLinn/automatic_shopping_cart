#define encoder_A 2   //yellow
#define encoder_B 3    //white
#define PWM 5
const byte DirPin = 7;
volatile long encoderCount = 0;

int interval = 1;

int motor_mode = 1;

int last_motor_mode = 1;
int counter_mode = 0;

//------------------------------------

float kp = 0.5;
float ki = 1 ;
float kd = 0;

unsigned long t;
unsigned long t_prev = 0;
volatile unsigned long count = 0;
unsigned long count_prev = 0;

float Theta, RPM, RPM_d;
float Theta_prev = 0;
int dt;
float RPM_max = 150;

long previousMicros = 0; 
long currentMicros = 0;

#define pi 3.1415926

float e_prev = 0, inte, inte_prev = 0;

float RPMFilt = 0;
float RPMPrev = 0;

void setup(){
  Serial.begin(9600); 
  pinMode(PWM,OUTPUT);
  pinMode(DirPin, OUTPUT);
  pinMode(encoder_A, INPUT);
  pinMode(encoder_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_A), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_B), ISR_EncoderB, CHANGE);

  previousMicros = micros();
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499; //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
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



  if (count > count_prev) {
    t = millis();    
    Theta = encoderCount*pi*2 / 4000.0;
    dt = (t - t_prev);
    RPM_d =RPM_max;
//    if (t / 1000.0 > 100) {
//      RPM_d = 0;
//   }

    
    RPM = (Theta - Theta_prev) / (2*pi*dt / 1000.0) * 60 ;
  
//    RPMFilt = 0.854*RPMFilt + 0.0728*RPM + 0.0728*RPMPrev;
//    RPMPrev = RPM;

    float e = RPM_d - RPM;
    
    inte = inte_prev + (dt * (e + e_prev) / 2);
    float u = kp * e + ki * inte + (kd * (e - e_prev) / dt) ;

    int dir = 1;
    if(u<0){
      dir = -1;
    }
    
     int pwr = (int) fabs(u);
     if(pwr > 255){
      pwr = 255;
    }

    setMotor(dir,pwr,PWM,DirPin);
    
    Serial.print("encoderCount: ");    Serial.println(encoderCount);
    Serial.print("RPM: ");    Serial.print(RPM);
    Serial.print("RPM_d: ");    Serial.println(RPM_d);
    Serial.print("u: ");    Serial.println(u);
    Serial.print("e: ");    Serial.println(e);
    
    Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
}
}

void setMotor(int dir, int pwmVal, int pwm, int dirpin){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(dirpin, HIGH);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(dirpin, LOW);
  }
  else{
    // Or dont turn
    digitalWrite(dirpin, LOW);    
  }
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


ISR(TIMER1_COMPA_vect) {
  count++;
  //Serial.print(count * 0.05); Serial.print(" \t");
}
