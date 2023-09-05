#define encoder1_A 21   //yellow
#define encoder1_B 20    //white
#define encoder2_A 19   //yellow
#define encoder2_B 18    //white
#define lightG 8
#define lightY 9
#define lightR 10

#define PWM1 6
#define PWM2 7
const byte DirPin1 = 4;
const byte DirPin2 = 5;
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

int x;
int y;
int count = 0;
int interval = 1;

int motor_mode1 = 1;
int last_motor_mode1 = 1;
int counter_mode1 = 0;
int motor_mode2 = 1;
int last_motor_mode2 = 1;
int counter_mode2 = 0;

//------------------------------------
float RPMFilt1 = 0;
float RPMPrev1 = 0;
float RPMFilt2 = 0;
float RPMPrev2 = 0;

float kp = 1; //1//1.4
float ki = 0.0025; //0.0025//0.0009
float kd = 0;

unsigned long t1,t2 = 0;
unsigned long t_prev1 = 0;
unsigned long t_prev2 = 0;
volatile unsigned long count1 = 0;
unsigned long count_prev1 = 0;
volatile unsigned long count2 = 0;
unsigned long count_prev2 = 0;

float Theta1, RPM1, RPM_d1;
float Theta_prev1 = 0;
int dt1,dt2 = 0;
float RPM_max1 = 30;
float Theta2, RPM2, RPM_d2;
float Theta_prev2 = 0;
float RPM_max2 = 30;

long previousMicros = 0; 
long currentMicros = 0;

#define pi 3.1415926
float Vmax = 100;
float Vmin = -100;
float V1 = 0.1; //右
float V2 = 0.1; //左
float e1,e2, e_prev1 = 0, inte1, inte_prev1 = 0, e_prev2 = 0, inte2, inte_prev2 = 0;

void WriteDriverVoltage1(float V, float Vmax) {
  int PWMval1 = int(255 * abs(V) / Vmax);
  if (PWMval1 > 255) {
    PWMval1 = 255;
  }
  if (V < 0) {
    digitalWrite(DirPin1, HIGH);
  }
  else if (V > 0) {
    digitalWrite(DirPin1, LOW);
  }
  else {
    digitalWrite(DirPin1, HIGH);
  }
  analogWrite(PWM1, PWMval1);
}

void WriteDriverVoltage2(float V, float Vmax) {
  int PWMval2 = int(255 * abs(V) / Vmax);
  if (PWMval2 > 255) {
    PWMval2 = 255;
  }
  if (V < 0) {
    digitalWrite(DirPin2, LOW);
  }
  else if (V > 0) {
    digitalWrite(DirPin2, HIGH);
  }
  else {
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWM2, PWMval2);
}

void setup(){
  pinMode(lightG,OUTPUT);
  pinMode(lightY,OUTPUT);
  pinMode(lightR,OUTPUT);
  Serial.begin(115200); 
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(encoder1_A, INPUT);
  pinMode(encoder1_B, INPUT);
  pinMode(encoder2_A, INPUT);
  pinMode(encoder2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_A), ISR1_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1_B), ISR1_EncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_A), ISR2_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_B), ISR2_EncoderB, CHANGE);

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
  
    check_phase(encoder1_A);
    check_phase(encoder1_B);
    check_phase(encoder2_A);
    check_phase(encoder2_B);

    if (Serial.available() > 0)
  {
    if (Serial.read() == 'X')
    {
      x = Serial.parseInt();
//      light(lightY);
      count++;
//      x = map(x, 0, 250, 0, 10);
//      if (Serial.read() == 'Y')
//      {
//        y = Serial.parseInt();
//        if (RPM_max1>50){
//          RPM_max1=50;
//        }
//        else if(RPM_max1<30){
//            RPM_max1=30;
//        }
//         else if(RPM_max2>50){
//              RPM_max2=50;
//         }
//          else  if(RPM_max2<30)
//              {
//                RPM_max2=30;
//              }
//        else{
//        RPM_max1=RPM_max1-x;  //右
//        RPM_max2=RPM_max2+x;  //左
//        }
//
//      }
        if(x>0)
        { light(lightR);
          RPM_max1 = 20;
          RPM_max2 = 30; 
          }
         else if(x<0)
         {light(lightG);
          RPM_max1 = 30;
          RPM_max2 = 20;
          }
         else if(x==0)
         {light(lightG);
          RPM_max1 = 30;
          RPM_max2 = 30;
          }
    
    }
//    while (Serial.available() > 0)
//    {
//      Serial.read();
//    }
  }
  else{
      RPM_max1 = 30;
      RPM_max2 = 30;
    }
  

  if (count1 > count_prev1) {
    t1 = millis();    
    light(lightY);
    Theta1 = encoderCount1*pi*2 / 4000.0;
    dt1 = (t1 - t_prev1);
    RPM_d1 =RPM_max1;
    RPM1 = (-1)*(Theta1 - Theta_prev1) / (2*pi*dt1 / 1000.0) * 60 ;
    e1 = RPM_d1 - RPMFilt1;
    inte1 = inte_prev1 + (dt1 * (e1 + e_prev1) / 2);
    V1 = kp * e1 + ki * inte1 + (kd * (e1 - e_prev1) / dt1) ;
    if (V1 > Vmax) { 
      V1 = Vmax;
      inte1 = inte_prev1;
    }
    if (V1 < Vmin) {
      V1 = Vmin;
      inte1 = inte_prev1;
   }
    WriteDriverVoltage1(V1, Vmax);

    RPMFilt1 = 0.854*RPMFilt1 + 0.0728*RPM1 + 0.0728*RPMPrev1;
    RPMPrev1 = RPM1; 
    Serial.print(RPM1);
    Serial.print("  ");   
    Serial.print(RPM_d1);
    Serial.print("  "); 
    Serial.print(e1);
    Serial.print("  ");
    
//    Serial.println();  
    Theta_prev1 = Theta1;
    count_prev1 = count1;
    t_prev1 = t1;
    inte_prev1 = inte1;
    e_prev1 = e1;
}
  if (count2 > count_prev2) {
    t2 = millis();    
    Theta2 = encoderCount2*pi*2 / 4000.0;
    dt2 = (t2 - t_prev2);
    RPM_d2 =RPM_max2;
    RPM2 = (Theta2 - Theta_prev2) / (2*pi*dt2 / 1000.0) * 60 ;
    e2 = RPM_d2 - RPMFilt2;
    inte2 = inte_prev2 + (dt2 * (e2 + e_prev2) / 2);
    V2 = kp * e2 + ki * inte2 + (kd * (e2 - e_prev2) / dt2) ;
    if (V2 > Vmax) { 
      V2 = Vmax;
      inte2 = inte_prev2;
    }
    if (V2 < Vmin) {
      V2 = Vmin;
      inte2 = inte_prev2;
   }
    WriteDriverVoltage2(V2, Vmax);

    RPMFilt2 = 0.854*RPMFilt2 + 0.0728*RPM2 + 0.0728*RPMPrev2;
    RPMPrev2 = RPM2;   
    Serial.print(RPM2);
    Serial.print("  ");   
    Serial.print(RPM_d2);
    Serial.print("  "); 
    Serial.print(e2);
    Serial.println();  
    Theta_prev2 = Theta2;
    count_prev2 = count2;
    t_prev2 = t2;
    inte_prev2 = inte2;
    e_prev2 = e2;
}

}
void light(int n){
  digitalWrite(n,HIGH);
  delay(50);
  digitalWrite(n,LOW);
  delay(10);
  }
void ISR1_EncoderA() {
  if (digitalRead(encoder1_A) == HIGH) {
    if (digitalRead(encoder1_B) == LOW) {
        motor_mode1 = 1;               //1 0
        encoderCount1++;
       } 
    else {
        motor_mode1 = 2;               //1 1
        encoderCount1--;
        } 
  }
  else
  {
    if (digitalRead(encoder1_B) == HIGH) {
        motor_mode1 = 3;               //0 1
        encoderCount1++;
      } 
    else {
        motor_mode1 = 4;               //0 0
        encoderCount1--;
       }
    }
 }

 void ISR2_EncoderA() {
  if (digitalRead(encoder2_A) == HIGH) {
    if (digitalRead(encoder2_B) == LOW) {
        motor_mode2 = 1;               //1 0
        encoderCount2++;
       } 
    else {
        motor_mode2 = 2;               //1 1
        encoderCount2--;
        } 
  }
  else
  {
    if (digitalRead(encoder2_B) == HIGH) {
        motor_mode2 = 3;               //0 1
        encoderCount2++;
      } 
    else {
        motor_mode2 = 4;               //0 0
        encoderCount2--;
       }
    }
 }

void ISR1_EncoderB() {
  if (digitalRead(encoder1_B) == HIGH) {
    if (digitalRead(encoder1_A) == HIGH) {
        motor_mode1 = 2;               //1 1 
        encoderCount1++;       
    }
    else {
        motor_mode1 = 3;               //0 1
        encoderCount1--;     
    }
  }
  else {
    if (digitalRead(encoder1_A) == LOW) {
        motor_mode1 = 4;               //0 0
        encoderCount1++;
      }          
    else {
        motor_mode1 = 1;               //1 0
        encoderCount1--;
     }         
     }
}

void ISR2_EncoderB() {
  if (digitalRead(encoder2_B) == HIGH) {
    if (digitalRead(encoder2_A) == HIGH) {
        motor_mode2 = 2;               //1 1 
        encoderCount2++;       
    }
    else {
        motor_mode2 = 3;               //0 1
        encoderCount2--;     
    }
  }
  else {
    if (digitalRead(encoder2_A) == LOW) {
        motor_mode2 = 4;               //0 0
        encoderCount2++;
      }          
    else {
        motor_mode2 = 1;               //1 0
        encoderCount2--;
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
  count1++;
  count2++;
}
