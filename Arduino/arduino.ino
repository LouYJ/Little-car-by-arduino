#include <Wire.h>
#include <LSM303.h>
#include <NewPing.h>
#include <PID_v1.h>

//compass
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
//Define some constant
#define Rotate_V 170
#define Forward_V 255
#define Backward_V 255
#define Stop_V 0
#define pi 3.1415926
#define  turn_acuracy 0.01
const byte accurarcy=5;
//calibrate
int count = 0;
//interrupt
const byte interruptlPin = 2,interruptrPin = 3;
const float circuml=20.8,circumr=20.8;
volatile byte state = HIGH;
const int desdis = 200;//stop at
volatile long int lrot=0,lrotb=0;
volatile long int rrot=0,rrotb=0;//4 events
volatile byte ldir=1,rdir=1;
const float L=14.0;//rotate diameter
const float lrot_90=PI*L*384/2/circuml,rrot_90=PI*L*384/2/circumr;

//ultrosonic
#define Sonar_Num     3 
#define Max_Distance 200
#define Ping_Interval 50 
unsigned long pingTimer[Sonar_Num]; 
unsigned int cm[Sonar_Num];         
uint8_t curSensor = 0;         
NewPing sonar[Sonar_Num] = { NewPing(12, 11, Max_Distance), NewPing(31, 32, Max_Distance),NewPing(52, 53, Max_Distance)};

//motor&encoder
const int rspeed_pin = 5,rdir_pin = 4; 
const int lspeed_pin = 6,ldir_pin = 7;
const byte linc=47,rinc=51;//every stage increment
float ldis,rdis;
float lrot_pre,rrot_pre;
int vstage=0;
double cur_ldis = 0, cur_rdis = 0;
float stop_ldis = 0, stop_rdis = 0;
int stopat_flag = 0;
int forward_dis = 0;
double ldiff, rdiff;

//Set the LED (Pin 13) to view the state.
const byte ledPin = 13;
byte ledstate=0;
int led_time=0;
int print_time=0;

//vprint
const int v_interval=30;
unsigned long int printv_time=0;

//int printv_time=0;
float ldis_pre=0,rdis_pre=0;
double vot_l=186,vot_r=200;//this is voltage should be output
double aggKp_l=14, aggKi_l=1.5, aggKd_l=0;
double consKp_l=6, consKi_l=0, consKd_l=0;
int pidflag=0;
float vr_av[5],vl_av[5];
PID lPID(&ldiff, &vot_l, &rdiff,consKp_l, consKi_l, consKd_l,DIRECT);
float error_l,error_r;
unsigned long PID_now;

// Transformation between pi and arduino
int hello = 0;
int ack_data = 0;
int connection = 0;
int op_flag = 1;
int data_flag = 0;
int byte_cnt = 0;
int ins_fin = 0;
int opcode = 0;
int data[4] = {0};

// Output integer
char str[20];
int angle;
float error_pre;

//calibrate car
float theta_sum=0,x_sum=0,d_sum=0;//this can be global 
float ldiff_pre=0,rdiff_pre=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);//Serial init
  Serial3.begin(9600);
  motor_setup();//motor init
  encoder_setup();//interrupt init
  sonar_setup();//sonar init
  compass_setup();//compass init
  delay(500);
  calibrate();
  delay(1000);
  //rotate(0,5);
  pinMode(ledPin, OUTPUT);//LED init 
  // lPID.SetMode(MANUAL);
  lPID.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly: 
  sonar_loop();     
  recv();
  exec_ins();

  if (Serial.available()) {
    char c = Serial.read();
    switch(c) {
      case 'g':
        angle=Serial.parseInt();
        stopat_flag = 1;
        pidflag = 1;
        forward_dis = angle;
       // cal_car(forward_dis);
        cur_ldis=((lrot-lrotb)/384.0)*circuml;
        cur_rdis=((rrot-rrotb)/384.0)*circumr;
        delay(1000);
        break;
      case 'x':
        stopat_flag = 0;
        x_stop();
        pidflag = 0;
        break;
      case 'a': 
        angle=Serial.parseInt();
        a_turn_l(angle);
        break; 
      case 'd': 
        angle=Serial.parseInt();
        d_turn_r(angle);
        break; 
    }
  }
 
  print_v(v_interval);
  if (stopat_flag == 1) {
    Serial.write(data[1]);
    stopat(forward_dis);
   // stopat(desdis);
  }
  //print_form();
  //led_blink(ledstate);
  PIDdis();
  Serial.println(theta_sum);
}

void PIDdis()
{
  unsigned long PID_diff;
  PID_diff=millis()-PID_now;
   if(pidflag && PID_diff>0){
   double gag_l=abs(rdiff-ldiff);
   if(gag_l<0.02)
    lPID.SetTunings(consKp_l,consKi_l,consKd_l);
    else
    lPID.SetTunings(aggKp_l, aggKi_l, aggKd_l);
   lPID.SetSampleTime(100);
   lPID.SetOutputLimits(-255,255);
   lPID.Compute();
    //int pinl=analogRead(lspeed_pin);
    //int pinr=analogRead(rspeed_pin);
    //Serial.println(pinl);
    //Serial.println(pinr);
   forward(vot_l,vot_r);
   }
}
void cal_diff(float ldiff_d,float rdiff_d)
{
  float R=0,theta=0,x=0,d=0;// as local variable 
  if(ldiff_d-rdiff_d!=0)
  {
  Serial.print("thetasum:");
     Serial.print(theta_sum);
     Serial.println();
  R=L*max(ldiff_d,rdiff_d)/(ldiff_d-rdiff_d);//:cm
  theta=(ldiff_d-rdiff_d)/L;
  theta_sum+=theta;//hope -pi/2 to pi/2
  Serial.print("thetasum:");
     Serial.print(theta_sum);
     Serial.println();
  if(R>0)
  x=(R)*(1-cos(theta))*cos(theta_sum);//cm
  else
  x=(R+L)*(1-cos(theta))*cos(theta_sum);
  x_sum+=x;
  d=(ldiff_d+rdiff_d)/2-(abs(R)-L/2)*sin(abs(theta))*cos(theta_sum);
  d_sum+=d;
  
     Serial.print("l");
     Serial.print(ldiff_d);
          Serial.print("r");
     Serial.print(rdiff_d);
     Serial.print("theta:");
     Serial.print(theta);
     Serial.print("thetasum:");
     Serial.print(theta_sum);
     Serial.print(" x_sum:");
     Serial.print(x_sum);
     Serial.print("d:");
     Serial.print(d_sum);
     Serial.println();
  }
  else
  {
    x=rdiff_d*sin(theta_sum);//cm
    x_sum+=x;
    d=(ldiff_d+rdiff_d)/2-(ldiff_d+rdiff_d)/2*cos(theta_sum);
    d_sum+=d;
    
         Serial.print("l");
     Serial.print(ldiff_d);
          Serial.print("r");
     Serial.print(rdiff_d);
     Serial.print("theta:");
     Serial.print(theta);
     Serial.print("thetasum:");
     Serial.print(theta_sum);
     Serial.print(" x_sum:");
     Serial.print(x_sum);
     Serial.print("d:");
     Serial.print(d_sum);
     Serial.println();
  }

}

void cal_car(int next_dis)
{
  Serial.println("this is x_sum");
  Serial.println(x_sum);
  Serial.println(theta_sum);
  Serial.println("this above is theta_sum");
  float tan_angle=x_sum/next_dis;
  float cal_radians=theta_sum+atan(tan_angle);
  turn_radians(cal_radians);
  theta_sum-=cal_radians;
  Serial.println("this is car_radians and next_dis");
  Serial.println(cal_radians);
  Serial.println(next_dis);
  Serial.println(theta_sum);
  Serial.println("this above is theta_sum");
}
void turn_radians(float radian)
{
  float angle=radian/PI*360;  
  if(angle<0)
  turn_left_angle(angle);
  else
  turn_right_angle(angle);
}
void motor_setup()
{
  analogWrite(rspeed_pin, 0); //255 = max power
  analogWrite(lspeed_pin, 0);
  for (int i = 4; i <= 7; i++) //Pin 4 to 7 are used
  pinMode(i, OUTPUT);
 
}
void encoder_setup()
{
  pinMode(interruptlPin,INPUT_PULLUP);
  pinMode(interruptrPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptlPin), l_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptrPin), r_interrupt, CHANGE);

}

void compass_setup() 
{ 
  //Here is to initialize the compass
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}

void sonar_setup() 
{
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < Sonar_Num; i++) // Set the starting time for eaccurarcyh sensor.
    pingTimer[i] = pingTimer[i - 1] + Ping_Interval;
}

void sonar_loop() 
{
   // Serial.println("initdidno");
  for (uint8_t i = 0; i < Sonar_Num; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += Ping_Interval * Sonar_Num;  // Set next time this sensor will be pinged.
      //if (i == 0 && curSensor == Sonar_Num - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[curSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      curSensor = i;                          // Sensor being accurarcycessed.
      cm[curSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[curSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

}
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[curSensor].check_timer())
    cm[curSensor] = sonar[curSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaccurarcyed with your code that does something with the ping results.
  /*for (uint8_t i = 0; i < Sonar_Num; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();*/
}
void w_inc_speed()
{
  vstage=vstage+1>5?5:vstage+1;
  ledstate=abs(vstage);
  analogWrite(rspeed_pin, abs(rinc*vstage)); 
  analogWrite(lspeed_pin, abs(linc*vstage));
  analogWrite(rdir_pin, vstage>0?255:0);
  analogWrite(ldir_pin, vstage>0?255:0); 
  ldir=vstage>0?1:0;
  rdir=vstage>0?1:0;
}
void s_dec_speed()
{
  vstage=vstage-1<-5?-5:vstage-1;
  ledstate=abs(vstage);
  analogWrite(rspeed_pin, abs(rinc*vstage)); 
  analogWrite(lspeed_pin, abs(linc*vstage));
  analogWrite(rdir_pin, vstage>0?255:0);
  analogWrite(ldir_pin, vstage>0?255:0); 
  ldir=vstage>0?1:0;
  rdir=vstage>0?1:0;
}
void a_turn_l(int angle)
{
  /*Serial.println("now turning left");
  compass.read();
  float heading = compass.heading();
  rotate(heading-90>0?heading-90:heading+270,5);*/
  vstage=0;
  ledstate=5;
  lrot_pre=lrot-lrotb;
  rrot_pre=rrot-rrotb;
  //turn_right_90();
  turn_left_angle(angle);
}
void d_turn_r(int angle)
{
  /*Serial.println("now turning right");
  compass.read();
  float heading = compass.heading();
  rotate(heading+90<360?heading+90:heading-270,accurarcy);*/
  vstage=0;
  ledstate=5;
  lrot_pre=lrot-lrotb;
  rrot_pre=rrot-rrotb;
  turn_right_angle(angle);
  
}
void x_stop()
{
  vstage=0;
  ledstate=0;
  analogWrite(rspeed_pin, 0);
  analogWrite(lspeed_pin, 0);
}
void stopat(int desdis)
{
  //Serial.write("Here!!!");
  ldiff = ldis - cur_ldis;
  rdiff = rdis - cur_rdis;
  //forward(vot_l,vot_r);
 // forward(255, 255);
  Serial.print("ldiff: ");
  Serial.print(ldiff);
  Serial.print(", rdiff: ");
  Serial.println(rdiff);
  /*
  sprintf(str, "%d\t", int(ldiff));
  Serial.write(str);
  sprintf(str, "%d\t", int(rdiff));
  Serial.write(str);
  sprintf(str, "%d\n", int(desdis));
  Serial.write(str);*/
   float ldiff_d=ldiff-ldiff_pre;
   float rdiff_d=rdiff-rdiff_pre;
   ldiff_pre=ldiff;
   rdiff_pre=rdiff;
   //cal_diff(ldiff_d,rdiff_d);
   
  if(ldiff+rdiff>(desdis+d_sum)*2)
  {
    pidflag = 0;
    //d_turn_r(8);
    Serial.write("tjbd");
    x_stop();
    stopat_flag = 0;
    stop_ldis = ldiff;
    stop_rdis = rdiff;
    compass.read();
    sprintf(str, "32%04x%04x", int((stop_ldis + stop_rdis)/2), int(compass.heading()));
    Serial3.write(str);
    d_sum=0;//reset d_sum
  }
  
}
void print_form()
{
  if(millis()%1000-print_time>499)
   {
     print_time=millis()%1000; 
     //distance calculate
     ldis=((lrot-lrotb)/384+(lrot-lrotb)%384*1.0/384)*circuml;
     rdis=((rrot-rrotb)/384+(rrot-rrotb)%384*1.0/384)*circumr;
     //heading read
     compass.read();
     float heading = compass.heading();
     /*
     Serial.print("H:");
     Serial.print(heading);
     Serial.print(" LD:");
     Serial.print(ldis);
     Serial.print("cm");
     Serial.print(" RD:");
     Serial.print(rdis);
     Serial.print("cm");
     Serial.print(" S0:");
     Serial.print(cm[0]);
     Serial.print("cm");
     Serial.print(millis());*/
     bonus_print();
     Serial.println();
   }
}
void print_v(int inter)
{
  //const int division=10000;
    ldis=((lrot-lrotb)/384.0)*circuml;
    rdis=((rrot-rrotb)/384.0)*circumr;
    ldiff=ldis-cur_ldis;
    rdiff=rdis-cur_rdis;
    
   /* Serial.print(delay_v);
    Serial.print("hi");
    Serial.print(mill);
    Serial.print("hello");
    error_l=rdiff-ldiff-error_pre;
    error_pre=rdiff-ldiff;
    
    Serial.print(error_l);
    Serial.print("errorl////////left dis:");
    Serial.print(ldiff);
    Serial.print("reft dis:");
    Serial.print(rdiff);
    Serial.println();
    
    printv_time=millis();*/
    
    /*Serial.print(vot_l); 
    Serial.print("vltagel"); 
    Serial.print(vot_r); 
    Serial.print("vltager");  
    Serial.println(); */
  
}
void bonus_print()
{
   Serial.print(" S1:");
   Serial.print(cm[1]);
   Serial.print("cm");
   Serial.print(" S2:");
   Serial.print(cm[2]);
   Serial.print("cm");  
}
void led_blink(byte state)
{
  led_time=millis()%(200*(6-state));
  if(led_time<(100*(6-state)))
  digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)  
  else                  
  digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
}
//interrupt events
void l_interrupt()
{
  if(ldir)lrot++;
  else lrotb++;
}
void r_interrupt()
{
  if(rdir)rrot++;
  else rrotb++;  
}
/*
The next four functions are used to control car go forward or baccurarcykward, turn right or left.
You need input the speed of right and left wheels.
*/
void forward(int lspeed,int rspeed) {
  analogWrite(lspeed_pin, abs(lspeed));
  analogWrite(rspeed_pin, abs(rspeed)); 
  analogWrite(ldir_pin, lspeed>0?255:0); 
  analogWrite(rdir_pin, rspeed>0?255:0);
  ldir=lspeed>0?1:0;
  rdir=rspeed>0?1:0;
}
void turnRight(int lspeed, int rspeed) {
  analogWrite(rspeed_pin, rspeed); //255 = max power
  analogWrite(lspeed_pin, lspeed);
  analogWrite(rdir_pin, 0);
  analogWrite(ldir_pin, 255);
  rdir=0,ldir=1;
}
void turnLeft(int lspeed, int rspeed) {
  analogWrite(rspeed_pin, rspeed); //255 = max power
  analogWrite(lspeed_pin, lspeed);           
  analogWrite(rdir_pin, 255);
  analogWrite(ldir_pin, 0);
  rdir=1,ldir=0;
}

void turn_left_angle(float angle)
{
  float lrot_angle;
  lrot_angle=angle/90.0*lrot_90;
  while(((rrot-rrotb-rrot_pre)+(lrot_pre-lrot+lrotb))<lrot_angle)
  {
    led_blink(ledstate);
    //print_form();
    turnLeft(Rotate_V,Rotate_V);
  }
  x_stop();
}


void turn_right_angle(float angle)
{
  float rrot_angle;
  rrot_angle=angle/90.0*rrot_90;
  while(((lrot-lrotb-lrot_pre)+(rrot_pre-rrot+rrotb))<rrot_angle)
  {
    led_blink(ledstate);
    //print_form();
    turnRight(Rotate_V,Rotate_V);
  }
  x_stop();
}


void turnleftangle(float angle)
{
 float radian=angle/180*PI;
 float theta_sumpre=theta_sum;
 float radian_diff=theta_sum-theta_sumpre+radian;
 cur_ldis=((lrot-lrotb)/384.0)*circuml;
 cur_rdis=((rrot-rrotb)/384.0)*circumr;
 ldiff_pre=0;
 rdiff_pre=0;
 Serial.println(theta_sum);
 Serial.println("hi");
 Serial.print(radian_diff);
  while(abs(radian_diff)>turn_acuracy)
  {

    
   if(radian_diff>0)
   turnLeft(Rotate_V,Rotate_V);
   else
   turnRight(Rotate_V,Rotate_V);  
   print_v(1);
   float ldiff_d=ldiff-ldiff_pre;
   float rdiff_d=rdiff-rdiff_pre;
   ldiff_pre=ldiff;
   rdiff_pre=rdiff;
   cal_diff(ldiff_d,rdiff_d);  
   radian_diff=theta_sum-theta_sumpre+radian;

  }
  theta_sum=theta_sum+radian+radian_diff;
  x_stop();
}


void turnrightangle(float angle)
{
 float radian=angle/180*PI;
 float theta_sumpre=theta_sum;
 float radian_diff=theta_sum-theta_sumpre-radian;
 cur_ldis=((lrot-lrotb)/384.0)*circuml;
 cur_rdis=((rrot-rrotb)/384.0)*circumr;
 ldiff_pre=0;
 rdiff_pre=0;
 Serial.println(theta_sum);
 Serial.println("hi");
 Serial.print(radian_diff);
  while(abs(radian_diff)>turn_acuracy)
  {

    
   if(radian_diff>0)
   turnLeft(Rotate_V,Rotate_V);
   else
   turnRight(Rotate_V,Rotate_V);  
   print_v(1);
      Serial.println(theta_sum);

   float ldiff_d=ldiff-ldiff_pre;
   float rdiff_d=rdiff-rdiff_pre;
   ldiff_pre=ldiff;
   rdiff_pre=rdiff;
   cal_diff(ldiff_d,rdiff_d);  
   radian_diff=theta_sum-theta_sumpre-radian;

  }
  theta_sum=theta_sum-radian-radian_diff;
  x_stop();
}

void calibrate() {
  /*
  while (count <= 6000) {
    turnRight(Rotate_V, Rotate_V);
    count ++;
    compass.read();
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);
    
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);
    compass.m_min = running_min;
    compass.m_max = running_max;
  }
  while (count <= 12000) {
    turnLeft(Rotate_V, Rotate_V);
    count ++;
    compass.read();
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);
    
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);
    compass.m_min = running_min;
    compass.m_max = running_max;
  }*/
  compass.m_min = { -3891,  -2809,  -3288};
  compass.m_max = { +1777,  +3118,   -413};
  
  //compass.m_min = { -3254,  -2238,  -2694};
  //compass.m_max = { +1745,  +2718,   -839};

  //compass.m_min = { -2795,  -4091,  -3170};
  //compass.m_max = { 2648,  1951,  1528};
  turnRight(0,0);
}
void rotate(int dir, int delta) {
  
  compass.read();
  int _mid = (dir + 180) % 360;
  float heading = compass.heading();
  int _max = (360 + dir + delta) % 360;
  int _min = (360 + dir - delta) % 360;
  int flag = 0 ;

  while (flag == 0) {
    //Serial.println("I'm here.");
    compass.read();
    heading = compass.heading();
    //Serial.println(heading);
    if (dir >= delta && dir < 180) {
      if (heading >= _min && heading < _max) {
        forward(0, 0);
        flag = 1;
      } else if (heading > _max && heading < _mid) {
        turnLeft(Rotate_V, Rotate_V);
      } else if (heading >= _mid || heading < _min) {
        turnRight(Rotate_V, Rotate_V);
      } 
    } else if (dir <= 360 - delta && dir >= 180) {
      if (heading >= _min && heading <= _max) {
        forward(0, 0);
        flag = 1;
      } else if (heading >= _mid && heading < _min) {
        turnRight(Rotate_V, Rotate_V);
      } else if (heading > _max || heading < _mid) {
        turnLeft(Rotate_V, Rotate_V);
      } 
    } else if ((dir < delta && dir >= 0) || (dir > 360 - delta && dir <= 360)){
      if ((heading >= _min && heading <= dir)|| (heading <= _max && heading >= dir)) {
        forward(0, 0);
        flag = 1;
      } else if (heading > _max && heading < _mid) {
        turnLeft(Rotate_V, Rotate_V);
      } else if (heading >= _mid && heading < _min) {
        turnRight(Rotate_V, Rotate_V);
      } 
    }
  }
}

int char2int(char c) {
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
}

void recv() {
  if (Serial3.available()) {
    int inByte = Serial3.read();
    if (op_flag == 1 && data_flag == 0) {
      opcode += char2int(inByte) * 16;
      Serial.write("-----------------------------");
      Serial.write("Here\n");
      Serial.write(inByte);
      Serial.write("rinima\n");
      op_flag = 0; 
    } else if (op_flag == 0) {
      Serial.write("qwq\n");
      Serial.write(inByte);
      Serial.write("\n");
      opcode += char2int(inByte);
      op_flag = 1;
      data_flag = 1;
    } else if (op_flag == 1 && data_flag == 1) {
      if (byte_cnt == 0) {
        data[0] += char2int(inByte) * 16;
        Serial.write("angry\n");
        Serial.write(inByte);
      }
      else if (byte_cnt == 1) {
        data[0] += char2int(inByte);
        
        Serial.write("nihao\n");
        sprintf(str, "%d", data[0]);
        Serial.write(str);
        Serial.write("\n");
      }
      else if (byte_cnt == 2) {
        data[1] += char2int(inByte) * 16;
        Serial.write(inByte);
      }
      else if (byte_cnt == 3) {
        data[1] += char2int(inByte);
        Serial.write("nihao\n");
        sprintf(str, "%d", data[1]);
        Serial.write(str);
        Serial.write("\n");
      }
      else if (byte_cnt == 4) {
        data[2] += char2int(inByte) * 16;
        Serial.write(inByte);
      }
      else if (byte_cnt == 5) {
        data[2] += char2int(inByte);
        Serial.write("nihao\n");
        sprintf(str, "%d", data[2]);
        Serial.write(str);
        Serial.write("\n");
      }
      else if (byte_cnt == 6) {
        data[3] += char2int(inByte) * 16;
        Serial.write(inByte);
      }
      else if (byte_cnt == 7) {
        data[3] += char2int(inByte);
        Serial.write("nihao\n");
        sprintf(str, "%d\n", data[3]);
        Serial.write(str);
        Serial.write("------------fuckyou----------\n");
        data_flag = 0;
        ins_fin = 1;
      }
      byte_cnt += 1;
      byte_cnt = byte_cnt%8;
    }
    // Serial.write(inByte);
  }
}


void exec_ins() {
  if (ins_fin == 1) {
    switch (opcode) {
      // 1. handshake and ack------------------------------------------------
      case 0x12:
        if (hello == 0 && connection == 0) {
          Serial.write("Receive hello.\n");
          // Send ack for hello.
          Serial3.write("17");
          hello = 1;
        }
        break;
      case 0x11:
        if (hello == 1) {
          Serial.write("Build connection successfully.\n");
          hello = 0;
          connection = 1;
        }
        break;
      case 0x13:
        // If receive the data_ack from pi, so arduino don't need to send again.
        if (ack_data == 1) {
          ack_data = 0;
          Serial.write("Receive ack_data successfully.");
        }
        break;
        
      // 2. the instructions from pi------------------------------------------------
      case 0x20:
        if (connection == 1) {
          int stop_dis = int((stop_ldis + stop_rdis) / 2);
          compass.read();
          sprintf(str, "31%04x%04x", stop_dis, int(compass.heading()));
          Serial.write("Get instruction.\n");
          Serial3.write(str);
          ack_data = 1;
        } else {
          Serial.write("Not connected.\n");
        }
        break;
      case 0x21:
        Serial.write("10086\n");
        forward_dis = data[0] * 256 + data[1];
        //cal_car(forward_dis);
        
        stopat_flag = 1;
        cur_ldis=((lrot-lrotb)/384.0)*circuml;
        cur_rdis=((rrot-rrotb)/384.0)*circumr;
        pidflag = 1;
        PID_now = millis();
        error_pre = 0;
        ldiff_pre=0;
        rdiff_pre=0;
        /*
        if (data[2] == 1) {
          lPID.SetMode(MANUAL);
        } else {
          lPID.SetMode(AUTOMATIC);
        }*/
        
        Serial.write("Go forward! And the distance is: ");
        sprintf(str, "%d", data[0] * 256 + data[1]);
        Serial.write(str);
        Serial.write("\n");
        Serial3.write("20");
        //
        //Serial3.write("3200000000");
        //Serial3.write("3100C8010E");
        //
        break;
      case 0x22:
        Serial.write("Go backward! And the distance is: ");
        sprintf(str, "%d", data[0] * 256 + data[1]);
        Serial.write(str);
        Serial.write("\n");
        Serial3.write("20");
        break;
      case 0x23:
        a_turn_l(data[2]);
        Serial.write("Turn left! And the degree is: ");
        sprintf(str, "%d", data[2]);
        Serial.write(str);
        Serial.write("\n");
        Serial3.write("20");
        delay(2000);
        break;
      case 0x24:
        d_turn_r(data[2]);
        Serial.write("Turn right! And the degree is: ");
        sprintf(str, "%d", data[2]);
        Serial.write(str);
        Serial.write("\n");
        Serial3.write("20");
        delay(2000);
        break;
      case 0x25:
        x_stop();
        Serial.write("Stop!\n");
        Serial3.write("20");
        break;
      case 0x26:
        if (data[0] == 2) {
          rotate(227, 3);
        } else if (data[0] == 3) {
          rotate(239, 3);
        } else if (data[0] == 4) {
          rotate(229, 3);
        } else if (data[0] == 5) {
          rotate(235, 3);
        } else if (data[0] == 6) {
          rotate(229, 3);
        }
        break;
      case 0x27:
        Serial.write("Finished!\n");
        connection = 0;
        break;
        
    }
    ins_fin = 0;
    opcode = 0;
    for (int i = 0; i < 4; i++) {
      data[i] = 0;
    }
  }
}
