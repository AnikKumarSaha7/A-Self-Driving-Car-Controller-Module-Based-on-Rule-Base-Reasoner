#define USE_ARDUINO_INTERRUPTS true    
#include <PulseSensorPlayground.h>        
#include<math.h>
#include <Wire.h>                                   
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
const int PulseWire = 0;       
const int LED13 = 43;          
int Threshold = 550;                                                               
PulseSensorPlayground pulseSensor;  
double lat1;                                          
double lon1;
double lat2;                                          
double lon2;
double u;
double v;
double w;
double d;
double c;
double e;
double f;
double rad;
double Duration0, Distance0, Duration1, Distance1, Duration2, Distance2; 
double R = 6371000.00;
double d_r = 0.017453;
double r_d = 57.295779;
double n[2];
double m[2];
double g;
double t;
double t1;
double p;
double q;
double degg;
double heading;
double distance;
int i;
int j;
int motorA_in1 = 5;
int motorA_in2 = 6;
int motorB_in1 = 7;
int motorB_in2 = 8;
int motorA_pwm = 10;
int motorB_pwm = 11;
int TrigPin0 = 23;
int EchoPin0 = 25;
int TrigPin1 = 30;
int EchoPin1 = 31;
int TrigPin2 = 32;
int EchoPin2 = 33; 
int encoderIn = 9;
int voltpin = 4;
int counter = 0;
int State;
int LastState;
int raspi_in = 12;
int raspi_in1 = 24;
int k = 13;
int l = 13;
String number;
String Send_Data;
char sms;
TinyGPSPlus gps;
QMC5883LCompass compass;
float biasx = -162.047534;
float biasy = -199.544990;
float biasz = -110.102940;
double x, y, z, a, b;
double Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal;
char myArray[3];

void setup()
{ 
  pinMode(motorA_in1, OUTPUT);
  pinMode(motorA_in2, OUTPUT);
  pinMode(motorB_in1, OUTPUT);
  pinMode(motorB_in2, OUTPUT);
  pinMode(motorA_pwm, OUTPUT);
  pinMode(motorB_pwm, OUTPUT);
  pinMode(TrigPin0, OUTPUT);
  pinMode(EchoPin0, INPUT);
  pinMode(TrigPin1, OUTPUT);
  pinMode(EchoPin1, INPUT);
  pinMode(TrigPin2, OUTPUT);
  pinMode(EchoPin2, INPUT); 
  pinMode(encoderIn, INPUT);
  pinMode(voltpin, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(22, INPUT);
  pinMode(26, INPUT);
  pinMode(28, INPUT);
  pinMode(raspi_in, OUTPUT);
  pinMode(raspi_in1, OUTPUT);
 // Wire.begin();
  Serial.begin(9600);
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED13);       
  pulseSensor.setThreshold(Threshold);   
   if (pulseSensor.begin()) {
    Serial.println("pulseSensor Object is created");   
  }
  compass.init();
}

void loop()
{
int myBPM = pulseSensor.getBeatsPerMinute();  
if (pulseSensor.sawStartOfBeat()) {             
 Serial.println(" HeartBeat! "); 
 Serial.print("BPM: ");                        
 Serial.println(myBPM); 
 check_Distance0();
 check_Distance1();
 check_Distance2();
 if (((myBPM>100) & (myBPM<60)) || (Distance0 < 30) || (Distance1 < 30) || (Distance2 < 30))
 {
  Stop();
  Serial.println("Stop");
 }
 if((myBPM>100) & (myBPM<60))
{
  gpsInfo();
  Send_Data = "Latitude = "+String(lat1, 6)+"\nLongitude = "+String(lon1, 6); 
  Serial.begin(9600);
  delay(1000);
  Serial.println("AT");
  delay(500);
  Serial.println("AT+CMGF=1");
  delay(500);
  Serial.println("AT+CMGS=\""+ number +"\"\r");
  delay(500);
  Serial.println(Send_Data);
  delay(500);
  Serial.write(26);
}
 else
 {
  bluetooth();
  distance_angle_calculation();
  direction1();
 }
}
}

void direction1()
{
if (sms == 'S')
{
if(distance > 0 && digitalRead(2)==LOW && digitalRead(3)==HIGH && digitalRead(22) == LOW)
{ 
  digitalWrite(voltpin, HIGH);
   State = digitalRead(encoderIn);
if (State != LastState)
{     
   counter = counter + 1;
   Serial.println(counter);
}
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Xm_off = x - biasx;
  Ym_off = y - biasy;
  Zm_off = z - biasz;

   Xm_cal =  0.038601* Xm_off + -0.000982 * Ym_off + 0.000062 * Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
  Ym_cal =  -0.000982 * Xm_off + 0.039362 * Ym_off + -0.000483 * Zm_off; //Y-axis correction for combined scale factors
  Zm_cal =  0.000062 * Xm_off + -0.000483 * Ym_off + 0.038661 * Zm_off; //Z-axis correction for combined scale factors
  
  double heading = atan2(Xm_cal, Ym_cal) / 0.0174532925;
    if (heading < 0) {
    heading += 360;
    }
    heading = 360 - heading;
    m[i++] = heading;
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("m[1]: ");
    Serial.println(m[1]);
if (((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
t = degg -heading;
n[j++] = t;
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && ((n[1]>20) || (n[1] < -20)) && (counter<=9))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("forward()/////");
Serial.print("n[1]: ");
Serial.println(n[1]);
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (n[1]>20) && (counter>9)  && ((p == 0) || (p>=9) || (p<=-9)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading-q-l;
TurnLeft();
Serial.println("TurnLeft()");
Serial.println(p);
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (n[1]>20) && (counter>9)  && ((p < 0) && (p>-9)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (n[1]< -20) && (counter>9)  && ((p == 0) || (p>=9) || (p<=-9)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
q = m[1] - 90 ;
if (q<0)
{
q= q +360;
}
p = heading-q+k;
SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (((n[1]>=0) && (n[1]<=20)) || ((n[1] <=0) &&(n[1] >= -20))) && (counter<=9) && ((p == 0) || (p>=9) || (p<=-9)))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("depends on pi");
}
if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (((n[1]>=0) && (n[1]<=20)) || ((n[1] <=0) &&(n[1] >= -20))) && (counter>9) && (counter<=11) && ((p == 0) || (p>=9) || (p<=-9)))
{
Forward();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Forward()");
}
if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (((n[1]>=0) && (n[1]<=20)) || ((n[1] <0) &&(n[1] >= -20))) && (counter > 11) && ((p == 0)))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
if (heading>=0 && heading <90)
{
t = degg - (heading+360);
n[j++] = t;
}
if(heading > 90)
{
t = degg - heading;
}}

if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (n[1] < -20) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>9) || (p<-9)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}

p = heading -q+k;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (n[1] < -20) && (counter>9) && ((p>0) && (p<9)))
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  digitalWrite(raspi_in1, LOW);
  Serial.println("Stop");
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && ((n[1] <= 0) && (n[1] >= -20)) && (counter<=9) && ((p == 0) || (p>9) || (p<-9)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && ((n[1] <= 0) && (n[1] >= -20)) && (counter>9) && (counter<=11) && ((p == 0) || (p>9) || (p<-9)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && ((n[1] <= 0) && (n[1] >= -20)) && (counter>11))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}
if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
if (heading>=270 && heading <=359.99)
{
t = degg - (heading-360);
n[j++] = t;
}
if(heading<270)
{
t = degg - heading;
}}

if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& (n[1] > 20) && (counter <= 9))
{
  digitalWrite(raspi_in1, HIGH);
  digitalWrite(raspi_in, LOW);
  Serial.println("Forward()");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}

if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& (n[1] > 20) && (counter > 9) && ((p == 0) || (p>9) || (p<-9)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q-l;
TurnLeft();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}
if ((n[1] > 20) && (p<= 16) && (p >=-16) && (p != 0))
{
 digitalWrite(raspi_in, HIGH); 
 digitalWrite(raspi_in1, LOW);
}
if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& ((n[1]>=0) && (n[1] <= 20)) && (counter <= 9) && ((p == 0) || (p>9) || (p<-9)))
{
 digitalWrite(raspi_in1, HIGH);
Serial.println("Forward()");
digitalWrite(raspi_in, LOW);
}
if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& ((n[1]>=0) && (n[1] <= 20)) && (counter > 9) && (counter <= 11) && ((p == 0) || (p>9) || (p<-9)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& ((n[1]>=0) && (n[1] <= 20)) && (counter > 11))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - heading;
t1 = (heading +360) - degg;
if (t<t1)
{
n[j++] = t;
}
else if (t>t1)
{
n[j++] = -t1;
}
else if (t==t1)
{
n[j++] = t;
}
}

if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && ((n[1] < -20) || (n[1] > 20)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>9) || (p<-9)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}

p = heading -q + k;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>9) || (p<-9)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q-l;
TurnLeft();
Serial.println("SlowLeftTurn()......");
Serial.println(p);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && (counter <= 11) && ((p == 0) || (p>9) || (p<-9)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = (degg + 360) - heading;
t1 = heading - degg;
if (t1<t)
{
n[j++] = -t1;
}
else if (t1>t)
{
n[j++] = t;
}
else if (t1==t)
{
n[j++] = -t1;
}
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && ((n[1] < -20) || (n[1] > 20)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>9) || (p<-9)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}
p = heading -q +k;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>9) || (p<-9)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q-l;
TurnLeft();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg> 90 && degg <180) && (m[1] <=359.99 && m[1] >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && (counter <= 11) && ((p == 0) || (p>9) || (p<-9)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}

if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = (degg + 360) - heading;
t1 = heading - degg;
if (t1<t)
{
n[j++] = -t1;
}
else if (t1>t)
{
n[j++] = t;
}
else if (t1==t)
{
n[j++] = -t1;
}
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && ((n[1] < -20) || (n[1] > 150)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>9) || (p<-9)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}
p = heading -q +k;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>9) || (p<-9)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q-l;
TurnLeft();
Serial.println("......SlowLeftTurn().....");
Serial.println(p);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && (counter <= 11) && ((p == 0) || (p>9) || (p<-9)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - heading;
t1 = (heading +360) - degg;
if (t<t1)
{
n[j++] = t;
}
else if (t>t1)
{
n[j++] = -t1;
}
else if (t==t1)
{
n[j++] = t;
}
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && ((n[1] < -20) || (n[1] > 20)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>9) || (p<-9)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}
p = heading -q +k;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (n[1] < -20) && (counter>9) && ((p>0) && (p<9))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>9) || (p<-9)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q-l;
TurnLeft();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && (counter <= 11) && ((p == 0) || (p>9) || (p<-9)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}

if ((p>0) && (p<9))
{
 Stop();
 digitalWrite(raspi_in, HIGH);
 Serial.println("Stop") ;
}
if ((p>9) && (p<-9) && (p == 0))
{
  digitalWrite(raspi_in, LOW);
  
}
if(digitalRead(26)==HIGH && digitalRead(28)==LOW && counter <= 9)
  {
    
    TurnRight(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("turnRight()...."); 
  }
if(digitalRead(26)==HIGH && digitalRead(28)==HIGH && counter<= 9)
  {
    
    SlowLeftTurn(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("turnLeft()"); 
  }
if(digitalRead(26)==LOW && digitalRead(28)==HIGH && counter <= 9)
  {
    
    Forward(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("forward()..//"); 
  }
  
LastState = State;
  }
else if (distance > 0 && digitalRead(2)==LOW && digitalRead(3)==LOW && digitalRead(22) == HIGH)
{
   digitalWrite(voltpin, HIGH);
   State = digitalRead(encoderIn);
if (State != LastState)
{     
   counter = counter + 1;
   Serial.println(counter);
}
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Xm_off = x - biasx;
  Ym_off = y - biasy;
  Zm_off = z - biasz;

  Xm_cal =  0.040704* Xm_off + -0.002558 * Ym_off + -0.000953 * Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
  Ym_cal =  -0.002558 * Xm_off + 0.042428 * Ym_off + -0.001274 * Zm_off; //Y-axis correction for combined scale factors
  Zm_cal =  -0.000953 * Xm_off + -0.001274 * Ym_off + 0.037767 * Zm_off; //Z-axis correction for combined scale factors

  double heading = atan2(Xm_cal, Ym_cal) / 0.0174532925;
    if (heading < 0) {
    heading += 360;
    }
    heading = 360 - heading;
    m[i++] = heading;
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("m[1]: ");
    Serial.println(m[1]);
if ((degg>= 0 && degg <=270) && (m[1] >= 0 && m[1] <= 270))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
t = degg -heading;
n[j++] = t;
}

if (((degg>= 0 && degg <=270) && (m[1] >= 0 && m[1] <= 270)) && (n[1] >= 30) && (counter<=9))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("forward()..");
Serial.print("n[1]: ");
Serial.println(n[1]);
}

if (((degg>= 0 && degg <=270) && (m[1] >= 0 && m[1] <= 270)) && (n[1] >= 30) && (counter>9)  && ((p == 0) || (p>16) || (p<-16)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
q = m[1] + 90 ;
p = heading-q;
SlowLeftTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if (((degg>= 0 && degg <=270) && (m[1] >= 0 && m[1] <= 270)) && (n[1] >= 30) && (counter>9)  && ((p < 0) && (p>-16)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}

if (((degg>= 0 && degg <=270) && (m[1] >= 0 && m[1] <= 270)) && (n[1] < 30) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("forward()");
}
if (((degg>= 0 && degg <=270) && (m[1] >= 0 && m[1] <= 270)) && (n[1] < 30)  && (counter>9) && (counter<=12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Forward()");
}
if (((degg>= 0 && degg <=270) && (m[1] >= 0 && m[1] <= 270)) && (n[1] < 30) && (counter > 12) && ((p == 0) || (p>16) || (p<-16)))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}


if (((degg>= 0 && degg <= 270) && (m[1] >= 270 && m[1] <= 359.99)) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = (degg+360) - heading;
n[j++] = t;
}

if (((degg>= 0 && degg <= 270) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] >= 30) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if (((degg>= 0 && degg <= 270) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] >= 30) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] + 90;
if (q>360)
{
q= q - 360;
}

p = heading -q;

SlowLeftTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if (((degg>= 0 && degg <= 270) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] >= 30) && (counter>9)  && ((p < 0) && (p>-16)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}
if (((degg>= 0 && degg <= 270) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] < 30) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if (((degg>= 0 && degg <= 270) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] < 30) && (counter>9) && (counter<= 12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if (((degg>= 0 && degg <= 270) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] < 30) && (counter>12))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}

if (((degg>=270 && degg <=359.99) && (m[1] >= 270 && m[1] <= 359.99)) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - heading;
n[j++] = t;
}

if (((degg>=270 && degg <=359.99) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] >= 30) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] >= 30) && (counter> 9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] + 90;
if (q>360)
{
q= q -360;
}

p = heading -q;

SlowLeftTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if (((degg>=270 && degg <=359.99) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] >= 30) && (counter>9)  && ((p < 0) && (p>-16)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] < 30) && (counter<= 9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] < 30) && (counter>9) && (counter<= 12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 270 && m[1] <= 359.99)) && (n[1] < 30) && (counter>12))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}

if (((degg> 270 && degg <= 359.99) && (m[1] >= 0 && m[1] < 270)) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - (360+heading);
n[j++] = t;
}

if (((degg> 270 && degg <= 359.99) && (m[1] >= 0 && m[1] < 270)) && (n[1] <= -30) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if (((degg> 270 && degg <= 359.99) && (m[1] >= 0 && m[1] < 270)) && (n[1] <= -30) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] + 90;

p = heading -q;

SlowLeftTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if (((degg> 270 && degg <= 359.99) && (m[1] >= 0 && m[1] < 270)) && (n[1] <= -30) && (counter>9)  && ((p < 0) && (p>=-16)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}
if (((degg> 270 && degg <= 359.99) && (m[1] >= 0 && m[1] < 270)) && (n[1] >= -30) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if (((degg> 270 && degg <= 359.99) && (m[1] >= 0 && m[1] < 270)) && (n[1] >= -30) && (counter>9) && (counter<=12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if (((degg> 270 && degg <= 359.99) && (m[1] >= 0 && m[1] < 270)) && (n[1] >= -30) && (counter>12))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}
if(digitalRead(26)==HIGH && digitalRead(28)==LOW && counter <= 9)
  {
    
    TurnRight(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("turnRight()...."); 
  }
  if(digitalRead(26)==HIGH && digitalRead(28)==HIGH && counter<= 9)
  {
    
    SlowLeftTurn(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("turnLeft()"); 
  }
  if(digitalRead(26)==LOW && digitalRead(28)==HIGH && counter <= 9)
  {
    
    Forward(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("forward()..//"); 
  }
LastState = State;
}
else if(distance > 0 && digitalRead(2)==LOW && digitalRead(3)==HIGH && digitalRead(22) == HIGH)
{ 
  digitalWrite(voltpin, HIGH);
   State = digitalRead(encoderIn);
if (State != LastState)
{     
   counter = counter + 1;
   Serial.println(counter);
}
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Xm_off = x - biasx;
  Ym_off = y - biasy;
  Zm_off = z - biasz;

   Xm_cal =  0.038601* Xm_off + -0.000982 * Ym_off + 0.000062 * Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
  Ym_cal =  -0.000982 * Xm_off + 0.039362 * Ym_off + -0.000483 * Zm_off; //Y-axis correction for combined scale factors
  Zm_cal =  0.000062 * Xm_off + -0.000483 * Ym_off + 0.038661 * Zm_off; //Z-axis correction for combined scale factors
  
  double heading = atan2(Xm_cal, Ym_cal) / 0.0174532925;
    if (heading < 0) {
    heading += 360;
    }
    heading = 360 - heading;
    m[i++] = heading;
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("m[1]: ");
    Serial.println(m[1]);
if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (counter<=9))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
t = degg -heading;
n[j++] = t;
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && ((n[1]>20) || (n[1] < -20)) && (counter<=9))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("forward()");
Serial.print("n[1]: ");
Serial.println(n[1]);
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (n[1]>20) && (counter>9)  && ((p == 0) || (p>=16) || (p<=-16)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading-q;
TurnLeft();
Serial.println("TurnLeft()");
Serial.println(p);
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (n[1]>20) && (counter>9)  && ((p < 0) && (p>-16)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (n[1]< -20) && (counter>9)  && ((p == 0) || (p>=16) || (p<=-16)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
q = m[1] - 90 ;
if (q<0)
{
q= q +360;
}
p = heading-q;
SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (n[1]< -20) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}

if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (((n[1]>=0) && (n[1]<=20)) || ((n[1] <=0) &&(n[1] >= -20))) && (counter<=9) && ((p == 0) || (p>=16) || (p<=-16)))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("forward()");
}
if ((((degg>=0 && degg <=180) && (m[1] >= 0 && m[1] <=180)) || ((degg> 180 && degg <=359.99) && (m[1] > 180 && m[1] <= 359.99)) || ((degg > 90 && degg < 270) && (m[1] > 90 && m[1] < 270))) && (((n[1]>=0) && (n[1]<=20)) || ((n[1] <=0) &&(n[1] >= -20))) && (counter>9) && ((p == 0) || (p>=16) || (p<=-16)))
{
Stop();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Stop()");
}

if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
if (heading>=0 && heading <90)
{
t = degg - (heading+360);
n[j++] = t;
}
if(heading > 90)
{
t = degg - heading;
}}

if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (n[1] < -20) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}

p = heading -q;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
 
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && (n[1] < -20) && (counter>9) && ((p>0) && (p<16)))
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  digitalWrite(raspi_in1, LOW);
  Serial.println("Stop");
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && ((n[1] <= 0) && (n[1] >= -20)) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if ((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <90) && ((n[1] <= 0) && (n[1] >= -20)) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
Stop();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Stop()");
}
if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
if (heading>=270 && heading <=359.99)
{
t = degg - (heading-360);
n[j++] = t;
}
if(heading<270)
{
t = degg - heading;
}}

if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& (n[1] > 20) && (counter <= 9))
{
  digitalWrite(raspi_in1, HIGH);
  digitalWrite(raspi_in, LOW);
  Serial.println("Forward()");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}

if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& (n[1] > 20) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q;
SlowLeftTurn();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}

if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& (n[1] > 20) && (counter > 9) && ((p < 0) && (p>-16)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}

if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& ((n[1]>=0) && (n[1] <= 20)) && (counter <= 9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in1, HIGH);
Serial.println("Forward()");
digitalWrite(raspi_in, LOW);
}
if ((m[1]>=270 && m[1] <=359.99) && (degg >= 0 && degg <90)&& ((n[1]>=0) && (n[1] <= 20)) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
Stop();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Stop()");
}


if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - heading;
t1 = (heading +360) - degg;
if (t<t1)
{
n[j++] = t;
}
else if (t>t1)
{
n[j++] = -t1;
}
else if (t==t1)
{
n[j++] = t;
}
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && ((n[1] < -20) || (n[1] > 20)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}

p = heading -q;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (n[1] < -20) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");  
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q;
SlowLeftTurn();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (n[1] > 20) && (counter > 9) && ((p < 0) && (p>-16)))
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");  
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg>180 && degg <270) && (m[1] >= 0 && m[1] <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
Stop();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Stop()");
}

if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = (degg + 360) - heading;
t1 = heading - degg;
if (t1<t)
{
n[j++] = -t1;
}
else if (t1>t)
{
n[j++] = t;
}
else if (t1==t)
{
n[j++] = -t1;
}
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && ((n[1] < -20) || (n[1] > 20)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}
p = heading -q;
SlowRightTurn();
Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (n[1] < -20) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q;
SlowLeftTurn();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (n[1] > 20) && (counter > 9) && ((p < 0) && (p>-16)))
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
}
if ((degg> 90 && degg < 180) && (m[1] <=359.99 && m[1] >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((degg> 90 && degg <180) && (m[1] <=359.99 && m[1] >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
Stop();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Stop()");
}

if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = (degg + 360) - heading;
t1 = heading - degg;
if (t1<t)
{
n[j++] = -t1;
}
else if (t1>t)
{
n[j++] = t;
}
else if (t1==t)
{
n[j++] = -t1;
}
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && ((n[1] < -20) || (n[1] > 20)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}
p = heading -q;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (n[1] < -20) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q;
SlowLeftTurn();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (n[1] > 20) && (counter > 9) && ((p < 0) && (p>-16)))
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]>180 && m[1] <270) && (degg >= 0 && degg <=180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
Stop();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Stop()");
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - heading;
t1 = (heading +360) - degg;
if (t<t1)
{
n[j++] = t;
}
else if (t>t1)
{
n[j++] = -t1;
}
else if (t==t1)
{
n[j++] = t;
}
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && ((n[1] < -20) || (n[1] > 20)) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (n[1] < -20) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}
p = heading -q;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (n[1] < -20) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (n[1] > 20) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, LOW);
q = m[1] + 90 ;
if (q>360)
{
q = q-360;
}
p = heading -q;
SlowLeftTurn();
Serial.println("SlowLeftTurn()");
Serial.println(p);
}

if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (n[1] > 20) && (counter > 9) && ((p < 0) && (p>-16)))
{
Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}

if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if ((m[1]> 90 && m[1] <180) && (degg <=359.99 && degg >= 180) && (((n[1] <= 0) && (n[1] > -20)) || ((n[1] >= 0) && (n[1] < 20))) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
Stop();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Stop()");
}

 if(digitalRead(26)==HIGH && digitalRead(28)==LOW && counter <= 9)
  {
    
    TurnRight(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("turnRight()"); 
  }
  if(digitalRead(26)==HIGH && digitalRead(28)==HIGH && counter<= 9)
  {
    
    SlowLeftTurn(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("turnLeft()"); 
  }
  if(digitalRead(26)==LOW && digitalRead(28)==HIGH && counter <= 9)
  {
    
    Forward(); 
    digitalWrite(raspi_in, LOW);
   
    Serial.println("forward()"); 
  }
  LastState = State;
 }
  
else if(distance > 0 && digitalRead(2)==HIGH && digitalRead(3)== LOW && digitalRead(22) == HIGH)
{ 
   digitalWrite(voltpin, HIGH);
   State = digitalRead(encoderIn);
if (State != LastState)
{     
   counter = counter + 1;
   Serial.println(counter);
}
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Xm_off = x - biasx;
  Ym_off = y - biasy;
  Zm_off = z - biasz;

  Xm_cal =  0.040704* Xm_off + -0.002558 * Ym_off + -0.000953 * Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
  Ym_cal =  -0.002558 * Xm_off + 0.042428 * Ym_off + -0.001274 * Zm_off; //Y-axis correction for combined scale factors
  Zm_cal =  -0.000953 * Xm_off + -0.001274 * Ym_off + 0.037767 * Zm_off; //Z-axis correction for combined scale factors

  double heading = atan2(Xm_cal, Ym_cal) / 0.0174532925;
    if (heading < 0) {
    heading += 360;
    }
    heading = 360 - heading;
    m[i++] = heading;
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("m[1]: ");
    Serial.println(m[1]);
if (((degg>=90 && degg <=359.99) && (m[1] >= 90 && m[1] <=359.99)) || ((degg>= 0 && degg <=90) && (m[1] > 90 && m[1] <= 270)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
t = degg -heading;
n[j++] = t;
}
if ((((degg>=90 && degg <=359.99) && (m[1] >= 90 && m[1] <=359.99)) || ((degg>= 0 && degg <=90) && (m[1] > 90 && m[1] <= 270))) && (n[1] < -30) && (counter<=9))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("forward()");
Serial.print("n[1]: ");
Serial.println(n[1]);
}

if ((((degg>=90 && degg <=359.99) && (m[1] >= 90 && m[1] <=359.99)) || ((degg>= 0 && degg <=90) && (m[1] > 90 && m[1] <= 270))) && (n[1]< -30) && (counter>9)  && ((p == 0) || (p>16) || (p<-16)))
{
  digitalWrite(raspi_in1, LOW);
  digitalWrite(raspi_in, LOW);
q = m[1] - 90 ;
p = heading-q;
SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if ((((degg>=90 && degg <=359.99) && (m[1] >= 90 && m[1] <=359.99)) || ((degg>= 0 && degg <=90) && (m[1] > 90 && m[1] <= 270))) && (n[1]< -30) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}

if ((((degg>=90 && degg <=359.99) && (m[1] >= 90 && m[1] <=359.99)) || ((degg>= 0 && degg <=90) && (m[1] > 90 && m[1] <= 270))) && (n[1] >= -30) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
digitalWrite(raspi_in, LOW);
Serial.println("forward()");
}
if ((((degg>=90 && degg <=359.99) && (m[1] >= 90 && m[1] <=359.99)) || ((degg>= 0 && degg <=90) && (m[1] > 90 && m[1] <= 270))) && (n[1]>= -30)  && (counter>9) && (counter<=12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
Serial.println("Forward()");
}
if ((((degg>=90 && degg <=359.99) && (m[1] >= 90 && m[1] <=359.99)) || ((degg>= 0 && degg <=90) && (m[1] > 90 && m[1] <= 270))) && (n[1] >= -30) && (counter > 12) && ((p == 0) || (p>6) || (p<-6)))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <270)) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - (heading+360);
n[j++] = t;
}

if (((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <270)) && (n[1] < -30) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <270)) && (n[1] < -30) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}

p = heading -q;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if (((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <270)) && (n[1] < -30) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
}

if (((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <270)) && (n[1] >= -30) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <270)) && (n[1] >= -30) && (counter>9) && (counter<=12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if (((degg>=270 && degg <=359.99) && (m[1] >= 0 && m[1] <270)) && (n[1] >= -30) && (counter>12))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}

if (((degg>=0 && degg <90) && (m[1] >= 0 && m[1] <90)) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = degg - heading;
n[j++] = t;
}

if (((degg>=0 && degg <90) && (m[1] >= 0 && m[1] <90)) && (n[1] < -30) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if (((degg>=0 && degg <90) && (m[1] >= 0 && m[1] <90)) && (n[1] < -30) && (counter>9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;
if (q<0)
{
q= q +360;
}

p = heading -q;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if (((degg>=0 && degg <90) && (m[1] >= 0 && m[1] <90)) && (n[1] < -30) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}
if (((degg>=0 && degg < 90) && (m[1] >= 0 && m[1] <90)) && (n[1] >= -30) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if (((degg>= 0 && degg < 90) && (m[1] >= 0 && m[1] <90)) && (n[1] >= -30) && (counter>9) && (counter<=12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if (((degg>= 0 && degg <= 90) && (m[1] >= 0 && m[1] <90)) && (n[1] >= -30) && (counter>12))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}


if (((degg>=0 && degg <90) && (m[1] >= 90 && m[1] <= 359.99)) && (counter <= 9))
{
digitalWrite(raspi_in, LOW);
digitalWrite(raspi_in1, LOW);
t = (360+degg) - heading;
n[j++] = t;
}

if (((degg>=0 && degg <90) && (m[1] >= 90 && m[1] <=359.99)) && (n[1] > 30) && (counter <= 9))
{
  digitalWrite(raspi_in, LOW);
  digitalWrite(raspi_in1, HIGH);
  Serial.println("High");
  Serial.print("n[1]: ");
  Serial.println(n[1]);
}
if (((degg>=0 && degg <90) && (m[1] >= 90 && m[1] <=359.99)) && (n[1] > 30) && (counter > 9) && ((p == 0) || (p>16) || (p<-16)))
{
 digitalWrite(raspi_in, LOW);
 digitalWrite(raspi_in1, LOW);
q = m[1] - 90;

p = heading -q;

SlowRightTurn();
  Serial.println("........FastTurn.....");
Serial.println(p);
}

if (((degg>=0 && degg <90) && (m[1] >= 90 && m[1] <=359.99)) && (n[1] > 30) && (counter>9) && ((p>0) && (p<16))) 
{
  Stop();
  digitalWrite(raspi_in, HIGH);
  Serial.println("Stop");
  
}
if (((degg>=0 && degg < 90) && (m[1] >= 90 && m[1] <=359.99)) && (n[1] <= 30) && (counter<=9) && ((p == 0) || (p>16) || (p<-16)))
{
digitalWrite(raspi_in1, HIGH);
Serial.println("HIgH");
digitalWrite(raspi_in, LOW);
}
if (((degg>= 0 && degg < 90) && (m[1] >= 90 && m[1] <=359.99)) && (n[1] <= 30) && (counter>9) && (counter<=12) && ((p == 0) || (p>16) || (p<-16)))
{
Forward();
Serial.println("Forward()");
digitalWrite(raspi_in1, LOW);
digitalWrite(raspi_in, LOW);
}
if (((degg>= 0 && degg < 90) && (m[1] >= 90 && m[1] <359.99)) && (n[1] <= 30) && (counter>12))
{
   digitalWrite(raspi_in, HIGH);
   digitalWrite(raspi_in1, LOW);
}
LastState = State;
}

else if(distance > 0 && digitalRead(2)==HIGH && digitalRead(3)==LOW && digitalRead(22) == LOW)
  {
    i=0;
    j=0;
    p = 0;
    TurnRight();
    digitalWrite(voltpin, LOW);
    digitalWrite(raspi_in, LOW);
    digitalWrite(raspi_in1, LOW);
    counter = 0;
    
    Serial.println("TurnRight()"); 
  }
  else if(distance > 0 && digitalRead(2)==HIGH && digitalRead(3)==HIGH && digitalRead(22) == LOW)
  {
    i=0;
    j=0;
    SlowLeftTurn();
    counter = 0;
    p = 0;
    digitalWrite(voltpin, LOW);
     digitalWrite(raspi_in, LOW);
     digitalWrite(raspi_in1, LOW);
   
    Serial.println("TurnLeft()");
  }
  else if(distance > 0 && digitalRead(2)==LOW && digitalRead(3)==LOW && digitalRead(22) == LOW )
  {
    i=0;
    j=0;
   Forward();
   digitalWrite(voltpin, LOW);
   digitalWrite(raspi_in, LOW);
   digitalWrite(raspi_in1, LOW);
   counter = 0;
   p = 0;
   Serial.println("Forward()");
  } 
else if(distance <= 0 && digitalRead(2)==HIGH && digitalRead(3)==HIGH && digitalRead(22) == HIGH)
  {
   i=0;
    j=0; 
    counter = 0;
    p = 0;
    Stop();
    digitalWrite(voltpin, LOW);
    digitalWrite(raspi_in, LOW);
    digitalWrite(raspi_in1, LOW);
    Serial.println("Stop().....");
  }
else if ((Distance0 < 30 || Distance1 < 30 || Distance2 < 30) && distance <= 0)
{
  Stop();
  Serial.println("Stop()");
}
else if (distance <= 0)
{
  Stop();
  Serial.println("You reached your destination");
}
}
}

void getGPS()                                                 
{
    while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      gpsInfo();

  
  if (millis() > 300 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
}
void gpsInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    lat1 = gps.location.lat();
    Serial.println(lat1, 6);
    Serial.print("Longitude: ");
    lon1 = gps.location.lng();
    Serial.println(lon1, 6);
  }
  else
  {
    Serial.println("Location: Not Available");
  }
}
void distance_angle_calculation()
{
  getGPS();
    u = (sq( sin( ((lat2 - lat1)*d_r) / 2.0)));
   // Serial.println(u);
    v = cos(lat1*d_r) * cos(lat2*d_r);
    //Serial.println(v);
    w = (sq(sin( ((lon2 - lon1)*d_r) / 2.0)));
   // Serial.println(w);
    d = u + (v * w);
   // Serial.println(d);
    c = 2.0 * atan2(sqrt(d), sqrt(1.0-d));
   // Serial.println(c);
    distance = c * R;
    Serial.println(distance, 6);
    e = cos(lat2*d_r) * sin((lon2*d_r) - (lon1*d_r));
    f = (cos(lat1*d_r) * sin(lat2*d_r)) - (sin(lat1*d_r) * cos(lat2*d_r) * cos((lon2*d_r)-(lon1*d_r)));
    rad = atan2(e, f);
    Serial.println(rad, 6);
    degg = rad * r_d;
    if (degg < 0)
    {
      degg = 360 + degg;
      Serial.print("degg: ");
      Serial.println(degg, 6);
      
    }
    else
    { 
      degg = degg;
      Serial.print("degg: ");
      Serial.println(degg, 6);
      
    }
}
void bluetooth()
{
   while(Serial.available()>0)
  {
    sms = Serial.read();
    Serial.println(sms);
  }
  if(sms == 'A')
  {
   lat2 = 23.900218;                                          
   lon2 = 89.138542; 
  }
  if (sms == 's')
  {
    Stop();
    Serial.println("Stop()");
  }
}

void Forward()
{
  analogWrite(motorA_pwm, 90);
  analogWrite(motorB_pwm, 90);
  digitalWrite(motorA_in1, HIGH);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in1, HIGH);
  digitalWrite(motorB_in2, LOW);
}

void Backward()
{
  analogWrite(motorA_pwm, 255);
  analogWrite(motorB_pwm, 255);
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, HIGH);
  digitalWrite(motorB_in1, LOW);
  digitalWrite(motorB_in2, HIGH);
}

void TurnRight()
{
  analogWrite(motorA_pwm, 0);
  analogWrite(motorB_pwm, 141);
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in1, HIGH);
  digitalWrite(motorB_in2, LOW);  
}

void TurnLeft()
{
  analogWrite(motorA_pwm, 255);
  analogWrite(motorB_pwm, 0);
  digitalWrite(motorA_in1, HIGH);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in1, LOW);
  digitalWrite(motorB_in2, LOW);  
}

void SlowRightTurn()
{
  analogWrite(motorA_pwm, 0);
  analogWrite(motorB_pwm, 255);
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in1, HIGH);
  digitalWrite(motorB_in2, LOW);
  Serial.println("........FastTurn.....");
}

void SlowLeftTurn()
{
  analogWrite(motorA_pwm, 141);
  analogWrite(motorB_pwm, 0);
  digitalWrite(motorA_in1, HIGH);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in1, LOW);
  digitalWrite(motorB_in2, LOW);  
}

void Stop()
{
  analogWrite(motorA_pwm, 0);
  analogWrite(motorB_pwm, 0);
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in1, LOW);
  digitalWrite(motorB_in2, LOW);
}

void check_Distance0()
{
digitalWrite(TrigPin0, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin0, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin0, LOW);
Duration0 = pulseIn(EchoPin0, HIGH);
Distance0 = (Duration0 *0.034)/2;
Serial.print("Distance0: ");
Serial.print(Distance0);
Serial.println (" cm");
}

void check_Distance1()
{
digitalWrite(TrigPin1, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin1, LOW);
Duration1 = pulseIn(EchoPin1, HIGH);
Distance1 = (Duration1 *0.034)/2;
Serial.print("Distance1: ");
Serial.print(Distance1);
Serial.println (" cm");
}

void check_Distance2()
{
digitalWrite(TrigPin2, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin2, LOW);
Duration2 = pulseIn(EchoPin2, HIGH);
Distance2 = (Duration2 *0.034)/2;
Serial.print("Distance2: ");
Serial.print(Distance2);
Serial.println (" cm");
}
