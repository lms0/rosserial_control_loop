//Codigo fuente de nodo rosserial. Suscripto al topico cmd_vel, del cual recibe una referencia velocidad <linear, angular> del tipo geometry_msgs/Twist.
//Convierte esa velocidad en velocidades diferenciales para los dos actuadores del robot (Vr derecha, Vl izquirda).
//La referencia es pasada por un algoritmo PID (realimentacion negativa), y se actuan los motores para seguir a la referencia.


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DualVNH5019MotorShield.h"
#include <PID_v1.h>

const byte pinEnc1a = 21;  // Pin encoder 1, senal A
const byte pinEnc2a = 18; // Pin encoder 2, senal A
const byte pinEnc1b = 20;  // Pin encoder 1, senal B
const byte pinEnc2b = 19; // Pin encoder 2, senal B


int Direc;
volatile long paso1=0;    // Contadores: pasos de cada encoder, tiempos entre cada paso
volatile long paso2=0;
unsigned long ta1, tb1, ta2, tb2, dT1, dT2;

int aux=0;
float R = 0.035; // Radio ruedas 
float L = 0.24;  // Longitud entre ruedas
volatile float v=0;
volatile float w=0;
double Vr, Vl =0;

//Declaracion de objetos PID y motores 
double Input1, Output1; 
double Input2, Output2;
double Kp=.5;
double Ki=0.3;
double Kd=0;
PID myPID1(&Input1, &Output1, &Vr, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Vl, Kp, Ki, Kd, DIRECT);
DualVNH5019MotorShield md;

ros::NodeHandle  nh;

//Cada vez que llega un mensaje se convierte <v,omega> en un par de velocidades diferenciales
void Cbmotor( const geometry_msgs::Twist& mensaje_motor){

  v = mensaje_motor.linear.x;
  w = mensaje_motor.angular.z;

  Vl = v + (w*L)/2;
  Vr = v - (w*L)/2;
  
  
  if (v==0 && w==0){
   aux=1; 
   md.setM1Speed(0);
   md.setM2Speed(0);
    
   while(aux==1){
    if (v!=0 || w!=0){
     Input1=0;
     Input2=0;
     ta1=ta2=0;
     aux=0;
    }
   nh.spinOnce();
  } 
  
  
 }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &Cbmotor );


void setup()
{ 
  
  pinMode(pinEnc1a, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEnc1a), interrupcion1, FALLING);
  pinMode(pinEnc2a, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEnc2a), interrupcion2, FALLING);
  pinMode(pinEnc1b, INPUT);
 // attachInterrupt(digitalPinToInterrupt(pinEnc1b), interrupcion1DIREC, CHANGE);
  pinMode(pinEnc2b, INPUT);
 // attachInterrupt(digitalPinToInterrupt(pinEnc2b), interrupcion2DIREC, CHANGE);

  Output1 = 0;
  Output2 = 0;
  md.init();
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-50,50 );
  myPID2.SetOutputLimits(-50,50 );
  md.setM1Speed(0);
  md.setM2Speed(0);
  ta1=ta2=micros();
  
  nh.initNode();
  nh.subscribe(sub);
  md.init();
  Serial.begin(57600);
}

void loop()
{  
   
    myPID1.Compute();
    md.setM1Speed(-(Vr+Output1));
          
    myPID2.Compute();
    md.setM2Speed(Vl+Output2);
   
  
  
  nh.spinOnce();
  
}


void interrupcion1()
{
   if (digitalRead(pinEnc1b)==HIGH) {Direc=1;}
   else {Direc=0;}
   Serial.println(Direc);
   paso1++;
   
   if (paso1%(70*16/6)==0) 
   {
    tb1 = micros();
    if (ta1!=0) {dT1 = tb1-ta1;
    Input1= (60000000UL*400)/(dT1*90);
     if (Vr<0) {
    Input1= -Input1;;
    }
    }
    Serial.println(Input1); 
    ta1=tb1;
   }
}

void interrupcion2()
{

   paso2++;
   
   if (paso2%(70*16/6)==0) 
   {
    tb2 = micros();
    if (ta2!=0) {dT2 = tb2-ta2;
    Input2= (60000000UL*400)/(dT2*90);
    if (Vl<0) {
    Input2= -Input2;
    }
    }
    ta2=tb2;
   }
}

/*
void interrupcion1DIREC(){

if (digitalRead(pinEnc1b)==HIGH){ 
b1=1;
}
else {b1=0;}
}

void interrupcion2DIREC(){

if (digitalRead(pinEnc2b)==HIGH){ 
b2=1;
}
else {b2=0;}
if (digitalRead())
}
*/
