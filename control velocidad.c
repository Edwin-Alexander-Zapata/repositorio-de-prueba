//edwin alexander zapata 
//tecnologia electronica 
//institucion universitaria pascualbravo
//aplicacion a los microcontroladores 

#include <TimerOne.h>

volatile int mensajeria;
int pin_pot_sp = A0;       //LECTURA DE PINES
int pin_pot_kp = A1;
int pin_pot_ti = A2;
int pin_pot_td = A3;
int pin_pot_ts = A4;//**TIEMPO DE MUESTREO**//
int pin_vel = 5; // pin pwm
int pin_freno = 6; //(freno)
int pin_cha = 2;
int pin_chb = 3;
///***VARIABLES***///

unsigned long thA;
unsigned long tlA;
unsigned long thB;
unsigned long tlB;
double res;
double frecA;
double frecB;
double pot_Sp;
double pot_Kp;
double ti_esc;
double td_esc;
double ts_esc;// **TIEMPO DE MUESTREO ESCALIZADO**
double Sp;
double Fb;
double Kp;
double pot_ti;
double pot_td;
double pot_ts;
double error;
double error_dif; // error diferencial
double error_ant; // error anterior
double error_esc;
double I_acum;
double error_ultimo;
double P;//constante proporcional
double I;//constante integrativa
double D; //constante derivativa
double PID;
double velocidad;///**SALIDA DEL CONTROL**//
bool cha;
bool chb;
bool freno;


void setup() {
  Serial.begin(9600);
  pinMode(pin_cha,INPUT);
  pinMode(pin_chb,INPUT);
  pinMode(pin_freno,OUTPUT);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(isr_timer);
  P = 0;
  I = 0;
  P = 0;
  mensajeria=0;
  res = 200;
  I_acum = 0;
  error_ant = 0;
}

void loop() {
  cha = digitalRead(pin_cha);
  chb = digitalRead(pin_chb);
  
  pot_Sp = analogRead(pin_pot_sp);
  pot_Kp = analogRead(pin_pot_kp);
  pot_ti = analogRead(pin_pot_ti);
  pot_td = analogRead ( pin_pot_td);
  pot_ts = analogRead (pin_pot_ts);

  analogWrite(pin_vel,velocidad);
  digitalWrite(pin_freno,freno); 

  Sp = map(pot_Sp,0,1023,0,1000);
  Kp = map(pot_Kp,0,1023,0,200);
  Kp /=10;
  ti_esc = map(pot_ti,0,1023,1,50000);
  td_esc = map (pot_td,0,1023,0,500); 
  td_esc /=10;
  ts_esc = map(pot_ts,0,1023,1,100);
  feedback();
  
  error = Sp-Fb;
  error_ant = error_esc;
  error_esc = map(error,-1000,1000,-255,255);// escalizacion pwm
  error_dif = error_esc - error_ant;
    
    // *** calculo proporcional *****
  P = Kp* error_esc; 
  
  // *** CALCULO INTEGRATIVO **
  I = Kp * ts_esc;     // Kp escalizado * tiempo de muestreo 
  I /= ti_esc;
  I *= error_esc;
  I += I_acum;
  I_acum = I;

  ///*** CALCULO DERIVATIVO **
  D = Kp * td_esc;
  D /= ts_esc;
  D *= error_dif;
  
  PID = P+I;
  PID += D;
  velocidad =  PID;

  if (velocidad>255){
    velocidad=255;}
  else if(velocidad<0){
    velocidad=0;}

  
    
  if (error >0 ){
    velocidad = velocidad;
    freno = LOW;
  }
   else if(error< -2 ){
    velocidad = 0;
    freno = HIGH;
  }
  else{
    velocidad = 0;
    freno = LOW;
  }
    mensajes(); 
    
}