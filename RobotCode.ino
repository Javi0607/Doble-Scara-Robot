#include <PIDController.h>


#include"MotorDC.h"
#include"Stepper.h"


// Definimos las constantes del PID
#define __Kp 26 // Proportional 
#define __Ki 1.7 // Integral 
#define __Kd 2000 // Derivative

#define VRX A8//Pines para el joystick
#define VRY A9
#define JSW 30

#define ESW 20 // Pin para el electroimán
#define EM 35
//Numero de puntos que tomamos
#define N 15
#define LED 40
volatile int aux1, aux2;


  //Definimos el contador de pulsos, variables auxiliares 
  volatile long int cont1 = 0, cont2 = 0, stppos = 0; // Contadores para las posiciones de los motores DC
  
  volatile int setPoint1 = 0, setPoint2 = 0, setPpos = -2000; // setPoint guardará el ángulo al que queremos llegar con los motores.
  
  
  volatile int motor_pwm1 = 0,motor_pwm2 = 0, rpm = 0, steps = 0; //Variables para guardar los valores que recibamos del PID
  String value0, value1, value2, value3, value4;
  String OP;  
  volatile int OP_int;
  volatile int OP_2 = 1;

 

// Constantes para el pid de velocidad
  volatile long prevT_i1 = 0, prevT_i2 = 0;
  volatile float velocity_i1 = 0, velocity_i2 = 0;  
  volatile float vPrev1 = 0,vPrev2 = 0;
  volatile float vfilt1 = 0, vfilt2 = 0;
 

// Variables para llevar el tiempo

volatile int  j = 0, aux = 0;
volatile unsigned long t1, t2;

// Variables para el Joystick
volatile float Xj = 0, Yj = 0;
volatile boolean state = 0;
volatile long int n;

// Variables para la cinemática

volatile float X = 0, Y = 0, xf = 0, yf = 0, Dt = 0, q1 = 0, q2 = 0;
String tam1,tam2,tlength;
volatile float  t;
volatile int trayectoria1[N];
volatile int trayectoria2[N];
volatile float qq1,qq2,vx,vy;
volatile float x_siguiente,y_siguiente;
volatile float Dx,Dy;



  // Creamos dos objetos PIDController, uno para cada uno de los motores.
  PIDController pidpos1;
  PIDController pidpos2;
  PIDController pidvel1;
  PIDController pidvel2;

  //Creamos el objeto del motor stepper
  //Stepper stepperz(stepsPerRevolution, STPP1, STPP2, STPP3, STPP4);
  BasicStepperDriver stepperz(MOTOR_STEPS, DIR, STEP, SLEEP);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Serial for Debugging
  pinMode(FINCARRERA3, INPUT_PULLUP);
  // Setup del Joystick
  joystickSetup();
  EMsetup();
  //Pines del motor 1
  SetupMDC(ENCODER_C11, ENCODER_C12, MOTOR_M11, MOTOR_M12, MOTOR_1PWM, FINCARRERA1 );

  //Pines del motor 2
  SetupMDC(ENCODER_C21, ENCODER_C22, MOTOR_M21, MOTOR_M22, MOTOR_2PWM, FINCARRERA2 );  

  //Interrupcion temporal stepper 1ms

  //Interrupciones encoder 1
  attachInterrupt(digitalPinToInterrupt(ENCODER_C11), encoder11, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C12), encoder12, CHANGE);

  //Interrupciones encoder 2
  attachInterrupt(digitalPinToInterrupt(ENCODER_C21), encoder21, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C22), encoder22, CHANGE);
  
  pidpos1.begin(); // initialize the PID instance
  pidpos1.tune(26, 1.7, 2000); // Tune the PID, arguments: kP, kI, kD
  pidpos1.limit(-100, 100); // Ponemos límite al resultado del PID

  pidpos2.begin(); // initialize the PID instance
  pidpos2.tune(26, 1.7, 2000); // Tune the PID, arguments: kP, kI, kD
  pidpos2.limit(-100, 100); // Ponemos límite al resultado del PID

  pidvel1.begin(); // initialize the PID instance
  pidvel1.tune(25, 10, 1000); // Tune the PID, arguments: kP, kI, kD
  pidvel1.limit(-255, 255); // Ponemos límite al resultado del PID

  pidvel2.begin(); // initialize the PID instance
  pidvel2.tune(25, 10, 1000); // Tune the PID, arguments: kP, kI, kD
  pidvel2.limit(-255, 255); // Ponemos límite al resultado del PID

  // Marcamos la velocidad del stepper.
  stepperz.begin(RPM, MICROSTEPS);  stepperz.disable();  

            //Buesqueda del 0 de los dos motores DC
  cont1 = busqueda0( FINCARRERA1 , MOTOR_M12 , MOTOR_M11, MOTOR_1PWM, cont1);
  //Serial.println("11");
  cont2 = busqueda0( FINCARRERA2 , MOTOR_M22 , MOTOR_M21, MOTOR_2PWM, cont2);
  //Serial.println("22");
  busqueda0stpp();
  //Serial.println("33");
  OP_int = 1;
  setPoint1 = 840;
  setPoint2 = 965;
  
  // Timer1.initialize(1500);
  // Timer1.attachInterrupt(stepmove);
  n=millis();

}



  void loop() {
    // put your main code here, to run repeatedly:
    directa();
  // Serial.print(X);Serial.print(',');Serial.println(Y);
  // Serial.print(q1);Serial.print(',');Serial.println(q2);
    if (Serial.available() ) {
      OP = Serial.readStringUntil(',');
     //value0 = Serial.readStringUntil(',');
      value1 = Serial.readStringUntil(',');
      value2 = Serial.readStringUntil(',');
      value4 = Serial.readStringUntil(',');
      value3 = Serial.readStringUntil('\n');
      
     OP_int = OP.toInt();
    // OP_2 = value0.toInt();

    // if(OP_2){
      xf = float(value1.toInt());
      yf = float(value2.toInt());
      setPpos = value4.toInt();
      t = value3.toInt();   

     // state = 0;   
     //}else state = 1;
    
    //digitalWrite(EM,OP_2);

     
//     Serial.print(OP_int); Serial.print(", "); 
//     Serial.print(xf); Serial.print(", ");
//     Serial.print(yf); Serial.print(", ");
//     Serial.println(t);
  
  aux = 0;
   t1 = millis();
   t2 = t1;
   j = 0;
  traslaciones();
  directa();
  Dx = (xf-X)/N;
  Dy = (yf-Y)/N;
  x_siguiente = X + Dx;
  y_siguiente = Y + Dy;
  velocidad();
}

  
   
    // Aquí seleccionamos las operaciones
      switch(OP_int){
  
         case 0: //Un shutdown de los motores DC
           digitalWrite(MOTOR_M12, LOW);
           digitalWrite(MOTOR_M11, LOW);
           digitalWrite(MOTOR_1PWM, LOW);
           digitalWrite(MOTOR_M22, LOW);
           digitalWrite(MOTOR_M21, LOW);
           digitalWrite(MOTOR_2PWM, LOW);
           stepperz.disable(); 
         break;
          
         case 1:
         funcionPIDpos(setPoint1, setPoint2 );          
         break;
          
        case 2:
        funcionPIDpos(setPoint1, setPoint2 );
        joystick();  
       
        
        break;
  
         case 3:
        
      
           //Serial.print(Vset1);Serial.print(",");Serial.print(Vset2);Serial.print(",");
           Serial.print(vfilt1);Serial.print(",");Serial.println(vfilt2);
          stepmovement(); 
           vfilt1 = velocidadactual(velocity_i1, vfilt1, vPrev1);   
           vfilt2 = velocidadactual(velocity_i2, vfilt2, vPrev2);
           proceso();
           funcionPIDvel(qq1, qq2);
           stepmovement();
           funcionrefresh();
  //        Serial.print(setPpos);Serial.print("");Serial.println(stppos);
          
         break;
         
         case 4:
         if(setPpos != stppos){
         stepmovement();  
         stepperz.enable();
         stepperz.move(steps);
         stppos = setPpos;
         stepperz.disable();
         }
    
           funcionPIDpos(setPoint1, setPoint2 );
             funcionrefresh();
             stepmovement(); 
    
         break;
      }
   // Serial.print(millis());Serial.print(",");Serial.print(integerValue);Serial.print(",");Serial.print(pwm);Serial.print(",");Serial.print(cont);Serial.print("/n");
 
 
  }
