#include <AccelStepper.h>
#include <MultiStepper.h> //Added for synchronous movement
//ROS
#include <ros.h>
#include <geometry_msgs/Quaternion.h>

#define limitSwitch1 9    // Define el pin del interruptor del límite theta1
#define limitSwitch2 10   // Define el pin del interruptor del límite theta2
#define limitSwitch4 11   // Define el pin del interruptor del límite z
#define limitSwitch3 A3   // Define el pin del interruptor del límite phi

// Init functions
void homing();
float unitToStep(int, float);
void homeMotor(int);

// Variables
long stepper1Position;
long stepper2Position;
long stepper3Position;
long stepper4Position;
bool homingStatus = false;
AccelStepper stepper[4] = {
  AccelStepper(1,2,5),    // Motor Articulacion 1 theta1
  AccelStepper(1,3,6),    // Motor Articulacion 3 theta2
  AccelStepper(1,12,13),  // Motor Articulacion 2 z
  AccelStepper(1,4,7),    // Motor Articulacion 4 phi
};
MultiStepper steppers;


// Constantes
const int defaultVel = 900;
const int defaultAcel = 250;
const float toStep[4] = {
  43.13933379,    // Constante de conversion del theta1; valor teorico -> 44.444444
  36.72147283,    // Constante de conversion del theta2; valor teorico ->  35.555555
  -25,          // Constante de conversion del z
  -10.27156257,    // Constante de conversion del phi valor teorico -> 8.8888
};

//ROS
ros::NodeHandle nh; // Node handler
geometry_msgs::Quaternion outputMessage;
ros::Publisher pub("/joint_pos", &outputMessage);

void callbackFcn(const geometry_msgs::Quaternion &inputMessage){
  // Extraer los valores de theta1, theta2, phi, z del comando
  float theta1 = inputMessage.x;
  float theta2 = inputMessage.y;
  float phi = inputMessage.w;
  float z = inputMessage.z;
  
  //Move to position
  // Unit to Steps conversion
  float stepperPosition[4] = {
    stepper1Position = theta1 * toStep[0],
    stepper2Position = theta2 * toStep[1],
    stepper4Position = z * toStep[2],
    stepper3Position = phi * toStep[3],
  };
  long positions[4];
  positions[0] = stepper1Position;
  positions[1] = stepper2Position;
  positions[2] = stepper4Position;
  positions[3] = stepper3Position;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  nh.logwarn("In position!");

  // Publish current position data to joint_pos
  outputMessage.x = stepper[0].currentPosition() / toStep[0];
  outputMessage.y = stepper[1].currentPosition() / toStep[1];
  outputMessage.z = stepper[3].currentPosition() / toStep[3];
  outputMessage.w = stepper[2].currentPosition() / toStep[2];
  pub.publish(&outputMessage);

}

ros::Subscriber<geometry_msgs::Quaternion> sub("/desired_joint_pos", &callbackFcn);
void setup() {
  Serial.begin(57600);
  //* ROS *
  nh.initNode(); // TODO opt: Change node name
  nh.advertise(pub);
  nh.subscribe(sub);
  //* Hardware *
  // Límites de carrera
  pinMode(limitSwitch1, INPUT); // Limite de carrera límite theta1
  pinMode(limitSwitch2, INPUT); // Limite de carrera límite theta2
  pinMode(limitSwitch4, INPUT); // Limite de carrera límite z
  pinMode(limitSwitch3, INPUT); // Limite de carrera ímite phi

  // Configuraciones iniciales del motor
  stepper[0].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 1     
  stepper[0].setAcceleration(defaultAcel);  // Establecer la aceleración del motor 1
  stepper[3].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 2
  stepper[3].setAcceleration(defaultAcel);  // Establecer la aceleración del motor 2
  stepper[1].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 3
  stepper[1].setAcceleration(defaultAcel);  // Establecer la aceleración del motor 3
  stepper[2].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 4
  stepper[2].setAcceleration(defaultAcel);  // Establecer la aceleración del motor 4
  
  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper[0]);
  steppers.addStepper(stepper[1]);
  steppers.addStepper(stepper[2]);
  steppers.addStepper(stepper[3]);

  homing();
  
}

void loop() {
  nh.spinOnce();
  delay(1);
}
  
  // Homing
void homing() {
    // Homing order z -> phi -> theta2 -> theta1
  homeMotor(2);
  homeMotor(3);
  homeMotor(1);
  homeMotor(0);
}

float unitToStep( int motor, float unit){
  return toStep[motor] * unit;
}

void homeMotor(int motor){
  float unit;
  int speed = -500;
  int limit;
  int moveto = 0;
  
  switch (motor) {
    case 0: //theta 1 
      unit = unitToStep(motor, -182); limit = limitSwitch1; break;
    case 1: //theta2
      unit = unitToStep(motor, -155); limit = limitSwitch2; break;
    case 2: //z
      unit = unitToStep(motor, 139); moveto = unitToStep(motor, 125); limit = limitSwitch4; break;
    case 3: //phi
      unit = unitToStep(motor, -193); speed = 900; limit = limitSwitch3; break;
    default: break;
  }

  while (!digitalRead(limit) == 0) {
    if(digitalRead(limit)==0){break;}
    stepper[motor].setAcceleration(defaultAcel);
    stepper[motor].setSpeed(speed);
    stepper[motor].runSpeed();
    stepper[motor].setCurrentPosition(unit);
  }
  delay(20);
  stepper[motor].moveTo(moveto);
  while (stepper[motor].currentPosition() != moveto) {
    stepper[motor].run();
  }
  delay(20);
}






