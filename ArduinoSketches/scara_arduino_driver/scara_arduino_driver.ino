#include <AccelStepper.h>
#include <math.h>

#define limitSwitch1 9    // Define el pin del interruptor del límite theta1
#define limitSwitch2 10   // Define el pin del interruptor del límite theta2
#define limitSwitch4 11   // Define el pin del interruptor del límite z
#define limitSwitch3 A3   // Define el pin del interruptor del límite phi

AccelStepper stepper[4] = {
  AccelStepper(1,2,5),    // Motor Articulacion theta1
  AccelStepper(1,3,6),    // Motor Articulacion theta2
  AccelStepper(1,12,13),  // Motor Articulacion z
  AccelStepper(1,4,7),    // Motor Articulacion phi
};

// Init functions
void homing();
float angleToStep(int motor, float angle);

// Variables
long stepper1Position;
long stepper2Position;
long stepper3Position;
long stepper4Position;

// Constantes
const int defaultVel = 500;
float toStep[4] = {
   44.44444,    // Constante de conversion del theta1
   35.55555,    // Constante de conversion del theta2
   25,          // Constante de conversion del z
  -10,          // Constante de conversion del phi
};

//ROS
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
ros::NodeHandle nh; // Node handler
geometry_msgs::Quaternion outputMessage;
ros::Publisher pub("/joint_pos", &outputMessage);

void callbackFcn(const geometry_msgs::Quaternion &inputMessage){
  // Every move from Ale's code
  //move(inputMessage);
  // TODO 2: Publish current position data to joint_pos
  
}

ros::Subscriber<geometry_msgs::Quaternion> sub("/desired_joint_pos", &callbackFcn);
void setup() {
  //* Hardware *
  // Límites de carrera
  pinMode(limitSwitch1, INPUT); // Limite de carrera límite theta1
  pinMode(limitSwitch2, INPUT); // Limite de carrera límite theta2
  pinMode(limitSwitch4, INPUT); // Limite de carrera límite z
  pinMode(limitSwitch3, INPUT); // Limite de carrera ímite phi

  // Configuraciones iniciales del motor
  stepper[0].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 1     
  stepper[0].setAcceleration(defaultVel);  // Establecer la aceleración del motor 1
  stepper[3].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 2
  stepper[3].setAcceleration(defaultVel);  // Establecer la aceleración del motor 2
  stepper[1].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 3
  stepper[1].setAcceleration(defaultVel);  // Establecer la aceleración del motor 3
  stepper[2].setMaxSpeed(defaultVel);      // Establecer la velocidad máxima del motor 4
  stepper[2].setAcceleration(defaultVel);  // Establecer la aceleración del motor 4

  //* ROS *
  nh.initNode(); // TODO opt: Change node name
  nh.advertise(pub);
  nh.subscribe(sub);

  // Call homing function

  homing(); // !! UNCOMMENT WHEN READY TO USE ROBOT


  // For debuggin
  //Serial.begin(9600);
}

void loop() {
  nh.spinOnce();
  delay(1000);
}
// TODO 3: limit the arms
void move(const geometry_msgs::Quaternion &inputMessage){

  // Extraer los valores de theta1, theta2, phi, z del comando
  float theta1 = inputMessage.x;
  float theta2 = inputMessage.y;
  float phi = inputMessage.w;
  float z = inputMessage.z;
  
  // Angle to Steps conversion
  float stepperPosition[4] = {
    stepper1Position = theta1 * toStep[0],
    stepper2Position = theta2 * toStep[1],
    stepper3Position = phi * toStep[2],
    stepper4Position = z * toStep[3],
  };

  // Set parameters for motors
  for(int i = 0; i<4; i++){
    // Articulacion i
    stepper[i].setAcceleration(0);
    stepper[i].setMaxSpeed(defaultVel);
    stepper[i].setSpeed(defaultVel);
    stepper[i].moveTo(stepperPosition[i]);
  }

    while (stepper[0].currentPosition() != stepper1Position) {
      if (digitalRead(limitSwitch1) == 0) {
        stepper[0].stop();
        break;
      }
      else {
        stepper[0].runToPosition();
      }
    }
    while (stepper[1].currentPosition() != stepper2Position) {
      if (digitalRead(limitSwitch2) == 0) {
        stepper[1].stop();
        break;
      }
      else {
        stepper[1].runToPosition();
      }
    }
    while (stepper[2].currentPosition() != stepper3Position) {
      if (digitalRead(limitSwitch3) == 0) {
        stepper[2].stop();
        break;
      }
      else {
        stepper[2].runToPosition();
      }
    }
    while (stepper[3].currentPosition() != stepper4Position) {
      if (digitalRead(limitSwitch4) == 0) {
        stepper[3].stop();
        break;
      }
      else {
        stepper[3].runToPosition();
      }
    }

  delay(500);
  }
  
  // Homing
void homing() {
  // // Homing Articulacion 4 z
  while (digitalRead(limitSwitch3) == 1) {
    stepper[3].setAcceleration(defaultVel);
    stepper[3].setSpeed(900);
    stepper[3].runSpeed();
    stepper[3].setCurrentPosition(1660);  // Para +-166 grados
  }
  delay(20);
  stepper[3].moveTo(0);
  while (stepper[3].currentPosition() != 0) {
    stepper[3].run();
  }
  // Homing Articulacion 3
  while (digitalRead(limitSwitch2) == 1) {
    stepper[1].setAcceleration(defaultVel);
    stepper[1].setSpeed(-500);
    stepper[1].runSpeed();
    //Serial.println(-angleToStep(1,90));
    stepper[1].setCurrentPosition(-angleToStep(1,158.5));
  }
  delay(20);
  stepper[1].moveTo(0);
  while (stepper[1].currentPosition() != 0) {
    stepper[1].run();
  }
  // Homing Articulacion 2
  while (digitalRead(limitSwitch4) == 1) {
    stepper[2].setAcceleration(defaultVel);
    stepper[2].setSpeed(-500);
    stepper[2].runSpeed();
    stepper[2].setCurrentPosition(-500); 
  }
  delay(20);
  stepper[2].moveTo(0);
  while (stepper[2].currentPosition() != 0) {
    stepper[2].run();
  }
  // Homing Articulacion 1
  while (digitalRead(limitSwitch1) == 1) {
    stepper[0].setAcceleration(defaultVel);
    stepper[0].setSpeed(-500);
    stepper[0].runSpeed();
    stepper[0].setCurrentPosition(-7222); // Para +-162 grados
  }
  delay(20);
  stepper[0].moveTo(0);
  while (stepper[0].currentPosition() != 0) {
    stepper[0].run();
  }
  delay(20);
}

float angleToStep( int motor, float angle){
  return toStep[motor] * angle;
}







