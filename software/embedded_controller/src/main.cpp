#include <Arduino.h>
#include <String.h>

hw_timer_t * timer = NULL;

// MOTORS
#define L_MOTOR 18            // left motor
#define L_MOTOR_DIRECTION 5 // rotation direction of left motor
#define R_MOTOR 34            // right motor
#define R_MOTOR_DIRECTION 35  // rotation direction of right motor

//ENCODERS
#define L_ENCODER_A 2   // left encoder
#define L_ENCODER_B 15 
#define R_ENCODER_A 27  // right encoder
#define R_ENCODER_B 26

#define L_ticks_per_mm 5.492  // number of ticks (points) read by the left encoder per mm travelled
#define R_ticks_per_mm 5.505 // same for right encoder

// Trigonometric direction rotation
#define L_ticks_per_rad 950.79 //  number of ticks (points) read by the left encoder processing a 1 radian rotation in trigo direction
#define R_ticks_per_rad 914.35 // by the right encoder

long L_encoder_ticks = 0;
long L_encoder_previous_ticks = 0;
long L_encoder_deltaTicks = 0;    // number of ticks measured by the left encoder after a short time

long R_encoder_ticks = 0;
long R_encoder_previous_ticks = 0;
long R_encoder_deltaTicks = 0;    // number of ticks measured by the right encoder after a short time

float distance_travelled_ticks = 0;  // distance in mm travelled by the robot after a short time
float angle_travelled_ticks = 0;    // angle travelled by the robot in ticks

float integral_error_distance_term = 0;
float integral_error_angle_term = 0;

float error_angle = 0;

float L_command;
float R_command;
float previous_L_command;
float previous_R_command;

int distance_setpoint_mm;
float distance_setpoint_ticks;
float distance_command_ticks;
float previous_distance_command_ticks;

int distance_then_rotation_setpoint_ticks[2];

float angle_setpoint_rad;
float angle_command_ticks;
float previous_angle_command_ticks;

int sample_time = 100;


// Maximum and minimum motors' speed allowed and max acceleration
const int MAX_SPEED = 75; // in ticks/sampletime
const int MAX_ACCELERATION = 5;
const int MAX_DECELERATION = 5;

float new_coef_deceleration;


bool closed_loop = true; // if true we apply a PID

// PID coefficients
// For distance
float P_V = 1;    // proportional coefficient
float I_V = 0.0;  // integrating coefficient
float D_V = 0.0;  // derivative coefficient

// For rotation

float P_W = 0.5;  // proportional coefficient
float I_W = 0.3;  ;  // integrating coefficient
float D_W = 0.0;  // derivative coefficient

float X_real = 0;
float Y_real = 0;
float Z_real = 0;
float V_real = 0;
float W_real = 0; // angular speed

int X_target = 0;
int Y_target = 0;
int Z_target = 0;

float X_theoretical;
float Y_theoretical;
float Z_theoretical;
float V_theoretical;
float W_theoretical;

float error_XY;
float error_Z;
float error_V;
float error_W;

int pid_v;
int pid_w;

unsigned long previousmillis = 0;  // Last measured time

char command[14];  // Store character by character the sent command with serial connection
int position_character = 0;

bool robot_moving = false;  // Indicate if a command has been sent
bool antiwindup = false;

float current_speed = 0;
float speed_target = 0;
float previous_speed_target = 0;
float left_speed_target;
float right_speed_target;
float left_speed_coef;
float right_speed_coef;

float acceleration_time;
float acceleration_distance;
float deceleration_time;
float deceleration_distance;
int constantspeed_time; // is int
float constantspeed_distance;
float maxspeed_reached;

enum Phase {STATIONARY, ACCELERATION, CONSTANT_SPEED, DECELERATION};

enum Trajectory {FORWARD_LINEAR, BACKWARD_LINEAR, LEFT_PIVOT, RIGHT_PIVOT};

int to_coordinates_trajectory = 0;

bool coordinates_target = false;

Trajectory trajectory_type = FORWARD_LINEAR;

Phase speed_phase = STATIONARY;

int target_ticks;

void setup() {
  //MOTORS
  pinMode(L_MOTOR, OUTPUT);
  pinMode(L_MOTOR_DIRECTION, OUTPUT);
  pinMode(R_MOTOR, OUTPUT);
  pinMode(R_MOTOR_DIRECTION, OUTPUT);

  //ENCODERS
  pinMode(L_ENCODER_A, INPUT_PULLUP);  // INPUT_PULLUP so if there is no signal on this pin, we will receive the value 1, so when we will reveive something it will be 0
  pinMode(L_ENCODER_B, INPUT_PULLUP);
  pinMode(R_ENCODER_A, INPUT_PULLUP);
  pinMode(R_ENCODER_B, INPUT_PULLUP);

  // ENCODERS Interruptions
  // The macro CHANGE triggers an interrupt when the pin changes, whether from LOW to HIGH or HIGH to LOW
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), L_Encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), R_Encoder_interrupt, CHANGE);

  //SERIAL COMMUNICATION
  Serial.begin(115200);  // speed of communication

  // Our integration and derivation in PID are done every sample_time seconds
  I_V = I_V * sample_time; // cause the integral coefficient is applied for each sample_time seconds. Indeed we integrate error*dt
  D_V = D_V / sample_time; // we derivate derror/dt 
  
  // TODO : replace millis with a timer !! 
  //uint8_t timer_id = 0; // Defines which timer will be used on the ESP32
  //uint16_t prescaler = 80; // Between 0 and 65 535 (timer division factor)
  //int threshold = 1000000; // 64 bits value (limited to int size of 32bits)

  //timer = timerBegin(timer_id, prescaler, true);
  //timerAttachInterrupt(timer, &timer_isr, true);
  //timerAlarmWrite(timer, threshold, true); // A timer is triggered when threshold is reached
  //timerAlarmEnable(timer);
}

float mm_to_ticks(float distance_mm){
  return distance_mm*(L_ticks_per_mm + R_ticks_per_mm)/2;
}

float ticks_to_mm(float distance_ticks){
  return distance_ticks / ((L_ticks_per_mm + R_ticks_per_mm)/2);
}

float rad_to_ticks(float angle_rad){
  return angle_rad * ((L_ticks_per_rad + R_ticks_per_rad)/2);
}

void motors_command(float L_command, float R_command) {  // L_command and R_command between -255 and 255 (if negative the wheel goes backward)
  if (L_command >= 0) {
    digitalWrite(L_MOTOR_DIRECTION, HIGH);
  } else {
    digitalWrite(L_MOTOR_DIRECTION, LOW);
  }
  if (R_command >= 0) {
    digitalWrite(R_MOTOR_DIRECTION, LOW);
  } else {
    digitalWrite(R_MOTOR_DIRECTION, HIGH);
  }
  analogWrite(L_MOTOR, abs(L_command));
  analogWrite(R_MOTOR, abs(R_command));
}

void L_Encoder_interrupt() {  // called when there is an interruption on the left encoder pin A

  // L_ENCODER_A and L_ENCODER_B are the same signal but are out of phase. We defined the interruption on the L_ENCODER_A pin
  if (digitalRead(L_ENCODER_A) == digitalRead(L_ENCODER_B)) {
    // So if the previous condition is true, it means that when the pin L_ENCODER_A detected a high value and interrupted the programm, the B pin was already detecting a high value
    // So the first signal to have a high value was the L_ENCODER_B, with our configuration it means the wheel is moving forward.
    L_encoder_ticks++;
  } else {  // else the wheel is moving backward
    L_encoder_ticks--;
  }

}

void R_Encoder_interrupt() {  // called when there is an interruption on one of the right encoder pin

  // same explanation as left interrupt function
  if (digitalRead(R_ENCODER_A) == digitalRead(R_ENCODER_B)) {  // the wheel is moving forward
    R_encoder_ticks++;
  } else {  // the wheel is moving backward
    R_encoder_ticks--;
  }
}

void readCommand(){
  if (Serial.available() > 0) {  // Si on a envoyé un charactère
    if (position_character == 0) {
      for(int i=0; i<5; i++){
        command[i] = 0;
      }
    }

    char character = Serial.read();

    if (character == '\n' || character == '\r') {  // Si on envoie la commande via le moniteur série en tappant entrée
      position_character = 0;                      // alors on remet à 0 l'indice auquel on ajouter le prochain caractère pour la prochaine commande
      integral_error_angle_term = 0;
      integral_error_distance_term = 0;

      parseCommand(command);
      return;
    }
    command[position_character++] = character;
  }
}

void parseCommand(char command[]) {
  // Analyze the String command to retrieve its parameter values.
  String commandString(command);

  if (commandString.startsWith("d=")) {  // Checks if the command starts with "d=" to indicate we want to send a distance
    // Extracts the substring next to the "=" sign
    String distanceSubString = commandString.substring(2);
    // Converts it to an int and defined it as the distance command
    distance_setpoint_mm = distanceSubString.toFloat();
    if (distance_setpoint_mm>=0){
      trajectory_type = FORWARD_LINEAR;
    }
    else{
      trajectory_type = BACKWARD_LINEAR;
    }
    distance_command_ticks = mm_to_ticks(distance_setpoint_mm);
    distance_setpoint_ticks = abs(distance_command_ticks);
    trajectory_order();
  }

  else if (commandString.startsWith("r=")) {  // Checks if the command starts with "r=" to indicate we want to send an angle in degree
    // Extracts the substring next to the "=" sign
    String angleSubString = commandString.substring(2);
    // Converts it to radians and defined it as the angle command
    angle_setpoint_rad = (angleSubString.toFloat() * PI) / 180;
    if (angle_setpoint_rad>=0){
      trajectory_type = LEFT_PIVOT;
    }
    else{
      trajectory_type = RIGHT_PIVOT;
    }
    distance_command_ticks = rad_to_ticks(angle_setpoint_rad);
    distance_setpoint_ticks = abs(distance_command_ticks);
    trajectory_order();
  }

  // Assign coordinates to the current location
  else if(commandString.startsWith("p=")) { // Checks if the command starts with "p=" to indicate we want to redefine the absolute position of the robot. Format: p=xxxx:yyyy:zzzz in millimeters
    // Extracts the substring next to the "=" sign
    X_real = commandString.substring(2, 6).toInt();
    Y_real = commandString.substring(7, 10).toInt();
    Z_real = commandString.substring(11, 14).toInt();
  }

  // Assign coordinates to the current location
  else if(commandString.startsWith("go=")) { // Checks if the command starts with "go=" to indicate we want to redefine the absolute position of the robot. Format: go=xxxx:yyyy:zzzz
    // Extracts the substring next to the "=" sign
    X_target = commandString.substring(3, 7).toInt();
    Y_target = commandString.substring(8, 12).toInt();
    Z_target = commandString.substring(13, 17).toInt();
    float X_distance = mm_to_ticks(X_target)-X_real;
    float Y_distance = mm_to_ticks(Y_target)-Y_real;
    distance_command_ticks = sqrt( X_distance*X_distance + Y_distance*Y_distance );
    distance_then_rotation_setpoint_ticks[1] = distance_command_ticks;
    Z_target = (Z_target*PI) / 180;
    distance_then_rotation_setpoint_ticks[0] = rad_to_ticks(Z_target) - Z_real;
    coordinates_target = true;
    trajectory_order();
  }

  else if(commandString.startsWith("P_V=")){
    P_V = commandString.substring(2).toFloat();
  }

  else if(commandString.startsWith("I_V=")){
    I_V = commandString.substring(2).toFloat() * sample_time;
  }

  else if(commandString.startsWith("D_V=")){
    D_V = commandString.substring(2).toFloat() / sample_time;
  }

  else if(commandString.startsWith("PR=")){
    P_W = commandString.substring(2).toFloat();
  }

  else if(commandString.startsWith("IR=")){
    I_W = commandString.substring(2).toFloat() * sample_time;
  }

  else if(commandString.startsWith("DR=")){
    D_W = commandString.substring(2).toFloat() / sample_time;
  }

  else if(commandString.startsWith("sampling=")){
    float new_sample_time = commandString.substring(2).toFloat();
    float ratio =  new_sample_time / sample_time;
    I_V *= ratio;
    D_V /= ratio;
    sample_time = new_sample_time;
  }

  else if(commandString.startsWith("x=")) { // Checks of the command starts with "x=" to indicate we want to define the absolute position in x of the robot.
    X_real = commandString.substring(2,5).toInt();
  }

  else if(commandString.startsWith("y=")) { 
    Y_real = commandString.substring(2,5).toInt();
  }

  else if(commandString.startsWith("z=")) { 
    Z_real = commandString.substring(2,5).toInt();
  }

  else if (commandString.startsWith("stop")) {  //
    motors_command(0, 0);
  }

  else if (commandString.startsWith("print")){
    Serial.println(L_encoder_ticks);
    Serial.println(R_encoder_ticks);

    Serial.print(X_theoretical);
    Serial.print(":");
    Serial.print(Y_theoretical);
    Serial.print(":");
    Serial.println(Z_theoretical);

    Serial.print(X_real);
    Serial.print(":");
    Serial.print(Y_real);
    Serial.print(":");
    Serial.println(Z_real);

    Serial.println("distance and angle commands");
    
    Serial.print("distance command mm = ");
    Serial.println(distance_command_ticks);
    Serial.print("angle command rad = ");
    Serial.println(angle_command_ticks*180/PI);
  }

  else if (commandString.startsWith("reset")) {  // Useful when the robot wedges itself against a border to calibrate.
    distance_command_ticks = 0; // We reset the commands 
    angle_command_ticks = 0;
    L_encoder_ticks = 0;  // We reset the count of encoders ticks to 0.
    R_encoder_ticks = 0;
  }
}

void trajectory_order(){
  if (coordinates_target){
    distance_command_ticks = distance_then_rotation_setpoint_ticks[to_coordinates_trajectory];
    distance_setpoint_ticks = abs(distance_command_ticks);
    if (to_coordinates_trajectory==0){
      trajectory_type = LEFT_PIVOT;
      to_coordinates_trajectory++;
    }
    else{
      trajectory_type = FORWARD_LINEAR;
      to_coordinates_trajectory = 0;
      coordinates_target = false;
    }
  }
  speed_phase = ACCELERATION;
  get_phases_distances();
}

void get_phases_distances(){
  // used to determine acceleration distance and deceleration distance
  acceleration_time = floor(MAX_SPEED / MAX_ACCELERATION);
  acceleration_distance = (MAX_ACCELERATION * acceleration_time * acceleration_time)/2; // kinematic equation for uniformly accelerated motion : Displacement=InitialPosition+ 1/2 ​×Acceleration×(Time)² 
  deceleration_time = floor(MAX_SPEED / MAX_DECELERATION);
  deceleration_distance = (MAX_DECELERATION * deceleration_time * deceleration_time)/2;
  
  if (acceleration_distance + deceleration_distance > distance_setpoint_ticks){ // Triangle speed profile, we don't have the time to have a constant speed phase
    constantspeed_distance = 0;
    constantspeed_time = 0;
    deceleration_time = floor(sqrtf( (2*distance_setpoint_ticks*MAX_ACCELERATION) / ( (MAX_DECELERATION)*(MAX_ACCELERATION+MAX_DECELERATION) ) )); 
    deceleration_distance = (MAX_DECELERATION * deceleration_time * deceleration_time)/2;
    acceleration_distance = distance_setpoint_ticks - deceleration_distance;
    maxspeed_reached = deceleration_time * MAX_DECELERATION;
  }
  else{
    maxspeed_reached = MAX_SPEED;
    constantspeed_distance = distance_setpoint_ticks - acceleration_distance - deceleration_distance;
    constantspeed_time =  floor(constantspeed_distance / maxspeed_reached) ;
  }

  // Now we adjust the deceleration coefficient to modify the deceleration slope (as we can't be more precise when calculating deceleration time due to roundings)

  // The formula for Uniformly accelerated rectilinear motion is accceleration= vf²-vi² / 2(xf-xi), f means final, i means initial.
  // here vf = 0, so accceleration= -vi² / 2(xf-xi), we don't consider the minus sign as we are searching for the absolute value of the coefficient (like MAX_DECELERATION which was positive)
  // xf is the final position and xi the position at the beginning of deceleration phase so xf - xi = deceleration_distance
  new_coef_deceleration = maxspeed_reached*maxspeed_reached / (2 * (distance_setpoint_ticks - maxspeed_reached * constantspeed_time - acceleration_distance));
  deceleration_time = ceil(maxspeed_reached/new_coef_deceleration); // ceil() cause it's better to have a greater slope than going beyond the value
  deceleration_distance = (new_coef_deceleration*deceleration_time*deceleration_time)/2; //
  
}

void get_speed_target(){
  previous_speed_target = speed_target;
  target_ticks++;
  if (speed_phase==ACCELERATION){
    if ( target_ticks >= acceleration_time){

      if (constantspeed_time==0){
        speed_phase = DECELERATION;
      }
      else{
        speed_phase = CONSTANT_SPEED;
        
      }
      target_ticks = 0;
    }
    else{
      speed_target += MAX_ACCELERATION;
    }
  }
  else if (speed_phase==CONSTANT_SPEED){
    if ( target_ticks >= constantspeed_time){
      speed_phase = DECELERATION;
      target_ticks = 0;
    }

  }
  else if (speed_phase==DECELERATION){
    if ( target_ticks >= deceleration_time){
      speed_target = 0;
      speed_phase = STATIONARY;
      target_ticks = 0;
      V_theoretical = 0;
      W_theoretical = 0;

      if (coordinates_target){
        trajectory_order();
      }
    }
    else{
      speed_target -= new_coef_deceleration; //We substract the adjusted new coefficient not the maximal one
    }
  }

  switch(trajectory_type){
    case BACKWARD_LINEAR:
      left_speed_coef = -1;
      right_speed_coef = -1;
      break;
    case LEFT_PIVOT:
      left_speed_coef = -1;
      right_speed_coef = 1;
      break;
    case RIGHT_PIVOT:
      left_speed_coef = 1;
      right_speed_coef = -1;
      break;
    default:
      left_speed_coef = 1;
      right_speed_coef = 1;
      break;
  }

  left_speed_target = speed_target * left_speed_coef;
  right_speed_target = speed_target * right_speed_coef;
}

void get_theoretical_speed_and_position(){
  get_speed_target();
  V_theoretical = left_speed_target + right_speed_target; // we don't divide by two cause we compare angles so it will be useless to do the same operation on the two compared quantities to evaluate the difference
  W_theoretical = right_speed_target - left_speed_target;
  
  Z_theoretical += W_theoretical; // différence de vitesse
  X_theoretical += V_theoretical * cos(Z_theoretical);
  Y_theoretical += V_theoretical * sin(Z_theoretical);
}

void get_real_speed_and_position(){
  V_real = distance_travelled_ticks;
  W_real = angle_travelled_ticks;
  Z_real += W_real;
  X_real += V_real * cos(Z_real); 
  Y_real += V_real * sin(Z_real); 
}

void get_errors(){
  get_theoretical_speed_and_position();
  get_real_speed_and_position();
  float error_distance = sqrt( pow(X_theoretical - X_real, 2) + pow(Y_theoretical - Y_real,2));
  float error_angle = 0;
  if (X_theoretical != X_real || Y_theoretical!=Y_real){
    error_angle = atan2(Y_theoretical - Y_real, X_theoretical - X_real) - Z_real;
  }
  error_XY = error_distance * cos(error_angle);
  error_V = V_theoretical - V_real;
  
  error_Z = Z_theoretical - Z_real;
  error_W = W_theoretical - W_real;
}

void correct_distance() {
  get_errors();

  if (antiwindup){ // then we only apply a proportional correction
    pid_v = (int)(P_V * error_V);
    pid_w = (int) (P_W * error_W);
  }
  else{
    pid_v = (int) (P_V * error_V +  I_V * error_XY  ); //  + D_V*(speed_target-previous_speed_target)
    pid_w = (int) (P_W * error_W +  I_W * error_Z  );
  }
  
  L_command = pid_v - pid_w;
  R_command = pid_v + pid_w;

  previous_L_command = L_command;
  previous_R_command = R_command;

  motors_command(L_command, R_command);  // we send the command to the motors
}

void loop() {

  readCommand(); // vérifier qu'on est pas bloqué, ajout de buzzer, timer

  if (millis() - previousmillis > sample_time) {  // we measure the number of ticks every sample_time milliseconds while et read command 
    previousmillis = millis();
  
    L_encoder_deltaTicks = (float)L_encoder_ticks - L_encoder_previous_ticks; // éviter les interruptions pendant le calcul.
    R_encoder_deltaTicks = (float)R_encoder_ticks - R_encoder_previous_ticks; // si while(temps>sample_time) 
    L_encoder_previous_ticks = L_encoder_ticks;
    R_encoder_previous_ticks = R_encoder_ticks;


    distance_travelled_ticks = (L_encoder_deltaTicks + R_encoder_deltaTicks);  // Distance travelled = average of distance travelled by each coding wheel or not divided by two if compared with theoretical
    current_speed = ticks_to_mm(distance_travelled_ticks) / (sample_time * 0.001); // in mm per sec

    angle_travelled_ticks = R_encoder_deltaTicks - L_encoder_deltaTicks; // we don't divide by two cause compared with theoretical angle which is not divided by two

    previous_distance_command_ticks = distance_command_ticks;
    previous_angle_command_ticks = angle_command_ticks;
    distance_command_ticks -= distance_travelled_ticks;  // Remaining distance to achieve (error)
    angle_command_ticks -= angle_travelled_ticks;      // Remaining angle to achieve (error)

    String commandString(command);
    if ( (commandString.startsWith("d=") || commandString.startsWith("r=") || commandString.startsWith("go=")) && closed_loop) {
      correct_distance();
    }

  }
}
