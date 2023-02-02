
/***************************************************************************
  This program is for Bone-In: Honey BBQ flight computer, Created by Zachary Ervin

Published to Git

................................................................................................................................................
READ ME
................................................................................................................................................

  SETUP:
  
  Set Current Location and Time Sea Level Pressure:
    Get current sea level pressure from: https://weather.us/observations/pressure-qnh.html
    Save under variables, Defined Constant "SEALEVELPRESSURE_HPA" 
  
  Sea Level vs ground reference setup:
    under barometer setup code, uncomment section 1 and comment section 2 for ground reference data. Uncomment section 2 and comment section 1 for ground reference data. 

  Target altitude setup:
    Under variables, change "TARGET_ALTITUDE" variable to the desired target altitude ABOVE THE GROUND.

  Gravity constant setup:
    Under variables, adjust "G" variable to accoring to previous flight data to obtain the most accurate apogee projection.



  MODE INDICATORS:
  
  ON POWER UP
  -LED blinks and buzzer beeps 3 times
  
  ARMED AND READY TO LAUNCH
  -Solid LED and Slow Beeps

  RECOVERY MODE
  -Slow long Beeps
  

  
  TROUBLESHOOTING:
  -SD Card error:       LED blinks and Buzzer beeps fast 
  -FILE error:          LED and Buzzer stay on
  -Accelerometer error: LED stays on, NO BUZZER
  -Barometer error:     Buzzer stays on, NO LED

................................................................................................................................................

  
 ***************************************************************************/


 
//LIBRARIES

#include <Wire.h>
//#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <MPU6050_light.h> // for accelerometer
#include <Adafruit_MPU6050.h>

#include <SD.h>
#include <Servo.h>



//VARIABLES

  int counter_mco = 0; //COUNTER for Main Engine Cutoff detection
  int counter_apogee = 0; //COUNTER for Apogee detection
  int counter_landed = 0; //COUNTER for Landed Detection
  
  float x_previous = 0; // previous position
  float x_current = 0; //current position
  float vel = 0; //velocity
  float acc = 0; //acceleration
  float acc_avg = 0; //acceleration variable for averaging data
  //float apo = 0; //predicted apogee
  float init_pressure = 0; //ground pressure reading in HPa
  float init_altitude = 0; //ground altitude reading in meters
  float delta_t = 0; //time between iterations in milliseconds
  float target_acc = 0; //acceleration needed to hit target altitude
  float distance_to_target = 0; //distance to target apogee
  float brake_position = 0; //position to set brakes to (0 for closed, 100 for open full)

  //PID values:
  float P_gain = 1; //gain value for p controller
  
  
  unsigned long t_previous = 0;  //previous clock time in milliseconds
  unsigned long t_current = 0;  //current clock time in milliseconds
  
  //brakes servo RANGE and Velocity Limit
  #define brakes_closed  625
  #define brakes_open  1580
  #define max_brake_velocity 500 //must be under this velocity in [m/s] to deploy brakes
  
  //DEFINE PIN NUMBERS
  #define BUZZER_PIN  15
  #define LED_PIN  5
  #define SERVO_PIN  3



  
//**************SET TARGET ALTITUDE HERE*************************
  #define TARGET_ALTITUDE (100) //target altitude above ground in meters

// *********SET SEA LEVEL PRESSURE HERE***************
  #define SEALEVELPRESSURE_HPA (1016.00)//set according to location and date
  
// *********SET FREQUENCY HERE IN HZ***************
  #define hz (5)//hz

// *********SET GRAVITY CONSTANT HERE***************
  #define G (9.81)// m/(s^2)

// *********SET TAKEOFF ALTITUDE HERE***************
  #define TAKEOFF_ALTITUDE  (30) //altitude above the ground in meters that triggers the detect_take_off function

// *********SET ACCELERATION OFFSET CONSTANT HERE***************
  #define OFFSET (-7)// offset for acceleration data in m/(s^2) for prediction algorithm




//Brakes Servo Handle
  Servo brakes; 

// accelerometer attachment
  MPU6050 mpu(Wire); //BMP_SCK 13 BMP_MISO 12 BMP_MOSI 11 BMP_CS 10

// barometer attachment
Adafruit_BMP3XX bmp;

// File for logging
File logFile;




//////SETUP//////////SETUP//////////////SETUP///////////////SETUP/////////////SETUP/////////////SETUP//////////SETUP//////////
// Setup Function (runs once)

void setup() {
  //Begin serial comunication
  Serial.begin(9600);
  while (!Serial);

  //Light setup
  pinMode(LED_PIN, OUTPUT);

  //BUZZER setup:
  pinMode(BUZZER_PIN, OUTPUT);
  

  //Servo setup************************************************
  
  brakes.attach(SERVO_PIN);  // attaches the servo on pin to the servo object

  /*
  //open & close brakes three times to identify issues
  open_brakes();
  delay(500);
  close_brakes();
  delay(500);
  open_brakes();
  delay(500);
  close_brakes();
  delay(500);
  open_brakes();
  delay(500);
  */
  
  close_brakes();
  
  
  //SD Card Setup***********************************************

  if (!SD.begin(10)) {
    while(!SD.begin()){
      //blink/buzz fast to indicate error
      turn_on_led(); //Turn on LED
      turn_on_buzzer(); //Turn on buzzer
      delay(200);
      turn_off_led(); //Turn off LED
      turn_off_buzzer(); //Turn off buzzer
      delay(100);
    }
  }


  //FILE Setup***************************************************

  open_file(); //File for writing
    if (logFile) {
    //file opened ok
  } else {
    // if the file didn't open, turn on buzzer/LED to indicate file problem
    turn_on_led(); //Turn on LED
    turn_on_buzzer(); //Turn on buzzer
    while(1){ 
      //run while loop forever
    }
  }


// Accelerometer sensor set up********************************************
  
  byte status = mpu.begin(1,3);
  while(status!=0){ 
    //turn LED on if could not connect to accelerometer
    turn_on_led();
    }
    turn_off_led();//turn off LED
    
  
  //Calculating offsets, do not move MPU6050
  delay(2000);//pause two seconds to give time to stabalize accelerometer
  mpu.calcOffsets(true,true); // gyro and accelerometer calibration


// Barometer setup********************************************************

  while (!bmp.begin_I2C()) {   
    //turn on buzzer if not connecting to barometer
    turn_on_buzzer();
  }
  turn_off_buzzer();

  // Set up oversampling and filter initialization settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);//no filtering
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


for(int i = 0; i<10; i++){ //calibrates initial pressure and starting altitude to zero
  read_barometer();//updates barometer data
  delay(50);

  //calibrate readings for altitude measurements.
  init_pressure = bmp.pressure/100; //used to call read altitude for ground level reference
  init_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  x_previous = bmp.readAltitude(init_pressure);
}

  
  set_header_file(); //sets headers at beginning of file


  delay(1000);
  x_current = bmp.readAltitude(init_pressure);

  if (abs(x_current - x_previous) > 3 ){//enters if roket is moving greater than 3 m/s
    logFile.print(F("ABORTED, ROCKET MOVING"));
    beep_buzz(1); //beeps and lights once to signal that rocket was detected moving
    while (1){//runs forever
      //update altitude at frequency (hz)
      average_acceleration_data(); // updates acceleration value acc, and delta_t value
      read_velocity();//reads new altitude as x_current and compares to previous altitude x_previous and delta_t to determine velocity, then updates x_previous as x_current
      log_data();//logs data to sd card
      reopen_file();//saves and reopens file
      
    }
  }

  
  
  //print initial sea level altitude to file
  logFile.print(init_altitude);

  //sets initial time, t_previous
  t_previous = millis();
  
  //LED light On Solid to show ARMED. Turns off when motor ignition detected. 
  beep_buzz(3); //beeps and blinks LED 3 times to indicate armed

  
  //COMPUTER IS NOW ARMED
}





/////////MAIN PROGRAM///////////////MAIN PROGRAM//////////////MAIN PROGRAM///////////////MAIN PROGRAM////////////////MAIN PROGRAM////////////////MAIN PROGRAM///////////////MAIN PROGRAM/////////
//BEGIN LOOP MAIN PROGRAM

void loop() { 


//STANDBY MODE********************
  while(!detect_take_off()){//Waiting for launch. Exits if altitude is above takeoff altitude
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x_current and compares to previous altitude x_previous and delta_t to determine velocity, the updates x_previous as x_current
  }

  log_data();//logs all data to sd card


//MOTOR BURNING MODE********************
  while(!detect_mco()){//runs until MCO is detected
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x_current and compares to previous altitude x_previous and delta_t to determine velocity, then updates x_previous as x_current
    log_data();//logs all data to sd card
  }
  
  
  if (logFile) {
    logFile.print(F("MCO Detected")); //logs event
    reopen_file();//saves and reopens file
  }



//ASCENDING MODE********************

  open_brakes();//apply full brakes
  logFile.print(F(" FULL BRAKE"));

  
  while(! detect_apogee()){//Runs until apogee is detected.
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x_current and compares to previous altitude x_previous and delta_t to determine velocity, then updates x_previous as x_current
  
    if(vel>0 && acc < 0){// enters if traveling up and decelerating 
      predict_apogee();//predicts apogee
    }
    
    log_data();//logs all data to sd card
  }

  if (logFile) {
    logFile.print(F("Apogee Detected"));
    reopen_file();//saves and reopens file once apogee is detected
  }




//DESCENDING MODE********************

  close_brakes();//fully closes brakes
  if (logFile) {
    logFile.print(F(" CLOSE BRAKE"));
  }
  
  while(! detect_touchdown()){//Runs until touchdown is detected
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x_current and compares to previous altitude x_previous and delta_t to determine velocity, then updates x_previous as x_current
    log_data();//logs all data to sd card
  }
  logFile.println(F("Landing Detected"));



//RECOVERY MODE********************
  close_file(); //closes the file
  recovery_beeps(); //enters recovery mode

}//end of main program









//******FUNCTIONS********FUNCTIONS********** FUNCTIONS **********FUNCTIONS*********FUNCTIONS**********FUNCTIONS*************

//***********LED FUNCTIONS***********

void turn_on_led(){ // Turns on LED
  digitalWrite(LED_PIN, HIGH);
}

void turn_off_led(){ // Turns off LED
  digitalWrite(LED_PIN, LOW);
}


//***********BUZZER FUNCTIONS***********

void turn_on_buzzer(){ // Turns on LED
  digitalWrite(BUZZER_PIN, HIGH);
}

void turn_off_buzzer(){ // Turns off LED
  digitalWrite(BUZZER_PIN, LOW);
}

void beep_buzz(int num){ // Blinks LED and Beeps Buzzer for num times
  for(int i = 0; i < num; i++){
    turn_on_buzzer();
    turn_on_led();
    delay(300);
    turn_off_buzzer();
    turn_off_led();
    delay(300);
  }
}

void recovery_beeps(){//Continuous Slow long beeps for recovery
  while(1){ //does not exit loop
  turn_on_buzzer();
  delay(1000); //beep delay
  turn_off_buzzer();
  delay(5000); //pause delay
  }
}


//***********BAROMETER FUNCTIONS***********

void read_barometer(){//takes barometer reading
  while (! bmp.performReading()) {//performs barometer reading
    //could not perform reading
  }
}

float read_altitude(){//reads and returns barometer altitude reading
  float altitudeReading;
  do {
    read_barometer(); 
    altitudeReading = bmp.readAltitude(init_pressure);
  } while (abs(altitudeReading - x_current) > 1200*(t_current - t_previous)); 
  //keeps reading altimiter if vertical speed is more than 1200 m/s (should never actually be that fast).
  return altitudeReading; // returns current altitude reading
}


//***********BRAKE FUNCTIONS***********

void open_brakes(){//opens brakes fully
  brakes.write(brakes_open);
}

void close_brakes(){//opens brakes fully
  brakes.write(brakes_closed);
}

void set_brakes(){//set brakes to desired brake position between 0 and 100 (0 = closed, 100 = open) 
  brakes.write((int)((brake_position/100)*(brakes_open - brakes_closed) + brakes_closed));
}


//***********FILE FUNCTIONS***********

void open_file(){// opens the file for writing
  logFile = SD.open("flight.txt", FILE_WRITE);
}

void set_header_file(){//sets headers at begining of file
  logFile.print(F("Flight Log:\t"));
  logFile.print(hz);
  logFile.println(F("hz"));
  logFile.print(F("Time:\tAlt:\tVel:\tAcc:\tApo:\tEvents:\t"));
}

void close_file(){
  logFile.close(); //closes file
}

void reopen_file(){//closes then reopens the file, saving data up to this point
  if (logFile){
  close_file();//closes the file
  }
  open_file();//opens the file
}

void log_data(){//saves current data to sd card
  if (logFile) {
    logFile.print("\n");
    logFile.print(t_current); logFile.print(F("\t"));       //logs current time
    logFile.print(x_current); logFile.print(F("\t"));       //logs current position
    logFile.print(vel); logFile.print(F("\t"));      //logs current velocity
    logFile.print(acc); logFile.print(F("\t"));      //logs current acceleration
    logFile.print(target_acc); logFile.print(F("\t"));      //logs larget acceleration apogee projection
    logFile.print(brake_position); logFile.print(F("\t"));      //logs larget acceleration apogee projection
  }
}



//***********ACCELEROMETER FUNCTIONS***********

void read_accelerometer(){//reads and updates z_global variable
 /*
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 */
  mpu.update(); //updates accelerometer data
  acc = 9.81 *  -(1+mpu.getAccX()*sin(mpu.getAngleY()*3.14/180)-mpu.getAccY()*cos(mpu.getAngleY()*3.14/180)*sin(mpu.getAngleX()*3.14/180)-mpu.getAccZ()*cos(mpu.getAngleY()*3.14/180)*cos(mpu.getAngleX()*3.14/180));// uses a transformation matrix to determine global z acceleration
}

void average_acceleration_data(){//averages acceleration data over the set frequency and saves to z_global variable. Also updates t_current time variable.
  int i = 0; //iteration counting variable
  acc_avg = 0; // resets z_avg variable to 0
  
  do{ //run at least once
    t_current = millis();        // updates t_current variable with current time
    read_accelerometer(); //updates z_global variable
    acc_avg += acc;    //adds all the acceleration readings for each iteration
    i++;                  //counts the number of iterations
  }while(t_current-t_previous < 1000/hz-1);//exits loop once elapsed time is greater than frequency time period

  acc = acc_avg/i;     //averages the z acceleration data for the period
  delta_t = t_current-t_previous;        //update change in time variable
  t_previous = t_current;                //updates the t_previous point for when the loop ended
}


//***********PHYSICS FUNCTIONS***********

void read_velocity(){//reads new altitude as x_current and compares to previous altitude x_previous and delta_t to determine velocity
  x_current = read_altitude(); //gets current altitude
  vel = (x_current - x_previous)/(delta_t / 1000); //determines velocity
  x_previous = x_current; //update previous altitude variable
  distance_to_target = TARGET_ALTITUDE - x_current; //update distance to target. Negative value implies target has been passed.
}

void predict_apogee(){//predicts apogee with current velocity, altitude, and acceleration and sets brakes to reach target altitude
  //apo = - sq(vel)/(2*(acc + OFFSET + G)) * log(- (acc + OFFSET) / G) + x_current;

  if (distance_to_target > 0) {//updates target acceleration if target is not reached
    target_acc = (sq(vel) * lambertW(- (2 * distance_to_target * G * exp(- (2 * distance_to_target * G)/sq(vel) ) )/sq(vel)) - 2 * distance_to_target * OFFSET)/(2 * distance_to_target);
  } else {
    target_acc = -100; //max deceleration because target is passed
  }
  
  brake_position = brake_position + P_gain * (acc - target_acc);

  if(brake_position > 100){
    brake_position = 100;
  } else if(brake_position < 0){
    brake_position = 0;
  }
  
  set_brakes();
}


float lambertW(float input){//returns the value of the lambert W function for the given input
  
  if (input > -0.367879){
    float v = 2000000000; //represents infinity
    float w = -2;
    float error;
    while (abs(w - v)/abs(w) > 0.00001){
      v = w;
      error = w*exp(w) - input;  // Iterate to make this quantity zero
      w = w - error/((exp(w)*(w+1) - (w+2)*error/(2*w+2)));
    }
    return w;
  } else {
    return -1;
  }
}



int detect_take_off(){//returns 1 if take off is detected, otherwise 0
  if (x_current >= TAKEOFF_ALTITUDE){
    return 1;
  } else {
    return 0;
  }
}


int detect_mco(){//returns 1 if MCO is detected, 0 otherwise.
  if(acc < 0){//enters if acceleration is less than 0
    counter_mco++;      //increments counter
  }
  else{
    counter_mco = 0;      //resets counter if positive acceleration
  }
  if(counter_mco > 3){    //enters if last 3 acceleration datas are less than 0 
    return 1;   //returns 1 for MCO detected
  }
  else{           
    return 0;   //returns 0 for MCO not detected
  }
}


int detect_apogee(){//looks for apogee and returns 1 if detected, 0 if not.
  if(x_current > x_previous){//enters if current position is higher than saved apogee value
    counter_apogee = 0;      //resets apogee counter
  }
  else{
    counter_apogee++;        //increments apogee counter if latest value is less than recorded apogee
  }
  if(counter_apogee > 3){    //enters if last 3 positions are lower than recorded apogee
    return 1;   //returns 1 for apogee detected
  }
  else{           
    return 0;   //returns 0 for apogee not reached
  }
}


int detect_touchdown(){//returns 1 if touchdown is detected, otherwise returns 0.
  if (vel < 1 && vel > -1){ //enters if velocity is almost 0
    counter_landed++;    //increments counter
  }
  else{
    counter_landed = 0;  //resets counter
  }
  if(counter_landed > 5){  //enters if counted 5 velocities in a row close to 0
    counter_landed = 0;    //resets counter
    return 1; //returns 1 if touchdown detected
  }
  else{
    return 0;  //returns 0 if touchdown not detected
  }
}
