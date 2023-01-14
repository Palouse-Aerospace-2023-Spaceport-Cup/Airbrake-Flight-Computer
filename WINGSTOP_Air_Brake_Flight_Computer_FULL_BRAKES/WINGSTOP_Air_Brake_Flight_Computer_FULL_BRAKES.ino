
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
    Under variables, change "target_altitude" variable to the desired target altitude ABOVE THE GROUND.

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

  int c = 0; //COUNTER
  float x1 = 0, x2 = 0, vel = 0, acc = 0, acc_avg = 0, apo = 0, apo_act = 0, init_pressure = 0, init_altitude = 0, delta_t = 0; //DATA VARIABLES (decimal)
  unsigned long t1 = 0, t2 = 0; //TIMER VARIABLES (integers)
  
  //brakes servo RANGE
  #define brakes_closed  625
  #define brakes_open  1580
  
  //DEFINE PIN NUMBERS
  #define BUZZER_PIN  15
  #define LED_PIN  5
  #define SERVO_PIN  3
  
//**************SET TARGET ALTITUDE HERE*************************
  float target_altitude = 100; //target altitude above ground in meters

// *********SET SEA LEVEL PRESSURE HERE***************
  #define SEALEVELPRESSURE_HPA (1016.00)//set according to location and date
  
// *********SET FREQUENCY HERE IN HZ***************
  #define hz (5)//hz

// *********SET GRAVITY CONSTANT HERE***************
  #define G (6.3)// m/(s^2)

//Brakes Servo Handle
  Servo brakes; 

// accelerometer attachment
  MPU6050 mpu(Wire); //BMP_SCK 13 BMP_MISO 12 BMP_MOSI 11 BMP_CS 10

// barometer attachment
Adafruit_BMP3XX bmp;

// File for logging
File myFile;




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
  beep_buzz(1);//beep and buzz 
  

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
  set_header_file(); //sets headers at beginning of file


// Accelerometer sensor set up********************************************
  
  byte status = mpu.begin(1,3);
  while(status!=0){ 
    //turn LED on if could not connect to accelerometer
    turn_on_led();
    }
    turn_off_led();//turn off LED
    
  
  //Calculating offsets, do not move MPU6050
  delay(2000);//pause one second to give time to stabalize accelerometer
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


for(int i = 1; i<10; i++){ //calibrates initial pressure and starting altitude to zero
  read_barometer();//updates barometer data
  delay(50);

  //calibrate readings for altitude measurements. CHOOSE SEALEVEL OR GROUND REFERENCE ALTITUDE READINGS. One section must be commented.

  ///* SECTION 1
  // COMMENT this section to use all sea level data, not ground reference data. Must uncomment second section. 
  init_pressure = bmp.pressure/100;
  init_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  x1 = read_altitude();
  //*///END SECTION 1 
}
  /* SECTION 2
  // UNCOMMENT this section to use all sea level data. Target altitude will be updated using initial altitude reading and target above ground. Must comment first section. 
  init_pressure = SEALEVELPRESSURE_HPA;
  init_altitude = read_altitude();
  x1 = init_altitude;
  target_altitude += init_altitude;
  *///END SECTION 2 

  
  //print initial sea level altitude to file
  myFile.print(init_altitude);

  //sets initial time, t1
  t1 = millis();
  
  //LED light On Solid to show ARMED. Turns off when motor ignition detected. 
  turn_on_led(); //Turn on LED

  //COMPUTER IS NOW ARMED
}





/////////MAIN PROGRAM///////////////MAIN PROGRAM//////////////MAIN PROGRAM///////////////MAIN PROGRAM////////////////MAIN PROGRAM////////////////MAIN PROGRAM///////////////MAIN PROGRAM/////////
//BEGIN LOOP MAIN PROGRAM

void loop() { 


//STANDBY MODE********************
  while(vel < 4){//Waiting for launch. Exits if velocity is greater than 4 m/s.
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x2 and compares to previous altitude x1 and delta_t to determine velocity, the updates x1 as x2
    if(c % (5*hz) == 0){//turns on buzzer every 5 seconds
      turn_on_buzzer();
    }
    if((c - (hz/4)) % (5*hz) == 0){//turns off buzzer after 1/4 of a second
      turn_off_buzzer();
    }
    c++;//increment counter for buzzer
  }

  c = 0; //reset counter for MCO detection
  turn_off_led();
  turn_off_buzzer();
  log_data();//logs all data to sd card
  myFile.print(F("Liftoff Detected"));


//MOTOR BURNING MODE********************
  while(! MCO()){//runs until MCO is detected
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x2 and compares to previous altitude x1 and delta_t to determine velocity, then updates x1 as x2
    log_data();//logs all data to sd card
  }
  
  myFile.print(F("MCO Detected"));
  reopen_file();//saves and reopens file once apogee is detected



//ASCENDING MODE********************


  open_brakes();//apply full brake
  myFile.print(F(" FULL BRAKE"));

  
  while(! detect_apogee()){//Runs until apogee is detected.
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x2 and compares to previous altitude x1 and delta_t to determine velocity, then updates x1 as x2
  
    if(vel>0 && acc < 0){// enters if traveling up and decelerating 
      predict_apogee();//predicts apogee
    }
    
    log_data();//logs all data to sd card
  }
  myFile.print(F("Apogee Detected"));
  reopen_file();//saves and reopens file once apogee is detected



//DESCENDING MODE********************



  close_brakes();//apply full brake
  myFile.print(F(" CLOSE BRAKE"));

  
  while(! detect_touchdown()){//Runs until touchdown is detected
    average_acceleration_data(); // updates acceleration value acc, and delta_t value
    read_velocity();//reads new altitude as x2 and compares to previous altitude x1 and delta_t to determine velocity, then updates x1 as x2
    log_data();//logs all data to sd card
  }
  myFile.println(F("Landing Detected"));



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
  while (! bmp.performReading()) {
    //performs until it gets a valid reading
  }
  return bmp.readAltitude(init_pressure); // returns current altitude reading
}


//***********BRAKE FUNCTIONS***********

void open_brakes(){//opens brakes fully
  brakes.write(brakes_open);//600 to 1580 for full range
}

void close_brakes(){//opens brakes fully
  brakes.write(brakes_closed);//600 to 1580 for full range
}

void set_brakes(float pos){//set brakes to any position between 0 and 1 (0 = closed, 1 = open) 
  brakes.write((int)(pos*(brakes_open - brakes_closed) + brakes_closed));//600 to 1580 for full range
}


//***********FILE FUNCTIONS***********

void open_file(){// opens the file for writing
  myFile = SD.open("flight.txt", FILE_WRITE);

  
  if (myFile) {
    //file opened ok
  } else {
    // if the file didn't open, turn on buzzer/LED to indicate file problem
    turn_on_led(); //Turn on LED
    turn_on_buzzer(); //Turn on buzzer
    while(1){ 
      //run while loop forever
    }
  }
}

void set_header_file(){//sets headers at begining of file
  myFile.print(F("Flight Log:\t"));
  myFile.print(hz);
  myFile.println(F("hz"));
  myFile.print(F("Time:\tAlt:\tVel:\tAcc:\tApo:\tApo_Act:\tEvents:\t"));
}

void close_file(){
  myFile.close(); //closes file
}

void reopen_file(){//closes then reopens the file, saving data up to this point
  close_file();//closes the file
  open_file();//opens the file
}

void log_data(){//saves current data to sd card
  myFile.print("\n");
  myFile.print(t2); myFile.print(F("\t"));       //logs current time
  myFile.print(x2); myFile.print(F("\t"));       //logs current position
  myFile.print(vel); myFile.print(F("\t"));      //logs current velocity
  myFile.print(acc); myFile.print(F("\t"));      //logs current acceleration
  myFile.print(apo); myFile.print(F("\t"));      //logs current apogee projection
  myFile.print(apo_act); myFile.print(F("\t"));;  //logs apogee
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

void average_acceleration_data(){//averages acceleration data over the set frequency and saves to z_global variable. Also updates t2 time variable.
  int i = 0; //iteration counting variable
  acc_avg = 0; // resets z_avg variable to 0
  
  do{ //run at least once
    t2 = millis();        // updates t2 variable with current time
    read_accelerometer(); //updates z_global variable
    acc_avg += acc;    //adds all the acceleration readings for each iteration
    i++;                  //counts the number of iterations
  }while(t2-t1 < 1000/hz-1);//exits loop once elapsed time is greater than frequency time period

  acc = acc_avg/i;     //averages the z acceleration data for the period
  delta_t = t2-t1;        //update change in time variable
  t1 = t2;                //updates the t1 point for when the loop ended
}


//***********PHYSICS FUNCTIONS***********

void read_velocity(){//reads new altitude as x2 and compares to previous altitude x1 and delta_t to determine velocity
  x2 = read_altitude(); //gets current altitude
  vel = (x2 - x1)/(delta_t / 1000); //determines velocity
  x1 = x2; //update previous altitude variable
}

void predict_apogee(){//predicts apogee with current velocity, altitude, and acceleration
  apo = - sq(vel)/(2*(acc - G)) * log((sq(vel) + sq(vel) * (-G)/(acc - G))/(sq(vel)*(-G)/(acc -G))) + x2;
}

int MCO(){//returns 1 if MCO is detected, 0 otherwise.
  if(acc < 0){//enters if acceleration is less than 0
    c++;      //increments counter
  }
  else{
    c = 0;      //resets counter if positive acceleration
  }
  if(c > 3){    //enters if last 3 acceleration datas are less than 0 
    c = 0;      //resets counter for apogee detection
    return 1;   //returns 1 for MCO detected
  }
  else{           
    return 0;   //returns 0 for MCO not detected
  }
}

int detect_apogee(){//looks for apogee and returns 1 if detected, 0 if not.
  if(x2 > apo_act){//enters if current position is higher than saved apogee value
    apo_act = x2; //saves new apogee value
    c = 0;      //resets apogee counter
  }
  else{
    c++;        //increments apogee counter if latest value is less than recorded apogee
  }
  if(c > 3){    //enters if last 3 positions are lower than recorded apogee
    c = 0;      //resets counter for touchdown
    return 1;   //returns 1 for apogee detected
  }
  else{           
    return 0;   //returns 0 for apogee not reached
  }
}


int detect_touchdown(){//returns 1 if touchdown is detected, otherwise returns 0.
  if (vel < 1 && vel > -1){ //enters if velocity is almost 0
    c++;    //increments counter
  }
  else{
    c = 0;  //resets counter
  }
  if(c > 5){  //enters if counted 5 velocities in a row close to 0
    c = 0;    //resets counter
    return 1; //returns 1 if touchdown detected
  }
  else{
    return 0;  //returns 0 if touchdown not detected
  }
}
