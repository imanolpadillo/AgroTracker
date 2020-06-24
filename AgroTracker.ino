// ********************************************************************************************* //
// ********************************************************************************************* //
// **************************************** AGROTRACKER **************************************** //
// ********************************************************************************************* //
// ********************************************************************************************* //
// AUTOR:   Imanol Padillo
// DATE:    31/05/2020
// VERSION: 1.0

// Board: Arduino Uno
// Processor: AVRISP mkll
// Port: /dev/cu.wchusbserial1420

// AgroTracker project is focused on solving two main plowing problems: a) angle deviation when
// performing straight lines and b) distance control in order to control the plowing area.
// The hardware used at this project is the following:
// 1. Arduino UNO
// 2. GPS GY-NEO-6M
// 3. QMC5883 magnetometer
// 4. 1602 LCD Display
// 5. 4x4 matrix keypad
// 6. Buzzer
// 7. RGB led

//  ARDUINO      |  GPS       | 
//  5V           |  VCC       | 
//  GND          |  GND       | 
//  D3(TX)       |  RX        | 
//  D4(RX)       |  TX        | 

//  ARDUINO      |  HALL-SENS.| 
//  VCC          |  VCC       | 
//  GND          |  GND       | 
//  D2           |  S         | 

//  ARDUINO      |  MAGNETOM. | 
//  5V           |  VCC       | 
//  GND          |  GND       | 
//  A4(SDA)      |  SDA       | 
//  A5(SCL)      |  SCL       | 

//  ARDUINO      |  LCD       | 
//  5V           |  VCC       | 
//  GND          |  GND       | 
//  A4(SDA)      |  SDA       | 
//  A5(SCL)      |  SCL       |

//  ARDUINO      |  KEYPAD    | 
//  D6           |  R1        | 
//  D7           |  R2        | 
//  D8           |  R3        | 
//  D9           |  R4        | 
//  D10          |  C1        | 
//  D11          |  C2        | 
//  D12          |  C3        | 
//  D13          |  C4        | 

//  ARDUINO      |  BUZZER    | 
//  5V           |  VCC       | 
//  GND          |  GND       | 
//  D5           |  I/O       | 

//  ARDUINO      |  LED       | 
//  GND          |  GND       | 
//  A0           |  R         | 
//  A1           |  G         | 
//  A2           |  B         | 

// Config commands can be typed in keypath. They must start with "*" and finish with "#". Bellow 
// the list of commands is shown:
//   *00X#      -> Act/Deact distance control (0:off, 1:gps, 2:tachometer)
//   *01XXXXXX# -> Set distance limit (mm). 6 digits must be included
//   *02XXXX#   -> Set wheel radius (mm). 4 digits must be included
//   *10X#      -> Act/Deact angle control (0:off, 1:on)
//   *11XX#     -> Set angle limit (degrees). 2 digits must be included
//   *12X#      -> Set angle axis. 1 digit must be included (0:horizontal, 1:vertical)
//   *13X#      -> Act/Deact angle turn (0:off, 1:on)
//   *20X#      -> Act/Deact buzzer (0:off, 1:on)
//   *21XX#     -> Buzzer max beeps. 2 digits must be included
// Besides there are some direct accesss (DA) commands. These commands are alphanumeric, and no 
// init/end characters are needed:
//   A          -> Statistics: total distance + time
//   B          -> Show config data
//   C          -> Show firmware version + cmd syntax
//   D          -> Reset distance/angle control  

// ********************************************************************************************* //
// INCLUDES 
// ********************************************************************************************* //
// - generic
#include <EEPROM.h>
#include <SoftwareSerial.h>
// -gps
#include <TinyGPS++.h>
// - lcd
#include <LiquidCrystal_I2C.h>
// - keypad
#include <Keypad.h>
// - i2c
#include "I2Cdev.h"
#include "Wire.h"
// - magnetometer
#include <MechaQMC5883.h>


// ********************************************************************************************* //
// GPIO
// ********************************************************************************************* //
#define GPIO_TACO                         2
#define GPIO_GPS_TX                       3
#define GPIO_GPS_RX                       4
#define GPIO_BUZZER                       5
#define GPIO_KP_R1                        6
#define GPIO_KP_R2                        7
#define GPIO_KP_R3                        8
#define GPIO_KP_R4                        9
#define GPIO_KP_C1                        10
#define GPIO_KP_C2                        11
#define GPIO_KP_C3                        12
#define GPIO_KP_C4                        13
#define GPIO_LED_R                        A0
#define GPIO_LED_G                        A1
#define GPIO_LED_B                        A2

// ********************************************************************************************* //
// CONSTANTS
// ********************************************************************************************* //
// - generic
#define TIMEOUT_SHORT                     2000
#define TIMEOUT_LARGE                     3500
// - commands                             1234567890123456
#define DISTANCE_NONE                     "0"
#define DISTANCE_GPS                      "1"
#define DISTANCE_TACOMETER                "2"
#define IDX_COMPASS_TOTAL                 3  
#define IDX_TOTAL                         9     
#define IDX_DISTANCE_ACT_DEACT            0
#define IDX_DISTANCE_LIMIT                1
#define IDX_DISTANCE_WHEEL                2
#define IDX_ANGLE_ACT_DEACT               3
#define IDX_ANGLE_LIMIT                   4
#define IDX_ANGLE_AXIS                    5
#define IDX_ANGLE_TURN_ACT_DEACT          6
#define IDX_BUZZER_ACT_DEACT              7
#define IDX_BUZZER_BEEPS                  8
#define IDX_COMPASS_X_OFFSET              9
#define IDX_COMPASS_Y_OFFSET              10
#define IDX_COMPASS_Z_OFFSET              11
#define OFF                               "0"
#define ON                                "1"
#define CMD_DISTANCE_ACT_DEACT            "00"
#define CMD_DISTANCE_LIMIT                "01"
#define CMD_DISTANCE_WHEEL                "02"
#define CMD_ANGLE_ACT_DEACT               "10"
#define CMD_ANGLE_LIMIT                   "11"
#define CMD_ANGLE_AXIS                    "12"
#define CMD_ANGLE_TURN_ACT_DEACT          "13"
#define CMD_BUZZER_ACT_DEACT              "20"
#define CMD_BUZZER_BEEPS                  "21"
#define CMD_DA_ESTATISTICS                "A"
#define CMD_DA_SHOW_CONFIG                "B"
#define CMD_DA_VERSION                    "C"
#define CMD_DA_RESET_DISTANCE_ANGLE       "D"
#define LEN_DISTANCE_LIMIT                6
#define LEN_DISTANCE_WHEEL                4
#define LEN_2_DIGITS                      2
#define LEN_1_DIGIT                       1
#define MSG_EMPTY                         "                "
#define MSG_INIT_1                        "Inicializando..."
#define MSG_INIT_2                        "GPS: ok, MAG: ok"
#define MSG_DISTANCE_ACT_DEACT            "CONTROL DIST:"
#define MSG_DISTANCE_LIMIT                "LIMITE DIST(mm):"
#define MSG_DISTANCE_WHEEL                "RADIO RUEDA(mm):"
#define MSG_ANGLE_ACT_DEACT               "CONTROL ANGULO:"
#define MSG_ANGLE_LIMIT                   "LIMITE ANG(d):"
#define MSG_ANGLE_AXIS                    "EJE H(0)/V(1):"
#define MSG_ANGLE_TURN_ACT_DEACT          "DETECTAR GIRO:"
#define MSG_BUZZER_ACT_DEACT              "BOCINA:"
#define MSG_BUZZER_BEEPS                  "MAX BEEPS:"
#define MSG_DA_ESTATISTICS                "TIME(h)/DIS(km):"
#define MSG_DA_VERSION                    "AgroTracker v1.0"
#define MSG_DA_DISTANCE_ACT_DEACT         "*00X#"
#define MSG_DA_DISTANCE_LIMIT             "*01XXXXXX#"
#define MSG_DA_DISTANCE_WHEEL             "*02XXXX#"
#define MSG_DA_ANGLE_ACT_DEACT            "*10X#"
#define MSG_DA_ANGLE_LIMIT                "*11XX#"
#define MSG_DA_ANGLE_AXIS                 "*12X#"
#define MSG_DA_ANGLE_TURN_ACT_DEACT       "*13X#"
#define MSG_DA_BUZZER_ACT_DEACT           "*20X#"
#define MSG_DA_BUZZER_BEEPS               "*21XX#"
#define MSG_CMD_ERROR                     "Error en comando"
// - gps
#define GPS_SATELLITES										4
// - buzzer                               
#define MSG_DA_RESET_DISTANCE_ANGLE       "Reset dis/ang"
#define BUZZER_SUCCESS                    0
#define BUZZER_ERROR                      1
#define BUZZER_KEY                        2
#define BUZZER_ACTIVATED                  3
#define BUZZER_DEACTIVATED                4
#define BUZZER_ALARM_DISTANCE             5
#define BUZZER_ALARM_ANGLE                6
// - led
#define LED_GREEN                         0  //distance ok, angle ok
#define LED_PURPLE                        1  //distance ok, angle no ok
#define LED_RED                           2  //distance no ok
#define LED_BLUE                          3  //configdata change
#define LED_YELLOW                        4  //less than 4 satellites
// - keypad
#define ROWS                              4
#define COLS                              4 
// - lcd
#define lcdAddress                        0x27
// - gyro
#define  mpuAddress                       0x68
// -auto limit distance
#define INIT_TURN_ANGLE                   60
#define END_TURN_ANGLE                    80
#define RESET_DISTANCE_ANGLE              0
#define RESET_DISTANCE                    1
#define RESET_ANGLE                       2

// ********************************************************************************************* //
// VARIABLES
// ********************************************************************************************* //
// - generic
int sensor_timeout_counter;
double distance_total;                            // accumalative distance value
double time_init;                                 // init microseconds
double time_total;                                // accumalated microseconds
SoftwareSerial ss(GPIO_GPS_RX, GPIO_GPS_TX);      // soft serial for gps
int eeAddress = 0;                                // init eeprom address
double distance = 0;                              // distance in meters
bool distance_limit_flag = false;                 // limit distance flag
bool angle_limit_flag = false;                    // limit angle flag
float angle = 0;                                  // jaw angle in degrees
// - command data
bool cmd_in_progress = false;                     // This flag is activated when cmd init character (*) is received
String cmd_data = "";                             // Command information
// - config data
String configdata [IDX_TOTAL] = {                 // config values: 
 "0", "018000", "0500",                           // dist_act_deact/dist_limit/dist_wheel/
 "1", "10", "0", "0",                             // ang_act_deact/ang_limit/ang_axis/ang_90_detection/
 "1", "03"                                        // buzzer_act_deact/buzzer_beeps                  
};  
// -gps
bool sensorsReady;
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;
double gps_init_lat=0;
double gps_init_lng=0;
// -lcd
LiquidCrystal_I2C lcd(lcdAddress, 16, 2);
// - keypad
const char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {GPIO_KP_R1, GPIO_KP_R2, GPIO_KP_R3, GPIO_KP_R4}; 
byte colPins[COLS] = {GPIO_KP_C1, GPIO_KP_C2, GPIO_KP_C3, GPIO_KP_C4}; 
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
// -gyro
bool reset_mag;
float angle_init, ang_y_init, ang_z_init;
bool turning;
int angleoffset;
// -led
int led_color;
// -Tacometer
volatile int tacoCounter=0;
// -buzzer
int alarmAngleCounter=0;
int alarmDistanceCounter=0;
// -magnetometer
MechaQMC5883 qmc;
float calibrated_values[3]; 

// ********************************************************************************************* //
// SETUP
// ********************************************************************************************* //
void setup() {
  // Setup digital pins
  pinMode(GPIO_LED_R, OUTPUT);   
  pinMode(GPIO_LED_G, OUTPUT);   
  pinMode(GPIO_LED_B, OUTPUT);   

  // Setup serial port
  Serial.begin(9600);

  // Get Eeprom config data
  //eepromWriteConfigData();
  eepromReadConfigData();
  
  // Init buzzer
  playBuzzer(BUZZER_SUCCESS);

  // Init RGB led
  playLed(LED_GREEN,0);
  
  // Init gps
  ss.begin(GPSBaud);
	
	// Init taco (interrupt)
	pinMode(GPIO_TACO, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(GPIO_TACO), tacoCheckLimitDistance, FALLING);

  // Init lcd
  lcd.begin();                      
  lcd.backlight();
  lcdPrint(MSG_DA_VERSION,"",TIMEOUT_LARGE);

  // Init I2C
  Wire.begin();

  // Init magnetometer
  qmc.init();
  reset_mag=true;

  // Wait for gps initialization
  lcdPrint(MSG_INIT_1,"",0);

  // Get init time
  time_init = millis();
   
}

// ********************************************************************************************* //
// MAIN
// ********************************************************************************************* //
void loop() {
  // set total time
  time_total = millis() - time_init;
  
  // wait until gps is initialized
  if ((sensorsReady == false) and (configdata[IDX_DISTANCE_ACT_DEACT]==DISTANCE_GPS) 
     and (gps_init_lat != 0) and (gps_init_lng != 0)){
    endGpsInicialization();
  } else if ((sensorsReady == false) and (configdata[IDX_DISTANCE_ACT_DEACT]!=DISTANCE_GPS)){
    endGpsInicialization();
  }
  
  // Check keypad input
  char customKey = customKeypad.getKey();
  if (isAlphaNumeric(customKey) or isDigit(customKey)
      or customKey == '*' or customKey == '#'){
    //Serial.println(customKey);
    playBuzzer(BUZZER_KEY);
  }
  
  // If "*" is pressed, a command will start
  if (customKey == '*'){
    cmd_in_progress = true;
    cmd_data = "";
  }
  // If "#" is pressed, a command has ended 
  else if (customKey == '#'){
    decodeCmd(2);
    cmd_in_progress = false;
    cmd_data = "";
  }
  // If "0-9" is pressed, a command is being sended 
  else if ((cmd_in_progress == true) and (isDigit(customKey))){
    cmd_data = cmd_data + customKey;
  }
  // If "A/B/C/D" is pressed, a info command must be shown
  else if ((customKey == 'A') or (customKey == 'B') or 
    (customKey == 'C') or (customKey == 'D')){
    if (cmd_in_progress == true) {
      playBuzzer(BUZZER_ERROR);
      cmd_in_progress = false;
      cmd_data = "";
    } 
    else {
      cmd_data = customKey;
      decodeCmd(1);
     cmd_data = "";
    }
  }

  // Angle: check orientation limit
  if (configdata[IDX_ANGLE_ACT_DEACT]==ON) {
    if (sensor_timeout_counter==20){
      angle_limit_flag = magCheckLimitAngle();
    } else {
      sensor_timeout_counter++;
    }
  }
  
  // Distance (GPS): check distance limit
  if (configdata[IDX_DISTANCE_ACT_DEACT]==DISTANCE_GPS) {
    while (ss.available()>0) {
      int c = ss.read();
      if (gps.encode(c)){
        distance_limit_flag = gpsCheckLimitDistance();
      }
    }
  }

  // Distance (Tacometer): check distance limit. Here interrupt data is handled
  if (configdata[IDX_DISTANCE_ACT_DEACT]==DISTANCE_TACOMETER) {
		distance = 2 * M_PI * configdata[IDX_DISTANCE_WHEEL].toDouble()/1000 * tacoCounter;
		if (distance>=configdata[IDX_DISTANCE_LIMIT].toInt()/1000) {
				distance_limit_flag = true;
		}
		else {
			alarmDistanceCounter=0;
			distance_limit_flag = false;
		}
  }

  // Distance auto mode
 if (configdata[IDX_ANGLE_TURN_ACT_DEACT]==ON and
     configdata[IDX_DISTANCE_ACT_DEACT]!=DISTANCE_NONE) {
    autoCheckLimitDistance();
  }
  
  // Display result
  if (sensorsReady == true){
    // results in lcd
    lcdDisplayResults();
    // results in buzzer
    buzzerDisplayResults();
    // results in lcd
    ledDisplayResults();
  }

  
}


// ********************************************************************************************* //
// ********************************************************************************************* //
// HELP FUNCTIONS
// ********************************************************************************************* //
// ********************************************************************************************* //

// ********************************************************************************************* //
// EEPROM
// ********************************************************************************************* //
// Serialize and write config data in eeprom
void eepromWriteConfigData () {
  String tmp="";
  // Serialize data
  for (int i=0; i<IDX_TOTAL; i++) {
    tmp = tmp + configdata[i] + ";";
  }
  // Write in eeprom
  for (int j=eeAddress; j<tmp.length(); j++){
    if (tmp[j]!=";"){
      unsigned int digit = tmp[j];
      EEPROM.write(j, digit);
    }
    else {
      EEPROM.write(j, 255);
    }
  }
}

// Read and unserialize config data from eeprom
void eepromReadConfigData () {
  String tmp = "";
  // Read from eeprom
  for (int j=eeAddress; j<40; j++){
    char digit = EEPROM.read(j);
    tmp = tmp + digit;
  }
  // Deserialize data
  for (int i=0; i<IDX_TOTAL; i++)
  {
    configdata[i] = splitString(tmp,';',i);
    Serial.println("configdata[" + String(i) + "]: " + configdata[i]);
  }
}

// ********************************************************************************************* //
// GENERIC
// ********************************************************************************************* //
// Split a string
String splitString(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Return a rounded float. Number of decimals is a function input.
float DecimalRound(float input, int decimals)
{
  float scale=pow(10,decimals);
  return round(input*scale)/scale;
}

// ********************************************************************************************* //
// SENSOR: GPS/TACO 
// ********************************************************************************************* //

// End gps inicialization
void endGpsInicialization(){
  lcdPrint(MSG_INIT_1,MSG_INIT_2,TIMEOUT_LARGE);
  lcd.clear();
  sensorsReady = true;
}

// it detects when vehicle has turned, and automatically resets angle and distance
void autoCheckLimitDistance() {
  if ((abs(angle) > INIT_TURN_ANGLE) and (turning == false)) {
    //Serial.println("Starting turn");
    turning = true;
  }
  else if ((abs(angle) > END_TURN_ANGLE) and (turning == true)) {
    //Serial.print("90deg turn");
    turning = false;
    resetSensors(RESET_DISTANCE);   // reset distance
    if (angle > INIT_TURN_ANGLE){
      angleoffset = angleoffset - 90;
    }
    else{
      angleoffset = angleoffset + 90;
    }
  }
}

// Check if limit distance has been achieved. Distance measured with gps.
bool gpsCheckLimitDistance() {
  bool output = false;
  double latitude = DecimalRound(gps.location.lat(),6);
  double longitude = DecimalRound(gps.location.lng(),6);
  if ((gps_init_lat == 0) and (gps_init_lng == 0)) {
    // Set init coordenates
    //Serial.println("Init gps coordinates");
    gps_init_lat=latitude;
    gps_init_lng=longitude;
  }
  else {
    // Calculate current distance
    distance = TinyGPSPlus::distanceBetween(
      gps_init_lat, gps_init_lng, latitude, longitude);
    //Serial.println("speed: " + String(gps.speed.kmph(),6) + ", distance: " + String(distance,6) + ", latitud_i: " + String(gps_init_lat,6) + ", longitud_i: " + String(gps_init_lng,6) + ", latitud: " + String(latitude,6) + ", longitud: " + String(longitude,6));
    if (distance>=configdata[IDX_DISTANCE_LIMIT].toInt()/1000) {
      output = true;
    }
    else{
      alarmDistanceCounter = 0;
    }
 } 
 return output;
}

// Check if limit distance has been achieved. Distance measured with tacometer.
bool tacoCheckLimitDistance(){
	tacoCounter = tacoCounter + 1;
	Serial.println("tacoCounter: " + String(tacoCounter));
}

// ********************************************************************************************* //
// SENSOR: Magnetómeter 
// ********************************************************************************************* //
bool magCheckLimitAngle(){
  int x,y,z;
  qmc.read(&x,&y,&z);

  float values_from_magnetometer[3];
  values_from_magnetometer[0] = x;
  values_from_magnetometer[1] = y;
  values_from_magnetometer[2] = z;
  transformation(values_from_magnetometer);

  int azimuth;
  int x_calibrated = (int) calibrated_values[0];
  int y_calibrated = (int) calibrated_values[1];
  int z_calibrated = (int) calibrated_values[2];
  if (configdata[IDX_ANGLE_AXIS] == "0"){
    azimuth = qmc.azimuth(&z_calibrated, &x_calibrated);
  } else {
    azimuth = qmc.azimuth(&z_calibrated, &y_calibrated);
  }
  
  
  int azimuth_prev;

  if (reset_mag == true){
    angle_init = azimuth;
    //Serial.println ("Init angle = " + String(angle_init));
    reset_mag = false;
  }
  if (azimuth!=0) {
    // check angle pass between 0 and 360
    azimuth_prev = angle_init - angle + angleoffset;
    if ((azimuth - azimuth_prev) > 300){
      angle_init = angle_init + 360;
    } else if ((azimuth_prev - azimuth) > 300){
      angle_init = angle_init - 360;
    }
    
    angle = angle_init - azimuth + angleoffset;   
  }

  //Serial.print ("Heading angle = ");
  //Serial.print (azimuth);
  //Serial.println(" Degree");
  
  //Serial.println(String(ang_x) + ";" + String(mx) + ";" + String(my) + ";" + String(mz));
  if (abs(angle) > configdata[IDX_ANGLE_LIMIT].toInt()){
    return true;
  } else {
    alarmAngleCounter=0;
    return false;}

 return true;
}

//angle transformation for a calibrated measurement
void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {1.204, 0, -0.023},
    {-0.049, 1.213, 0.234},
    {0.022, -0.137, 1.264}   
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    -49.334,
    193.519,
    -510.485
  }; 
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

// ********************************************************************************************* //
// LCD
// ********************************************************************************************* //
// Print data in lcd display
void lcdPrint(String line1, String line2, int timeout)
{
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  if (timeout != 0){
    delay(timeout);
    lcd.clear();
  }
}

// Display current distance and angle data
void lcdDisplayResults()
{
  String tdistance;
  String tangle;
  
  if (configdata[IDX_DISTANCE_ACT_DEACT] != DISTANCE_NONE) {
    tdistance = String(DecimalRound(distance,2));
  }
  else {
    tdistance = "----";
  }
  if (configdata[IDX_ANGLE_ACT_DEACT] != OFF) {
    tangle = String(DecimalRound(angle,2));
  }
  else {
    tangle = "----";
  }
  String line1 = "Dist(m):" + tdistance + "     ";
  String line2 = "Ang (d):" + tangle + "     ";
  lcdPrint(line1, line2, 0);
}

// ********************************************************************************************* //
// COMMANDS AND CONFIG DATA
// ********************************************************************************************* //
// Decode a finished command
void decodeCmd(int cmdID_length){
  String cmdID = "";
  if (cmd_data.length()>= cmdID_length) {
    cmdID = cmd_data.substring(0,cmdID_length);
  }
  // DISTANCE_ACT_DEACT
  if (cmdID == CMD_DISTANCE_ACT_DEACT){
    setConfigData(IDX_DISTANCE_ACT_DEACT, LEN_1_DIGIT, MSG_DISTANCE_ACT_DEACT);
  }
  // DISTANCE_LIMIT
  else if (cmdID == CMD_DISTANCE_LIMIT){
    setConfigData(IDX_DISTANCE_LIMIT, LEN_DISTANCE_LIMIT, MSG_DISTANCE_LIMIT);
  }
  // DISTANCE_WHEEL
  else if (cmdID == CMD_DISTANCE_WHEEL){
    setConfigData(IDX_DISTANCE_WHEEL, LEN_DISTANCE_WHEEL, MSG_DISTANCE_WHEEL);
    resetSensors(RESET_DISTANCE_ANGLE);
  }
  // ANGLE_ACT_DEACT
  else if (cmdID == CMD_ANGLE_ACT_DEACT){
    setConfigData(IDX_ANGLE_ACT_DEACT, LEN_1_DIGIT, MSG_ANGLE_ACT_DEACT);
  }
  // ANGLE_LIMIT
  else if (cmdID == CMD_ANGLE_LIMIT){
    setConfigData(IDX_ANGLE_LIMIT, LEN_2_DIGITS, MSG_ANGLE_LIMIT);
  }
  // ANGLE_AXIS
  else if (cmdID == CMD_ANGLE_AXIS){
    setConfigData(IDX_ANGLE_AXIS, LEN_1_DIGIT, MSG_ANGLE_AXIS);
    resetSensors(RESET_ANGLE);
  }
  // ANGLE_TURN_ACT_DEACT
  else if (cmdID == CMD_ANGLE_TURN_ACT_DEACT){
    setConfigData(IDX_ANGLE_TURN_ACT_DEACT, LEN_1_DIGIT,
      MSG_ANGLE_TURN_ACT_DEACT);
    resetSensors(RESET_DISTANCE_ANGLE);
  }
  // BUZZER_ACT_DEACT
  else if (cmdID == CMD_BUZZER_ACT_DEACT){
    setConfigData(IDX_BUZZER_ACT_DEACT, LEN_1_DIGIT, MSG_BUZZER_ACT_DEACT);
  }
  // BUZZER_BEEPS
  else if (cmdID == CMD_BUZZER_BEEPS){
    setConfigData(IDX_BUZZER_BEEPS, LEN_2_DIGITS, MSG_BUZZER_BEEPS);
  }
  // DA_ESTATISTICS
  else if (cmdID == CMD_DA_ESTATISTICS){
    lcd.clear();
    playBuzzer(BUZZER_SUCCESS);
    float time_total_h = time_total/3600000;
    lcdPrint(MSG_DA_ESTATISTICS, String(DecimalRound(time_total_h,2)) + "/" + 
      String(DecimalRound(distance_total/1000,1)), TIMEOUT_LARGE);
  }
  // DA_SHOW_CONFIG
  else if (cmdID == CMD_DA_SHOW_CONFIG){
    playBuzzer(BUZZER_SUCCESS);
    lcd.clear();
    String tmp;
    
    if (configdata[IDX_DISTANCE_ACT_DEACT]==OFF) {tmp="off";} 
    else if(configdata[IDX_DISTANCE_ACT_DEACT]==ON) {tmp="gps";} else{tmp="taco";}
    lcdPrint(MSG_DISTANCE_ACT_DEACT, tmp, TIMEOUT_SHORT);
    
    lcdPrint(MSG_DISTANCE_LIMIT, configdata[IDX_DISTANCE_LIMIT], TIMEOUT_SHORT);

    lcdPrint(MSG_DISTANCE_WHEEL, configdata[IDX_DISTANCE_WHEEL], TIMEOUT_SHORT);
    
    if (configdata[IDX_ANGLE_TURN_ACT_DEACT]==OFF) {tmp="off";} else {tmp="on";}
    lcdPrint(MSG_ANGLE_TURN_ACT_DEACT, tmp, TIMEOUT_SHORT);

    if (configdata[IDX_ANGLE_ACT_DEACT]==OFF) {tmp="off";} else {tmp="on";}
    lcdPrint(MSG_ANGLE_ACT_DEACT, tmp, TIMEOUT_SHORT);
    
    lcdPrint(MSG_ANGLE_LIMIT, configdata[IDX_ANGLE_LIMIT] ,TIMEOUT_SHORT);

    if (configdata[IDX_ANGLE_AXIS]==OFF) {tmp="H";} else {tmp="V";}
    lcdPrint(MSG_ANGLE_AXIS, tmp, TIMEOUT_SHORT);

    if (configdata[IDX_BUZZER_ACT_DEACT]==OFF) {tmp="off";} else {tmp="on";}
    lcdPrint(MSG_BUZZER_ACT_DEACT, tmp, TIMEOUT_SHORT);

    lcdPrint(MSG_BUZZER_BEEPS, configdata[IDX_BUZZER_BEEPS], TIMEOUT_SHORT);
  }
  // DA_VERSION
  else if (cmdID == CMD_DA_VERSION){
    playBuzzer(BUZZER_SUCCESS);
    lcd.clear();
    
    lcdPrint(MSG_DA_VERSION,MSG_EMPTY,TIMEOUT_LARGE);
    
    /*lcdPrint(MSG_DISTANCE_ACT_DEACT, MSG_DA_DISTANCE_ACT_DEACT, TIMEOUT_SHORT);
    
    lcdPrint(MSG_DISTANCE_LIMIT, MSG_DA_DISTANCE_LIMIT, TIMEOUT_SHORT);

    lcdPrint(MSG_DISTANCE_WHEEL, MSG_DA_DISTANCE_WHEEL, TIMEOUT_SHORT);
    
    lcdPrint(MSG_ANGLE_TURN_ACT_DEACT, MSG_DA_ANGLE_TURN_ACT_DEACT, TIMEOUT_SHORT);

    lcdPrint(MSG_ANGLE_ACT_DEACT, MSG_DA_ANGLE_ACT_DEACT, TIMEOUT_SHORT);
    
    lcdPrint(MSG_ANGLE_LIMIT, MSG_DA_ANGLE_LIMIT, TIMEOUT_SHORT);

    lcdPrint(MSG_ANGLE_AXIS, MSG_DA_ANGLE_AXIS, TIMEOUT_SHORT);

    lcdPrint(MSG_BUZZER_ACT_DEACT, MSG_DA_BUZZER_ACT_DEACT, TIMEOUT_SHORT);

    lcdPrint(MSG_BUZZER_BEEPS, MSG_DA_BUZZER_BEEPS, TIMEOUT_SHORT);*/
  }
  // DA_RESET_DISTANCE_ANGLE
  else if (cmdID == CMD_DA_RESET_DISTANCE_ANGLE){
    distance_total = distance_total + distance;
    resetSensors(RESET_DISTANCE_ANGLE);
    playBuzzer(BUZZER_SUCCESS);
    lcdPrint(MSG_DA_RESET_DISTANCE_ANGLE,MSG_EMPTY,TIMEOUT_LARGE);
  }
  // OTHER
  else{
    playBuzzer(BUZZER_ERROR);
    playLed(LED_RED, TIMEOUT_SHORT);
  }
}

// Set config data
bool setConfigData(int configDataIndex, int cmdlength, String msg) {
  // delete cmd ID
  cmd_data = cmd_data.substring(2);
  if (cmd_data.length() == cmdlength) {
    int buzzer_output = checkCmdSyntax(configDataIndex);
    //Serial.println("buzzer_output:" + String(buzzer_output));
    if (buzzer_output != BUZZER_ERROR){
      configdata[configDataIndex] = String(cmd_data);
      playBuzzer(buzzer_output);
      playLed(LED_BLUE, TIMEOUT_SHORT);
      lcd.clear();
      lcdPrint(msg,cmd_data,TIMEOUT_LARGE);
      eepromWriteConfigData();
    }
    else {
      playBuzzer(buzzer_output);
      playLed(LED_BLUE, TIMEOUT_SHORT);
      lcd.clear();
      lcdPrint(MSG_CMD_ERROR,cmd_data + MSG_EMPTY,TIMEOUT_LARGE);
    }
  }
  else {
    playBuzzer(BUZZER_ERROR);
    playLed(LED_RED, TIMEOUT_SHORT);
    lcdPrint(MSG_CMD_ERROR,cmd_data + MSG_EMPTY,TIMEOUT_LARGE);
  }
}

// check command syntax
int checkCmdSyntax(int configDataIndex) {
  // Syntax: 0, 1
  if ((configDataIndex == IDX_ANGLE_TURN_ACT_DEACT) or
     (configDataIndex == IDX_ANGLE_ACT_DEACT) or
     (configDataIndex == IDX_ANGLE_AXIS) or
     (configDataIndex == IDX_BUZZER_ACT_DEACT))
  {
    if (cmd_data == OFF) {
      return BUZZER_DEACTIVATED;
    }
    else if (cmd_data == ON) {
      return BUZZER_ACTIVATED;
    }
    else {
      return BUZZER_ERROR;
    }
  }
  // Syntax: 0, 1, 2
  else if (configDataIndex == IDX_DISTANCE_ACT_DEACT)
  {
    if (cmd_data == "0") {
      return BUZZER_DEACTIVATED;
    }
    else if (cmd_data == "1") {
      return BUZZER_ACTIVATED;
    }
    else if (cmd_data == "2") {
      return BUZZER_ACTIVATED;
    }
    else {
      return BUZZER_ERROR;
    }
  }
  else {
    return BUZZER_SUCCESS;
  }
}

// reset sensors (0: distance/angle, 1: distance, 2: angle)
void resetSensors(int sensor) {
  // Reset distance
  if (sensor == 0 or sensor == 1){
    distance_total = distance_total + distance;
    distance_limit_flag = false;
    distance = 0;
    gps_init_lat = 0;
    gps_init_lng = 0;
    tacoCounter = 0;
    alarmDistanceCounter=0;
  }
  // Reset angle
  if(sensor == 0 or sensor == 2){
    angle_limit_flag = false;
    angle = 0;
    angleoffset = 0;
    reset_mag = true;
    turning = false;
    alarmAngleCounter=0;
  }
}

// ********************************************************************************************* //
// BUZZER / LED
// ********************************************************************************************* //
// Play buzzer (0:success, 1:error, 2:key, 3:activated, 4:deactivated)
void playBuzzer (int option)
{
  if (configdata[IDX_BUZZER_ACT_DEACT] == ON){
    if ((option == BUZZER_SUCCESS) or (option == BUZZER_ACTIVATED) or (option == BUZZER_DEACTIVATED))
    {
      tone(GPIO_BUZZER, 11300);
      delay(500);
      noTone(GPIO_BUZZER);
      delay(200);
      tone(GPIO_BUZZER, 11600);
      delay(200);
      noTone(GPIO_BUZZER);
      digitalWrite(GPIO_BUZZER, HIGH);
    }
    else if (option == BUZZER_ERROR or option == BUZZER_ALARM_DISTANCE)
    {
      tone(GPIO_BUZZER, 100, 500);
      delay(500);
      noTone(GPIO_BUZZER);
      digitalWrite(GPIO_BUZZER, HIGH);
    }
    else if (option == BUZZER_KEY or option == BUZZER_ALARM_ANGLE)
    {
      tone(GPIO_BUZZER, 10000);
      delay(80);
      noTone(GPIO_BUZZER);
      digitalWrite(GPIO_BUZZER, HIGH);
    }
  }
}

// play RGB led
void playLed (int color, int timeout) {
  int prev_led_color = led_color;
  if (color == LED_GREEN) {
    digitalWrite(GPIO_LED_R, 0);
    digitalWrite(GPIO_LED_G, 255);
    digitalWrite(GPIO_LED_B, 0);
  }
  else if (color == LED_PURPLE) {
    digitalWrite(GPIO_LED_R, 255);
    digitalWrite(GPIO_LED_G, 0);
    digitalWrite(GPIO_LED_B, 255);
  }
  else if (color == LED_RED) {
    digitalWrite(GPIO_LED_R, 255);
    digitalWrite(GPIO_LED_G, 0);
    digitalWrite(GPIO_LED_B, 0);
  }
  else if (color == LED_BLUE) {
    digitalWrite(GPIO_LED_R, 0);
    digitalWrite(GPIO_LED_G, 0);
    digitalWrite(GPIO_LED_B, 255);
  }
	else if (color == LED_YELLOW) {
    digitalWrite(GPIO_LED_R, 255);
    digitalWrite(GPIO_LED_G, 255);
    digitalWrite(GPIO_LED_B, 0);
  }
	
  if (timeout != 0){
    delay(timeout);
    playLed(prev_led_color,0);
  }
  led_color = color;
}

// Display results using buzzer
void buzzerDisplayResults() {
  int maxBuzzerBeeps = configdata[IDX_BUZZER_BEEPS].toInt();
  if (configdata[IDX_BUZZER_ACT_DEACT] == ON) {
    // Alarm distance
    if (configdata[IDX_DISTANCE_ACT_DEACT] != DISTANCE_NONE){
      if (distance_limit_flag == true and 
        (alarmDistanceCounter<maxBuzzerBeeps or maxBuzzerBeeps == 0)){
        playBuzzer(BUZZER_ALARM_DISTANCE);
        alarmDistanceCounter++;
      }
    }
    // Alarm angle
    if (configdata[IDX_ANGLE_ACT_DEACT] != 0){
      if (angle_limit_flag == true and 
        (alarmAngleCounter<maxBuzzerBeeps or maxBuzzerBeeps == 0)){
        playBuzzer(BUZZER_ALARM_ANGLE);
        alarmAngleCounter++;
      }
    }
  }
}

// Display results using rgb led
void ledDisplayResults() {
  bool ledAlreadySet = false;
  // Alarm distance
  if (configdata[IDX_DISTANCE_ACT_DEACT] != DISTANCE_NONE){
    if (distance_limit_flag == true){
      playLed(LED_RED,0);
      ledAlreadySet = true;
    }
  }
  // Alarm angle
  if ((configdata[IDX_ANGLE_ACT_DEACT] != OFF) and (ledAlreadySet == false)){
    if (angle_limit_flag == true){
      playLed(LED_PURPLE,0);
			ledAlreadySet = true;
    }
  }
	// Alarm number of satellites
  if ((configdata[IDX_DISTANCE_ACT_DEACT] == DISTANCE_GPS) and (ledAlreadySet == false)){
    if (gps.satellites.value() < GPS_SATELLITES){
      //Serial.print("Satelites:");
      //Serial.println(gps.satellites.value());
      playLed(LED_YELLOW,0);
			ledAlreadySet = true;
    }
  }
	// Default: green led
	if (ledAlreadySet == false){
		playLed(LED_GREEN,0);
	}
}
