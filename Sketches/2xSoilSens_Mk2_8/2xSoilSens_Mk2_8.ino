/*    Last mod date: 5/2/24
  Features:
    >Datalogger for temperature, humidity and soil humidity measurement.
    >Shows on LCD scree 2X16 the temperature, humidity and VPD values
    >Speed control of 2 fans
      >controlled by TB6612 driver module
      >changes speeds based on the VPD values
        >high humidity = low VPD -> extractor and aeration fan speed increases
        >low humidity = high VPD -> decrease extractor fan speed 
        >high temperature = high VPD -> aeration fan speed increases
    >Usage of watering capabilities when humidity is <65%
    

  TODO:
    >(later) add external functionality with buttons and indicator leds
    >(!!!) add fan control with VPD
    >Remove making 3 measurements and the logging of the standard deviation... it is kinda useless tbh
    >implement sleep mode for the LCD screen
      >only turns on for 3-5 mins if a button is pressed
        >or if an error is occurring (?)
    >include VPD values in temperature and humidity .txt

  
  Connections:
    Connection of RTC: 
      5V, SDA->A4, SCL->A5
    Connection of SD reader:
      5V, CS->D4 SCK->D13, MOSI->D11, MISO->D12
    Connection of LCD
      5V, SDA->A4, SCL->A5
    DHT22  -> 5V, D5
*/

//Libraries
  #include <LCD_I2C.h>          //for LCD
  #include <I2C_RTC.h>          //for Real time clock
  #include <SD.h>               //for SD card datalogger
  #include <DHT.h>              //for DHT22 temperature/humidity sensor
  #include <SparkFun_TB6612.h>  //for the motor driver for fan speed control


//Constants
  unsigned long SoilMesTime = millis();
  unsigned long prevTime2 = millis();
  unsigned long timestats = millis();

  //constants for sensors
    #define SPin1 A0           // A pin where Soil sensor1 is connected
    #define SPin2 A3           // A pin where Soil sensor2 is connected
    #define DHTPIN 5           // D pin for DHT22
    #define DHTTYPE DHT22      // DHT 22  (AM2302)
    DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor for normal 16mhz Arduino

    #define WSensOn 2       // D pin of turning on the water sensor
    #define WSensIn 3       // D pin of the water sensor input
    #define WPump A2        // D pin of the water pump

    const byte numReadings = 3;  // define the number of readings to take to make an average
    byte SoilSensNum = 1;        // number of Soil sensors (water pump will only be controlled by sensor 1)

    //defines top and bottom measurements at start to know the range they work at
    //for soil capacitor sensor, higher value = less water
    struct Cal {
      unsigned int HighCal1 : 10;
      unsigned int LowCal1 : 10;
      unsigned int HighCal2 : 10;
      unsigned int LowCal2 : 10;
    } Calvalues;

  //Constants for other modules
    //#define RLED 3     // D pin where LED indicator is connected
    LCD_I2C lcd(0x27, 16, 2);  // Default address of most PCF8574 modules, change according
    static DS3231 RTC;
    const byte chipSelect = 4;  //??? i believe it is for the RTC?
    int hours, minutes, seconds, day, month, year;

  //constants for the motor driver
    const int offsetA = 1;
    const int offsetB = 1;
    #define STBY 11
    //pins for the motor1
    #define AIN1 6
    #define AIN2 7
    #define PWMA 9
    //pins for motor2
    #define BIN1 8
    #define BIN2 A1
    #define PWMB 10

    Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
    Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//references of functions
  void LogDate();
  void SoilMeasurement1();
  void SoilMeasurement2();
  void LogDate();
  void ManualSoilCal();
  void Watering1();

void setup() {
  //initiate serial and LCD
  Serial.begin(9600);
  while (!Serial) {
  }
  //digitalWrite(RLED, HIGH);
  lcd.begin();
  lcd.backlight();
  lcd.print("Initiating...");
  Serial.println("Initiating...");
  //start RTC
  RTC.begin();
  RTC.getHourMode() == CLOCK_H24;

  //start the SD card
  // see if the card is present and can be initialized:
  if (!SD.begin(4)) {
    Serial.println("Card failed, or not present");
    lcd.clear();
    lcd.print("Card failed,");
    lcd.setCursor(0, 1);
    lcd.print("or not present");
    // don't do anything more:
    while (1) {
      /*
      digitalWrite(RLED, HIGH);
      delay(100);
      digitalWrite(RLED, LOW);
      delay(100);
      */
    }
  }
  lcd.setCursor(0, 1);
  lcd.print("SD card working");

  //Start DHT22 hum. and temp. meter
  dht.begin();
  //Set initial calibration values for soil meter.      !!! if you already know the upper and lower values write it
  Calvalues.HighCal1 = 225;
  Calvalues.HighCal2 = 345;
  Calvalues.LowCal1 = 565;
  Calvalues.LowCal2 = 750;
  //start common stuff
    // Pin settings:
      pinMode(BIN2, OUTPUT);      // Used for pin BIN2 of the motor driver, as it requires the use of A1 as digital pin
      pinMode(WPump, OUTPUT);     // Used similarly for the water pump, being analog pin A2
        //if both of them fail to work, use "digitalWrite" instead

      pinMode(WSensOn, OUTPUT);
      pinMode(WSensIn, INPUT);

      digitalWrite(WSensOn, LOW);
      digitalWrite(WPump, LOW);
    //
      millis();

  //Complete the setup by showing date
    delay(2000);
    lcd.clear();
    lcd.print(RTC.getHours());
    lcd.print(":");
    lcd.print(RTC.getMinutes());
    lcd.print(":");
    lcd.print(RTC.getSeconds());
    lcd.print(", ");
    lcd.setCursor(0, 1);
    lcd.print(RTC.getDay());
    lcd.print("-");
    lcd.print(RTC.getMonth());
    lcd.print("-");
    lcd.print(RTC.getYear());
    lcd.print("");
    delay(2000);
    lcd.clear();
    //digitalWrite(RLED, LOW);

  //ManualSoilCal();  //removed, used to obtain measures when connected to PC, otherwise TODO: create input functionality with button

}
void loop() {
  

  if (millis() - SoilMesTime >= 900000) { //Soil sensor measurement 900000 = 15 mins

    lcd.setCursor(0, 1);
    lcd.print("Reading soil hum.");
    //digitalWrite(RLED, HIGH);

    float Soil1Data[2];
    SoilMeasurement1(Soil1Data);

    // in case there is >1 soil sensor:
      if (SoilSensNum > 1) {
        float Soil2Data[2];
        SoilMeasurement2(Soil2Data);
        float SoilIntArray[4];
        for (int i = 0; i < 2; i++) {
          SoilIntArray[i] = Soil1Data[i];
          SoilIntArray[i + 2] = Soil2Data[i];
        }
        PrintData(1, SoilIntArray);
        LogData(1, SoilIntArray);
      }

    // in case there is only 1 soil sensor:
      else {    //creates an array and sends it to print and log data 
        float SoilIntArray[2];
        for (int i = 0; i < 2; i++) {
          SoilIntArray[i] = Soil1Data[i];
        }
        PrintData(1, SoilIntArray);
        LogData(1, SoilIntArray);
      }
    //restarts the counting for next soil humidity datalog
      SoilMesTime = millis();

    //checks if humidity was lower than a threshold
      if (Soil1Data[0] <= 65) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Humidity low");
        lcd.setCursor(0, 1);
        lcd.print("Starting watering");
        Watering1();
      }
    //digitalWrite(RLED, LOW);
  }

  if (millis() - prevTime2 >= 600000) { //Temp+Hum sensor measurement 300000 = 5 min
    lcd.clear();
    lcd.print("measuring Air...");
    float AirData[2];
    ReadDHT22(AirData);
    PrintData(2, AirData);
    LogData(2, AirData);
    prevTime2 = millis();
    //digitalWrite(RLED, LOW);
  }

  if (millis() - timestats >= 10000)  //Temp+Hum sensor measurement real time 5000 = 5s
  {
    float temp = dht.readTemperature();
    float RH = dht.readHumidity();
    byte offset = 1;  //Offset of leaf temperature difference from air temperature. It is in negative! 2 = -2 ºC difference
    lcd.clear();
    lcd.print(RH, 1);
    lcd.print(" RH, ");
    lcd.print(temp, 1);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("  ");
    float VPD = (0.61078 * exp((temp - offset) / ((temp - offset) + 237.3) * 17.2694)) - ((0.61078 * exp(temp / (temp + 237.3) * 17.2694)) * RH / 100);
    lcd.print(VPD);
    lcd.print(" kPa VPD");
    timestats = millis();
    if (VPD <= 0.5 || VPD >= 1.0) {
      
    } 
  }
  /*   DEBUG INFO OF THE SENSORS
    int Soilsens1;
    int Soilsens2;
    Soilsens1 = map(analogRead(SPin1), Calvalues.HighCal1, Calvalues.LowCal1, 100, 0);
    Soilsens2 = map(analogRead(SPin2), Calvalues.HighCal2, Calvalues.LowCal2, 100, 0);
    lcd.print("S1: ");
    lcd.print(Soilsens1);
    lcd.print(", ");
    lcd.print(analogRead(SPin1));
    lcd.setCursor(0, 1);
    lcd.print("S1: ");
    lcd.print(Soilsens2);
    lcd.print(", ");
    lcd.print(analogRead(SPin2));
    delay(1000);
    lcd.clear();
  */
}

//Measurements of Soil moisture sensors
void SoilMeasurement1(float* Soil1Data) {
  // Sensor measurement creating a total to later calculate average converted to %
  float sensorReadings[numReadings];  // array for raw measurements
  float percentArray[numReadings];    // array for raw converted to %
  float total = 0;

  for (int i = 0; i < numReadings; i++) {
    sensorReadings[i] = analogRead(SPin1);
    percentArray[i] = map(sensorReadings[i], Calvalues.HighCal1, Calvalues.LowCal1, 100, 0);
    total += percentArray[i];
    delay(1000);
  }
  float average = total / numReadings;

  // calculate standard deviation
  float sqDevSum = 0;
  for (int i = 0; i < numReadings; i++) {
    sqDevSum += (average - percentArray[i]) * (average - percentArray[i]);
  }
  float stDev = sqrt(sqDevSum / (numReadings - 1));

  // convert into an array, while also converting into int variable
  Soil1Data[0] = average;
  Soil1Data[1] = stDev;
}
void SoilMeasurement2(float* Soil2Data) {
  // Sensor measurement creating a total to later calculate average converted to %
  float sensorReadings[numReadings];  // array for raw measurements
  float percentArray[numReadings];    // array for raw converted to %
  float total = 0;
  for (int i = 0; i < numReadings; i++) {
    sensorReadings[i] = analogRead(SPin2);
    percentArray[i] = map(sensorReadings[i], Calvalues.HighCal2, Calvalues.LowCal2, 100, 0);
    total += percentArray[i];
    delay(500);
  }
  float average = total / numReadings;

  // calculate standard deviation
  float sqDevSum = 0;
  for (int i = 0; i < numReadings; i++) {
    sqDevSum += (average - percentArray[i]) * (average - percentArray[i]);
  }
  float stDev = sqrt(sqDevSum / (numReadings - 1));

  // convert into an array
  Soil2Data[0] = average;
  Soil2Data[1] = stDev;
}

//Logs data on different files depending on wether it is soil moisture or temperature
//Also logs the time next to the values
void LogData(byte type, float* DataArray) {
  //Select type of data to upload into one folder or another
  char filename[20];
  byte n;
  if (type == 1) {
    strcpy(filename, "SOILDATA.txt");
    n = 4;
  } else {
    strcpy(filename, "AIRDATA.txt");
    n = 2;
  }

  DataArray[n];
  // Open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  delay(300);
  File dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    // if the file isn't open, pop up an error:
    Serial.println("error opening .txt");
    while (1) {
      /*
        digitalWrite(RLED, HIGH);
        delay(100);
        digitalWrite(RLED, LOW);
        delay(100);
        */
    }
  }
  // if the file is available, write to it:
  else {
    //print hour and date
    dataFile.print(RTC.getHours());
    dataFile.print(":");
    dataFile.print(RTC.getMinutes());
    dataFile.print(":");
    dataFile.print(RTC.getSeconds());
    dataFile.print(", ");
    dataFile.print(RTC.getDay());
    dataFile.print("-");
    dataFile.print(RTC.getMonth());
    dataFile.print("-");
    dataFile.print(RTC.getYear());
    dataFile.print(", ");
    //print the data of the sensor
    for (int i = 0; i < n; i++) {
      dataFile.print(DataArray[i]);
      dataFile.print(", ");
      delay(10);
    }
    dataFile.println("");
    dataFile.close();
    delay(300);

    // print to the serial port that it was successful
    Serial.println("Data logged.");
  }
}

void PrintData(byte type, float* DataArray) { //Similar to LogData but on Serial, used mostly for DEBUG

  //Select type of data to upload into one folder or another
    const char* filename;
    byte n;
    if (type == 1) {
      n = 4;
    } else {
      n = 2;
    }
  DataArray[n];
  //print hour and date
    Serial.print(RTC.getHours());
    Serial.print(":");
    Serial.print(RTC.getMinutes());
    Serial.print(":");
    Serial.print(RTC.getSeconds());
    Serial.print(", ");
    Serial.print(RTC.getDay());
    Serial.print("-");
    Serial.print(RTC.getMonth());
    Serial.print("-");
    Serial.print(RTC.getYear());
    Serial.print(", ");
  //print the data of the sensor
    for (int i = 0; i < n; i++) {
      Serial.print(DataArray[i]);
      Serial.print(", ");
      delay(10);
    }
}

// Used to read DHT sensor data and convert into array
void ReadDHT22(float* AirData) {
  AirData[2];
  float hum;   //Stores humidity value
  float temp;  //Stores temperature value
    //Read data and store it to variables hum and temp
  hum = dht.readHumidity();
  temp = dht.readTemperature();

  // convert into an array
  AirData[0] = hum;
  AirData[1] = temp;
  return;
}

//Used when soil humidity is low. Starts water lvl sensor, initiates pump if enough water
void Watering1() {
  //Starts the water sensor and waits a bit for it to calibrate
    digitalWrite(WSensOn, HIGH);
    delay(5000);
    lcd.setCursor(0, 0);
  //Checks if it detects water, and if so it starts the pump for the delay time
    if(digitalRead(WSensIn) == HIGH) {
      lcd.print("Water lvl ok");
      delay(300);
      lcd.setCursor(0, 1);
      lcd.print("starting pump");
      digitalWrite(WPump, HIGH);
      delay(5000);
      lcd.clear();
      lcd.print("stopping pump");
      digitalWrite(WPump, LOW);
    }
  // If it doesnt detect water, it shows a message
    else {
      lcd.print("No water");
      delay(3000);
    }
  lcd.clear();
  lcd.print("turning OFF sensor");
  digitalWrite(WSensOn, LOW);
}

void ManualSoilCal() {
  // Wet phase of calibration
  bool Stable = 0;
  Serial.println("Performing calibration");
  delay(1000);
  Serial.println("Introduce sensors in water");
  for (byte n = 0; n < 3; n++) {
    /*
        digitalWrite(RLED, HIGH);
        delay(800);
        digitalWrite(RLED, LOW);
        delay(800);
        */
  }
  unsigned int prevTimeCal = millis();
  while (!Stable || millis() - prevTimeCal <= 15000) {
    int Analog1 = analogRead(SPin1);
    int Analog2 = analogRead(SPin2);

    if (Analog1 < Calvalues.HighCal1) {
      Calvalues.HighCal1 = Analog1;
      Serial.println(analogRead(SPin1));
    }
    if (Analog2 < Calvalues.HighCal2) {
      Calvalues.HighCal2 = Analog2;
      Serial.println(analogRead(SPin2));
    }
    if (Analog1 >= Calvalues.HighCal1 || Analog2 >= Calvalues.HighCal2) {
      Stable = 1;
    }
    delay(200);
  }



  //Dry Phase
  Serial.println("Dry the sensors now: ");
  for (byte n = 0; n < 3; n++) {
    /*
        digitalWrite(RLED, HIGH);
        delay(800);
        digitalWrite(RLED, LOW);
        delay(800);
        */
  }
  Stable = 0;
  prevTimeCal = millis();
  while (!Stable || millis() - prevTimeCal <= 1500) {
    int Analog1 = analogRead(SPin1);
    int Analog2 = analogRead(SPin2);

    if (Analog1 > Calvalues.LowCal1) {
      Calvalues.LowCal1 = Analog1;
    }
    if (Analog2 > Calvalues.LowCal2) {
      Calvalues.LowCal2 = Analog2;
    }
    if (Analog1 <= Calvalues.LowCal1 || Analog2 <= Calvalues.LowCal2) {
      Stable = 1;
    }
    delay(200);
  }

  //print new calibration values
  delay(2000);
  Serial.println("Ending calibration");
  Serial.println("New Upper-Lower values");
  Serial.print("S1: ");
  Serial.print(Calvalues.HighCal1);
  Serial.print(" and ");
  Serial.print(Calvalues.LowCal1);
  Serial.print(", S2: ");
  Serial.print(Calvalues.HighCal2);
  Serial.print(" and ");
  Serial.println(Calvalues.LowCal2);
  delay(1000);
}
void CallCalibr() {  //UNUSED requires more work and was useful with display
  /*
  Serial.println("Calibrate now?");

  int count = 0;
  int maxcount = 7;
  bool buttonState = false;
  char buffer[1];    // Choose an appropriate buffer size

  while (buttonState == false && count < maxcount) {
    if (digitalRead(ButtNum) == HIGH){
      buttonState = true;
      Serial.println(" Button pressed");
    }
    count++;
    Serial.print(count);
    Serial.print("...");
    delay(1000);
  }
  if (buttonState == true) {
    lcdserialln("Calibrating...",0);
    delay(1000);
    ManualSoilCal();
    }

  if (count == maxcount && buttonState == LOW) {
    lcd.clear();
    lcdserialln("Calibration skipped.",0);
    delay(2000); 
  }

    delay(1000);
  */
}
