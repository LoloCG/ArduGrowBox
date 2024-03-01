/* 
    -This is intended for Arduino NANO 3.0 typically sold in aliexpres
    --------------------------------------------------------------------------------------------------------------------------*/
#define Version "1.2.0"

//Configuration
    //Options
        #define Atmega328

        #define ExtendVerbose           //Disables texts and functions that are less than necessary
        #define FanControl              //Enables usage of all Fan related things. Disable when external Fan usage
        #define SoilSensCalibr        //Enables function to calibrate soil moisture sensor in the menu
        //#define Datalog                 //Enables logging data into SD. Requires RTC module present also
        #define AutoWater               //Enables water pump to water the plant. (TODO: If disabled, low humidity alert is displayed instead)
        #define SoilSensNoise           //Used when Soil humidity sensor has high noise/signal ratio. Allows removal of outliers and increased measurements.

        //#define ClockSet              //Used to set RTC time. Enable once to set time, then flash the MCU with option disabled.

    //Values
        #define SoilHumWatering 70              //Threshold of soil humidity to water plant. Depends on sensor, plant and soil.
        #define TimeSoilMeasurement 1800000     //Time between Soil measurements (by default: 1800000 = 30mins)
        #define TimeAirMeasurement 900000       //Time between Air measurements (by default: 900000 = 15mins)
//Libraries
	#include <Arduino.h>
    #include <Wire.h> 
    #include <LiquidCrystal_I2C.h>    
    #include <DHT.h>                    //for DHT22 temperature/humidity sensor
        
    #ifdef Datalog
    #include <SD.h>                     //for SD card datalogger
    #include "RTClib.h"                 //for RTC Adafruit's library
    #include <SPI.h>                    //required for SD's SPI communication
    #endif
    
    #if defined(Atmega328) && defined(SoilSensCalibr)
    #include <EEPROM.h>
    #endif
    
    #ifdef FanControl
    #include <SparkFun_TB6612.h>        //for the motor driver
    #endif


//Constants    
    //millis timers
        unsigned long AirMesmillis = millis();
        unsigned long SoilMesmillis = millis();
        unsigned long OtherMillis = millis();       //Used as general purpose
        unsigned long ScreenOff = millis();
        unsigned long ButtonMillis = millis();

    //Constants for the LCD and keypad
        #define ARROW_LEFT  0x7E
        LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
        boolean MenuOn;
        boolean PowerSave;          //Turns the display off if true

        #define KeypadPin A6        //Pin for the keypadMatrix

    //Constants for Sensors
        //-------------------------------FOR DHT22 SENSOR-------------------------------//
        #define DHTPIN 5           // Digital pin used
        #define DHTTYPE DHT22      // DHT 22  (AM2302)
        DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor for normal 16mhz Arduino

        //-------------------------------FOR SOIL SENSOR-------------------------------//
        #define SPin1 A0           // A pin where Soil sensor is connected
                
        struct SoilCal {
            unsigned int HighCal1 : 10;
            unsigned int LowCal1 : 10;
        } Calvalues;

        int SoilAdress = 0;           //Adress of the EEPROM where calibration values will be stored
        
    //RTC and SD constants
        #ifdef Datalog
        RTC_DS3231 rtc;                             //The type of RTC used
        const byte chipSelect = 4;                  //CS pin for SD card reader
        #endif
    //Constants for Fans and TB6612 motor driver
        #ifdef FanControl
        byte FanSpeed;

        const int offsetA = 1;
        const int offsetB = 1;

        #define STBY 11                             //Not actually used here... DEBUG: but requires a pin to be used. Doesnt seem to affect anything...
        //pins for the motor1
            #define AIN1 7
            #define AIN2 6
            #define PWMA 9
        //pins for motor2
            #define BIN1 8
            #define BIN2 A1
            #define PWMB 10

        Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
        Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
        #endif

//Function references
    void ReadDHT22(float* AirData);
    void PrintData(byte Datasize, float* DataArray);
    void LogData(byte FileNum, byte Datasize, float* DataArray);
    void SoilMeasurement(float* SoilData);
    byte MainMenu();
    byte ButtonPress();
    #ifdef FanControl
    void AutoFanControl(float* AirData);
    #endif
    //void ManualFanSet();      //Not yet added the option
    void HumidityCheck();
    void ManualSoilCal();
    #ifdef AutoWater
    void PumpWater();
    #endif
    void ErrorMessages(byte ErNum);
    
void setup() { 
    //initiate serial
        Serial.begin(9600);
        while (!Serial) {}

    //Initiate LCD screen
        while (lcd.begin(16, 2, LCD_5x8DOTS) != 1) {    //colums, rows, characters size
            Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
            delay(5000);   
        }

        lcd.print("Initiating...");
        lcd.setCursor(0, 1);
        lcd.print("Version: ");
        lcd.print(Version);
        delay(1500);
        
        pinMode(KeypadPin,INPUT);
        MenuOn = false;
        PowerSave = false;          //Powersave mode is set as off at setup
        ScreenOff = millis();
    //start the SD card
        // see if the card is present and can be initialized:
        #ifdef Datalog
        if (!SD.begin(chipSelect)) {
            ErrorMessages(1);
        }
        #if !defined(ExtendVerbose)
        lcd.clear();
        lcd.print("Initiating...");
        lcd.setCursor(0, 1);
        lcd.print("SD card present");
        delay(1500);
        #endif
        #endif
    
    //start RTC and set time
        #ifdef Datalog
        if (! rtc.begin()) {
            ErrorMessages(2);
        }
        
        #if !defined(ExtendVerbose)
        lcd.clear();
        lcd.print("Initiating...");
        lcd.setCursor(0, 1);
        lcd.print("RTC: ");
        lcd.print(rtc.lostPower() == 1 ? "Has stopped" : "OK");       //Shows if the RTC has stopped at any time (no battery)
        #endif
        delay(1500);
        #ifdef ClockSet
        //Uncomment the following line to set the time when the code is uploaded
            //If left commented, Time will be set at the upload time everytime the MCU restarts
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        #endif 
        #endif
    //Setup DHT22 and soil humidity Sensors
        dht.begin();
 


    //Show the date and time set
        #ifdef Datalog
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Date is set at:");
        DateTime now = rtc.now();
        lcd.setCursor(0, 1);
        //Show Date
            lcd.print(now.day(), DEC);
            lcd.print('/');
            lcd.print(now.month(), DEC);
            lcd.print('/');
            lcd.print(now.year(), DEC);
        //Show time
            lcd.print(' ');
            lcd.print(now.hour(), DEC);    
            lcd.print(':');
            lcd.print(now.minute(), DEC);
        delay(2000);
        #endif
    //Set fan control parameters
        #ifdef FanControl
        pinMode(10,OUTPUT);
        pinMode(9, OUTPUT);

        //Set PWM frequency 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
            TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
            TCCR1B = (1 << CS10) | (1 << WGM13);
            ICR1 = 320;
            OCR1A = 160; // 50% duty cycle
            OCR1B = 160;

        pinMode(BIN2, OUTPUT);      //Used for pin BIN2 of the motor driver, as it requires the use of A1 as digital pin 
        //motor1.drive(10 * 2.55);    //Auto start the fan extractor at 10% speed
        #endif
        #ifdef AutoWater
        pinMode(A2, OUTPUT);        //AnalogPin 2 is used for water pump
        #endif 
    lcd.clear();
    OtherMillis = millis() + 5000;
}

void loop() {
    
    if (millis() - OtherMillis >= 5000) {      //Temp+Hum sensor measurement real time 5000 = 5s
        //Obtain RH and Temp values
            float AirData[3];      
            ReadDHT22(AirData);
        //Print into screen if menu is not open, or powersave mode off
            if (!MenuOn && !PowerSave){
                lcd.clear();
                lcd.print(AirData[0], 1);
                lcd.print(" RH, ");
                lcd.print(AirData[1], 1);
                lcd.print(" C");

                lcd.setCursor(2, 1);
                lcd.print(AirData[2],2);
                lcd.print(" kPa VPD");
                OtherMillis = millis();
                delay(2500);
            }

        //Send values to Fan control function
        #ifdef FanControl
            AutoFanControl(AirData);
        #endif

    }

    if (millis() - SoilMesmillis >= TimeSoilMeasurement) {      //Soil sensor measurement 1800000 = 30 mins
        //Takes data from the function
            float SoilData[1];
            SoilMeasurement(SoilData); 

        //Logs data into SD card and prints it in the screen
            #ifdef Datalog
            LogData(1, 1, SoilData);
            delay(2000);
            #if !defined(ExtendVerbose)
            PrintData(2, SoilData);  
            delay(2000);
            #endif
            #endif
        
        #ifdef AutoWater
        //checks if humidity was lower than a threshold
            if (SoilData[0] <= SoilHumWatering) { 
                PumpWater();
                #ifdef Datalog
                LogData(3, 1, SoilData);
                #endif
            }
        #endif 
        //restarts the counting for next soil humidity datalog
            SoilMesmillis = millis();
    }
    
    #ifdef Datalog           //Right now, function only serves datalogging purposes
    if (millis() - AirMesmillis >= 900000) {       //Temp+Hum sensor measurement 900000 = 15 min
        //obtain values
            float AirData[3];      
            ReadDHT22(AirData);
        
        
        //print data + send to SD card logging
            LogData(2, 3, AirData);
            delay(2000);
            #if !defined(ExtendVerbose)
            PrintData(2, AirData);  
            delay(2000);
            #endif
        

        //Reset the timer for next 15 mins
            AirMesmillis = millis();
    }
    #endif

    if (analogRead(KeypadPin) <= 1000) {                   //used to detect button press and start the menu
        ButtonMillis = millis();
        PowerSave = false;
        lcd.displayOn();

        while (analogRead(KeypadPin) <= 1000){         //wait for the button to stop being pressed
            delay(1);
        }
        
        if (millis() - ButtonMillis >= 750) {   //if 2s has passed then go to the menu
            MenuOn = true;
            byte MenuOutput;
            MenuOutput = MainMenu();

            delay(200);
            switch (MenuOutput) {
                case 1:                     //Manual Soil Calibration 
                    #ifdef SoilSensCalibr
                    ManualSoilCal();
                    #endif
                    #ifndef SoilSensCalibr
                    ErrorMessages(4);
                    #endif
                break;

                case 2:                   
                    HumidityCheck();
                break;

                case 3: 
                    #ifdef FanControl
                    //ManualFanSet();
                    lcd.print("Not available");
                    #endif
                    #ifndef FanControl
                    ErrorMessages(4);
                    #endif
                break;
                

            }
        }

        MenuOn = false;
        ButtonMillis = millis();                //added some time to prompt the air values
        ScreenOff = millis();                   //Restarts the counting or LCD backlight off mode
    }

    if (millis() - ScreenOff >= 60000 && !PowerSave) {      //Used to turn off the screen after not touching any button
        PowerSave = true;
        lcd.displayOff();
    } 
}

#ifdef FanControl
void AutoFanControl(float* AirData){      //Fan control with VPD   
    // Determine the new fan speed based on AirData
       byte NewFanSpeed = map(AirData[1], 25, 35, 0, 70);

    //Check if the fan speed needs to change, and print values + change speed   
        if (abs(FanSpeed - NewFanSpeed) > 2) {
            FanSpeed = NewFanSpeed;
            lcd.clear();
            lcd.setCursor(3, 0);
            lcd.print("Fan ");
            lcd.print(FanSpeed);
            lcd.print("%");
            motor1.drive(FanSpeed * 2.55); // Set the new fan speed
        }
}
#endif

//TODO: maybe change from float to int, to reduce memory consumption
void SoilMeasurement(float* SoilData) {             //Measurements of Soil moisture sensor

    //Retrieve from EEPROM old values
        SoilCal GetCal1;
        EEPROM.get(SoilAdress, GetCal1);

    //Captures sensor data and transforms into %
        #ifdef SoilSensNoise
        byte ReadNum = 5;
        #endif
        #ifndef SoilSensNoise
        byte ReadNum = 3;
        #endif

        float sensorReadings[ReadNum];  // array for raw measurements
        float percentArray[ReadNum];    // array for raw converted to %
        float total = 0;
        for (int i = 0; i < ReadNum; i++) {
            sensorReadings[i] = analogRead(SPin1);
            percentArray[i] = map(sensorReadings[i], GetCal1.HighCal1, GetCal1.LowCal1, 100, 0);
            total += percentArray[i];
            delay(500);
        }

    #ifdef SoilSensNoise
    //Removes data from the array of % that deviates by 5
        float average = 0;
        byte ValidCount = 0;
        for (byte i = 0; i < ReadNum; i++) {
            if (abs(percentArray[i] - (total/ReadNum)) <= 5) {
                average += percentArray[i];
                ValidCount++;
            }
        }
        average /= ValidCount;
    
    /* DEBUG:   Used to display readings for 1 min to ensure sensor works
    
        OtherMillis = millis();
        while(millis() - OtherMillis <= 60000){
            for (int i = 0; i < 4; i++) {
                sensorReadings[i] = analogRead(SPin1);
                percentArray[i] = map(sensorReadings[i], GetCal1.HighCal1, GetCal1.LowCal1, 100, 0);
                total += percentArray[i];
                delay(500);
            }
            lcd.clear();
            lcd.setCursor(0,0);
            for (byte i = 0; i < 4; i++) {
                lcd.print(percentArray[i],0);
                lcd.print(" ");
            }
            lcd.setCursor(0,1);
            for (byte i = 0; i < 4; i++) {
                lcd.print(analogRead(SPin1));
                lcd.print(" ");
            }
            delay(2000);
        }
    */

    #endif

    // convert into an array
        SoilData[0] = average;
}

void ReadDHT22(float* AirData) {                //Function to measure air temp. and humidity
    //Read data and store it to variables hum and temp
        float RH;   //Stores humidity value
        float temp;  //Stores temperature value
        RH = dht.readHumidity();
        temp = dht.readTemperature();
    
    //Calculate VPD
        byte offset = 1;    //Offset of leaf temperature difference from air temperature. 
                            //It is in negative! 1 = -1 ºC difference in leaf temperature!
        float VPD = (0.61078 * exp((temp - offset) / ((temp - offset) + 237.3) * 17.2694)) - ((0.61078 * exp(temp / (temp + 237.3) * 17.2694)) * RH / 100);
        
    // convert into an array
        AirData[0] = RH;
        AirData[1] = temp;
        AirData[2] = VPD;
        return;
}

#ifdef Datalog
void LogData(byte FileNum, byte Datasize, float* DataArray) {
    //Select folder based on FileNum
        char filename[20]; //TODO: not sure if there is a way to optimize
        if (FileNum == 1) {
            strcpy(filename, "SOILDATA.txt");
        } 
        else if (FileNum == 2) {
            strcpy(filename, "AIRDATA.txt");
        }
        //TODO: save the waterings to the same file as SOIL DATA with a boolean value 1/0... 
        else if (FileNum == 3) {
            strcpy(filename, "WATERING.txt");
        }

       
    //Open the file. Only one file can be open at a time, so you have to close this one before opening another.
        File dataFile = SD.open(filename, FILE_WRITE);
        delay(300);         //Wait a little for file to open
        if (!dataFile) {    //If the file isn't open, pop up an error:    
            ErrorMessages(3);
        }
        
    //If the file is available, write to it:
        else {
            //print hour and date
                DateTime now = rtc.now();
                //Date
                    dataFile.print(now.day(), DEC);
                    dataFile.print('/');
                    dataFile.print(now.month(), DEC);
                    dataFile.print('/');
                    dataFile.print(now.year(), DEC);
                //time
                    dataFile.print(",");
                    dataFile.print(now.hour(), DEC);    
                    dataFile.print(':');
                    dataFile.print(now.minute(), DEC);
                    dataFile.print(",");
            //print the data of the sensor
                for (int i = 0; i < Datasize; i++) {
                    dataFile.print(DataArray[i]);
                    dataFile.print(", ");
                    delay(10);
                }
                dataFile.println("");
                dataFile.close();
                delay(300);
        }
}
#endif 

void ErrorMessages(byte ErNum) {
    PowerSave = false;
    lcd.displayOn();
    lcd.clear();
    boolean Freeze = false;

    #ifdef Datalog    
    if (ErNum == 1) {          //SD card not present/failed at start
        lcd.print("SD card failed,");
        lcd.setCursor(0, 1);
        lcd.print("or not present");
        Freeze = true;
    }

    else if (ErNum == 2) {               //RTC not present/failed at start
        lcd.print(" RTC not found");
        lcd.setCursor(0, 1);
        lcd.print("Check and reset");
        Freeze = true;   
    }

    else if (ErNum == 3) {          //Unable to open SD card file 
        lcd.print("cannot open .txt");
        lcd.setCursor(0, 1);
        lcd.print(" Check SD card");
        Freeze = true;         
    }
    #endif
    if (ErNum == 4) {
        lcd.print(" Option Disabled");
    }

    //Freezes controller indefinetly or delays to display error
        if (Freeze == true){
            while (1) {}
        } else { 
            delay(3000);
        }
    
}

byte ButtonPress() {
        //Loop that senses duration of button press to filter noise from signal.
            //Waits for button press, starts timer and captures analog value, 
            // resumes when button is no longer pressed, exits the loop selecting 
            // the type of button press.
            //boolean LongPress;
            int val;
            boolean Noise = true;
            while (Noise == true) {
                while (analogRead(KeypadPin) > 1000) {		
                    delay(1);
                } 
                ButtonMillis = millis();
                val = analogRead(KeypadPin);
            	while (analogRead(KeypadPin) < 1000) {
			        delay(1);
		        }

                if (millis() - ButtonMillis >= 50 && millis() - ButtonMillis < 1000) {
                    //LongPress = false;
                    Noise = false;
                } else if (millis() - ButtonMillis > 1000) {
                    //LongPress = true;
                    Noise = false;
                }
            }
            
	//saves analog value and with a loop finds closest threshold to set it to the determined key
	    //Sets the values for analog thresholds and keypad 
		int KeyThresholds[3] = {540, 690, 775};
		byte keypad[3] = {0,1,2};
		byte keypress;

		for (int i = 0; i < 3; i++) {					//loop 3 times increasing i
			if (abs(val - KeyThresholds[i]) < 30) {		//If analog is in threshold, select key
				keypress = keypad[i];
			}
		}
        /*
         //DEBUG
        lcd.clear();
        lcd.print(keypress);
        lcd.print(", ");
        lcd.print(val);
        delay(1000);
        */
		return keypress;
        //TODO: 
        //return LongPress;
}

byte MainMenu() {       //Shows the main menu, retuens a byte corresponding to the option selected
    const char* MainMenuOptions[] = {
        "Return",          // Option 0
        "Soil H% calib.",  // Option 1
        "Soil Humidity",    // Option 2
        "Set Fan Speed"
    };
    
    byte MaxOptions = 3;
	byte Options = 0;
	boolean ExitMenu = false;
    byte MenuOutput;    

	while (!ExitMenu)  { 
        if (Options == MaxOptions - 1) {
            lcd.clear();   
            lcd.setCursor(1, 0);
            lcd.print(MainMenuOptions[Options - 1]);
            lcd.setCursor(0, 1);
            lcd.write(ARROW_LEFT);
            lcd.print(MainMenuOptions[Options]);
        }
        else { 
            lcd.clear();   
            lcd.setCursor(0, 0);
            lcd.write(ARROW_LEFT);
            lcd.print(MainMenuOptions[Options]);
            lcd.setCursor(1, 1);
            lcd.print(MainMenuOptions[Options + 1]);
        }

        delay(300);
		byte keypress;
		keypress = ButtonPress();           //Returns the key that was pressed
    
        switch (keypress) {
            case 0:                     //Enter button
            	if (Options == 0){
                lcd.clear();
                lcd.print("Returning...");
                delay(1500);
                MenuOutput = Options;
                } 
                else {
                    MenuOutput = Options;
                }
                ExitMenu = true;
            break;
    
            case 1:                     //Up key
                if (Options > 0) {
                    Options--;
                }
                else {
                    Options = 0;
                }
            break;
            
            case 2:                     //Down key/blue
                if (Options < 2) {
                    Options = Options + 1;
                }
                else {
                    Options = MaxOptions - 1;
                }
            break;
        }
    }
	lcd.clear();
    return MenuOutput;
}

#ifdef SoilSensCalibr
void ManualSoilCal() {
    //Retrieve from EEPROM old values
        SoilCal GetCal1;
        EEPROM.get(SoilAdress, GetCal1);

    // Print the saved calibration values
        lcd.clear();
        lcd.print("Current values:");
        lcd.setCursor(0,1);
        lcd.print("U: ");
        lcd.print(GetCal1.LowCal1);
        lcd.print(", L: ");
        lcd.print(GetCal1.HighCal1);
        delay(2000);
        
        lcd.setCursor(0,0);
        lcd.print("Calibrate?      "); 
        byte keypress;
        keypress = ButtonPress();

    //return if pressing other than center
        if (keypress != 0) {           
            lcd.print("Returning...");
            delay(1000);
            return;
        }

    //start calibration sequence
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Put in Dry soil");
        lcd.setCursor(0, 1);
        lcd.print(" Then wet soil");
        delay(4000);
    
    //first, it sets the sensor to whatever value is now 
        int Analog1 = analogRead(SPin1);            
        Calvalues.LowCal1 = Analog1;
        Calvalues.HighCal1 = Analog1;
        bool Stable = 0;
    
        lcd.clear();
        lcd.print("Press when calbr");
        lcd.setCursor(0, 1);
        lcd.print("is completed...");
        delay(3000);

    //Calibration Loop
        while (Stable == 0) {
            Analog1 = analogRead(SPin1);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(" U: ");
            lcd.print(Calvalues.LowCal1);
            lcd.print(", L: ");
            lcd.print(Calvalues.HighCal1);
            lcd.setCursor(0, 1);
            lcd.print("  Actual: ");
            lcd.print(Analog1);

            if (Analog1 > Calvalues.LowCal1) {
                Calvalues.LowCal1 = Analog1;
            }
            if (Analog1 < Calvalues.HighCal1) {
                Calvalues.HighCal1 = Analog1;
            }
            if (analogRead(KeypadPin) <= 1000) {
                Stable = 1;
            }
            delay(500);
        }
        delay(3000);

    //Ask and Save into EEPROM
        lcd.setCursor(0,0);
        lcd.print("Save new values?"); 
        keypress = ButtonPress();

        if (keypress == 0) {           
            #if defined(Atmega328) && defined(SoilSensCalibr)
            SoilCal NewCal1 = { Calvalues.HighCal1, Calvalues.LowCal1 };
            EEPROM.put(SoilAdress, NewCal1);
            #endif
            lcd.print("values saved"); 
            delay(2000);
        }

}
#endif

#ifdef AutoWater
void PumpWater() {
    lcd.clear();
    lcd.print(" Watering plant");
    delay(500);

    boolean WaterStop = false;
    OtherMillis = millis();
    digitalWrite(A2, HIGH);
    while (!WaterStop) {
        float SoilData[1];
        SoilMeasurement(SoilData);
        delay(5000);
        if (SoilData[0] >= 95 || millis() - OtherMillis >= 60000) {
            WaterStop = true;
        }
    }
    digitalWrite(A2, LOW);

    #if !defined(ExtendVerbose)
    lcd.clear();
    lcd.print(" Plant watered");
    delay(1000);
    #endif
}
#endif

void HumidityCheck() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Measuring soil...");

    float SoilData[1];
    SoilMeasurement(SoilData); 

    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(SoilData[0],0);
    lcd.print(" % humidity");

   
    #ifdef AutoWater
    if (SoilData[0] <= SoilHumWatering + 5) {                //TODO: this watering is not saved in the .txt file!
        OtherMillis = millis();
        lcd.setCursor(0, 1);
        lcd.print("Enter to water...");

        byte keypress;
		keypress = ButtonPress();

        if (keypress == 0) {            //If enter key is pressed
            PumpWater();
		} 
        else if (keypress != 0) {       //TODO: this is not a definitive solution. It gets stuck until a button is pressed
            return;
        }
         
    } else {
        delay(2000);
    }
    #endif

    #ifndef AutoWater
    delay(3000);
    #endif
}

#if !defined(ExtendVerbose)
void PrintData(byte Datasize, float* DataArray) {
    //When the function is called, it includes the size of data array in a byte number format, and the array with data to save
    //print hour and date
        lcd.clear();
        lcd.setCursor(0, 0);
        DateTime now = rtc.now();
        //Show Date
            lcd.print(now.day(), DEC);
            lcd.print('/');
            lcd.print(now.month(), DEC);
            lcd.print('/');
            lcd.print(now.year(), DEC);
        //Show time
            lcd.print(' ');
            lcd.print(now.hour(), DEC);    
            lcd.print(':');
            lcd.print(now.minute(), DEC);

    //print the data of the sensor
        lcd.setCursor(0, 1);
        for (int i = 0; i < Datasize; i++) {
            lcd.print(DataArray[i]);
            lcd.print(", ");
            delay(10);
        }
}
#endif

/*-----------------------------------   UNUSED CODE ---------------------------------------------
    LEFT HERE FOR FUTURE USE OR WHATEVER

    //this was used for compilatign multiple sensors Data into same array to store them all together.

        float SoilIntArray[4];
        for (int i = 0; i < 2; i++) {
            SoilIntArray[i] = Soil1Data[i];
            SoilIntArray[i + 2] = Soil2Data[i];
        }
    
    //this was used for calculating standard deviation from the 3 values of soil measurement %
         
        float sqDevSum = 0;
        for (int i = 0; i < 3; i++) {
            sqDevSum += (average - percentArray[i]) * (average - percentArray[i]);
        }
        float stDev = sqrt(sqDevSum / (3 - 1));

        #ifdef SoilSensNoise
        lcd.setCursor(3, 1);
        lcd.print(SoilData[1],2);
        lcd.print(" StDev");
        #endif
        
*/