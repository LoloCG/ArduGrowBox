/* 
    -This is intended for Arduino NANO 3.0 typically sold in aliexpres
    RAM:   [========  ]  82.0% (used 1679 bytes from 2048 bytes)
    Flash: [========  ]  79.3% (used 24350 bytes from 30720 bytes)

    -Basic Functionalities
        [X]-Exhaust fan speed control
        [ ]-Intake fan speed control
        [X]-Temp, Hum%, VPD display
        [?]-Soil humidity display and alert
        [X]-Datalogging to SD card
        [ ]-Day/Night cycle fan stop

    -Optional functionalities
        [ ]-Autowatering
    --------------------------------------------------------------------------------------------------------------------------*/
#define Version "1.2.2"

//Configuration
    //Options
        #define Atmega328
        
        #define FanControl              //Enables usage of all Fan related things. Disable when external Fan usage
        #define SoilSensCalibr        //Enables function to calibrate soil moisture sensor in the menu
        #define Datalog                 //Enables logging data into SD. Requires RTC module present also
        //#define AutoWater         //Enable water pump when low soil humidity. If disabled, alert is displayed instead.
    
        //#define SerialPrint       //Used for debug purposes
        //#define ClockSet              //Used to set RTC time. Enable once to set time, then flash the MCU with option disabled.

    //Values
        #define SoilHumWatering 60              //Threshold of soil humidity to water plant. Depends on sensor, plant and soil.
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
    
    #if defined(Atmega328)
    #include <EEPROM.h>
    #endif
    
    #ifdef FanControl
    #include <SparkFun_TB6612.h>        //for the motor driver
    #endif


//Constants    
    //General
        boolean LowSoilHAlert;

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

        const int SoilAdress = 0;           //Adress of the EEPROM where calibration values will be stored
        
    //RTC and SD constants
        #ifdef Datalog
        RTC_DS3231 rtc;                             //The type of RTC used
        const byte chipSelect = 4;                  //CS pin for SD card reader
        #endif
    //Constants for Fans and TB6612 motor driver
        #ifdef FanControl
        byte FanSpeed;
        byte FanSpeedAdjst; 
    
        const int offsetA = 1;
        const int offsetB = 1;

        #define STBY A3                             //Not actually used here... DEBUG: but requires a pin to be used. Doesnt seem to affect anything...
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
    void ErrorMessages(byte ErNum);
    void LogData(byte FileNum, byte Datasize, float* DataArray);
    void GetSoilData(float* SoilData, bool ReturnAnalog);
    byte MainMenu();
    byte ButtonPress();
    void OpenSecMenu(byte MenuOutput);
    void SoilMeasurement();

    #ifdef FanControl
    void AutoFanControl(float* AirData);
    void FanSpeedAdjust();
    #endif

    void HumidityCheck();
    void ManualSoilCal();

    #ifdef AutoWater
    void PumpWater();
    #endif

    #ifdef SerialPrint
    void PrintData(byte Datasize, float* DataArray);
    #endif

    
void setup() { 
    //initiate serial
        Serial.begin(9600);
        while (!Serial) {}

        #ifdef SerialPrint
            Serial.println(F("Initiating Program"));
        #endif
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
        #ifdef SerialPrint
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
        
        #ifdef SerialPrint
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
    //Set fan control and water pump parameters
        #ifdef FanControl
        pinMode(10, OUTPUT);
        pinMode(9, OUTPUT);

        //Set PWM frequency 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
            TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
            TCCR1B = (1 << CS10) | (1 << WGM13);
            ICR1 = 320;
            OCR1A = 160; // 50% duty cycle
            OCR1B = 160;
        
        pinMode(BIN2, OUTPUT);      //Used for pin BIN2 of the motor driver, as it requires the use of A1 as digital pin 
        //motor1.drive(10 * 2.55);    //Auto start the fan extractor at 10% speed
        
        FanSpeedAdjst = 10;            //in a byte scale of 0-255, 128 is ~50%

        #endif


        #ifdef AutoWater
        pinMode(A2, OUTPUT);        //AnalogPin 2 is used for water pump
        #endif
        #ifndef AutoWater
        LowSoilHAlert = false;
        #endif
        
    lcd.clear();
    OtherMillis = millis() + 5000;
}

void loop() {
    #ifndef AutoWater
    if (LowSoilHAlert) {
        lcd.clear();
        lcd.print(F("Soil humid. Low"));
        lcd.setCursor(0,1);
        lcd.print(F("Water plant..."));
        MenuOn = true;
    }
    #endif

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
        SoilMeasurement();
        //restarts the counting for next soil humidity datalog
            SoilMesmillis = millis();
    }
    
    #ifdef Datalog                          //Right now, function only serves datalogging purposes
    if (millis() - AirMesmillis >= 900000) {       //Temp+Hum sensor measurement 900000 = 15 min
        //obtain values
            float AirData[3];      
            ReadDHT22(AirData);
        
        
        //print data + send to SD card logging
            LogData(2, 3, AirData);
            delay(2000);
            #ifdef SerialPrint
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
            
            OpenSecMenu(MenuOutput);
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

/*====================================================================================
________________________________________SENSORS_______________________________________
======================================================================================*/

void GetSoilData(float* SoilData, bool ReturnAnalog) {             //Measures the soil humidity
    #ifdef SerialPrint  
    Serial.println(F("------------------------"));
    Serial.println(F("measurement commenced:"));
    #endif

    //Main soil sensor reading loop
        byte ReadNum = 8;
        boolean exit = false;
        int AnalogAvgClean = 0;
        while (!exit) {
            //Takes analog values of soil sensor and joins them in an array + sums into "AnalogAvg"
                #ifdef SerialPrint
                Serial.println("Analog values: ");
                #endif
                int sensorReadings[ReadNum];
                int AnalogAvg = 0;
                for (int i = 0; i < ReadNum; i++) {
                    sensorReadings[i] = analogRead(SPin1);
                    AnalogAvg += sensorReadings[i];
                    
                    #ifdef SerialPrint
                    Serial.print(sensorReadings[i]);
                    Serial.print(", ");
                    #endif
                }
                AnalogAvg /= ReadNum;
                        
                #ifdef SerialPrint
                Serial.println("");
                Serial.print("average reading: ");
                Serial.println(AnalogAvg);
                #endif

            //Counts how many are within the threshold, then creates an average with those ("AnalogAvgClean")
                byte ValidCount = 0;
                
                for (byte i = 0; i < ReadNum; i++) {
                    if (abs(sensorReadings[i] - (AnalogAvg)) < ((AnalogAvg)*0.05)) {
                            AnalogAvgClean += sensorReadings[i];
                            ValidCount++;
                    }
                }
                AnalogAvgClean /= ValidCount;
                #ifdef SerialPrint
                Serial.print("Valid Count: ");
                Serial.println(ValidCount);
                Serial.print("Final Analog AVG: ");
                Serial.println(AnalogAvgClean);
                #endif
            
            //Loops if the count is less than half
                if (ValidCount >= ValidCount/2) {
                    exit = true;
                    #ifdef SerialPrint
                    Serial.println("Exit measurement loop");
                    #endif
                }
        }
    //Returns if Analog values required    
        if(ReturnAnalog == true) {
            SoilData[0] = AnalogAvgClean;
        } else {
    //Retrieve from EEPROM old values of Max and min
        SoilCal GetCal1;
        EEPROM.get(SoilAdress, GetCal1);
        
        #ifdef SerialPrint
        Serial.print("LowCal (0%): ");
        Serial.println(GetCal1.LowCal1);        //low humidity = higher number 
        Serial.print("HighCal (100%): ");
        Serial.println(GetCal1.HighCal1);       //high humidity = lower number
        #endif

    //Converts the Analog value into %
        byte offsetCal = 0.05;
        float SoilPercent = map(AnalogAvgClean, GetCal1.HighCal1*(1 + offsetCal), GetCal1.LowCal1*(1 - offsetCal), 100, 0);
        SoilData[0] = SoilPercent;


       #ifdef SerialPrint
        Serial.print("Final % value: ");
        Serial.println(SoilData[0]);
        Serial.println("------------------------");
        #endif
    }
}

void SoilMeasurement(){
    //Takes data from the function
        float SoilData[1];
        GetSoilData(SoilData, false);    


    //Logs data into SD card and prints it in the screen
        #ifdef Datalog
        LogData(1, 1, SoilData);
        delay(2000);
        #ifdef SerialPrint
        PrintData(2, SoilData);  
        delay(2000);
        #endif
        #endif
    
    
    //checks if humidity was lower than a threshold
        if (SoilData[0] <= SoilHumWatering) { 
            #ifdef AutoWater
            PumpWater();
            #ifdef Datalog
            LogData(3, 1, SoilData);
            #endif
            #endif
            #ifndef AutoWater
            LowSoilHAlert = true;
            #endif
        }   else {
            LowSoilHAlert = false;
        }
             
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
        float VPD = (0.6108 * exp((temp - offset) / ((temp - offset) + 237.3) * 17.269)) - ((0.6108 * exp(temp / (temp + 237.3) * 17.269)) * RH / 100);
        
    // convert into an array
        AirData[0] = RH;
        AirData[1] = temp;
        AirData[2] = VPD;
        return;
}

#ifdef SoilSensCalibr
void ManualSoilCal() {
    //Retrieve current values from EEPROM
        SoilCal GetCal1;
        EEPROM.get(SoilAdress, GetCal1);

    //Print the saved calibration values
        lcd.clear();
        lcd.print(F("Current values:"));
        lcd.setCursor(0,1);
        lcd.print("U: ");
        lcd.print(GetCal1.LowCal1);
        lcd.print(", L: ");
        lcd.print(GetCal1.HighCal1);
        delay(2000);
        
        lcd.setCursor(0,0);
        lcd.print(F("Calibrate now?")); 
        byte keypress;
        keypress = ButtonPress();

    //return if pressing other than center
        if (keypress != 0) {           
            ErrorMessages(5);
            return;
        }

    //display instructions
        #ifdef SerialPrint
            Serial.print("------------------------------------");
            Serial.println("Starting calibration...");
        #endif

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Put in Dry soil"));
        lcd.setCursor(0, 1);
        lcd.print(F(" Then wet soil"));
        //delay(3000);

        lcd.clear();
        lcd.print(F("Press when calbr"));
        lcd.setCursor(0, 1);
        lcd.print(F("is completed..."));
        delay(3000);

    //Set measurement as upper/lower

        float SoilData[1];
        GetSoilData(SoilData, true); 
        Calvalues.LowCal1 = SoilData[0];
        Calvalues.HighCal1 = SoilData[0];
        #ifdef SerialPrint
            Serial.println(F("Setting current values as upper and lower"));
            Serial.println(SoilData[0]);
        #endif
    //Calibration Loop
        bool Stable = false;
        while (!Stable) {
            GetSoilData(SoilData, true); 

            lcd.clear();
            lcd.print(" U: ");
            lcd.print(Calvalues.LowCal1);
            lcd.print(", L: ");
            lcd.print(Calvalues.HighCal1);
            lcd.setCursor(0, 1);
            lcd.print("  Actual: ");
            lcd.print(SoilData[0]);

            if (SoilData[0] > Calvalues.LowCal1) {
                Calvalues.LowCal1 = SoilData[0];
            }
            if (SoilData[0] < Calvalues.HighCal1) {
                Calvalues.HighCal1 = SoilData[0];
            }
            if (analogRead(KeypadPin) <= 1000) {
                Stable = true;
            }
            delay(250);
        }
        delay(3000);

    //Ask and Save into EEPROM
        lcd.clear();
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

void HumidityCheck() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Measuring soil..."));

    float SoilData[1];
    GetSoilData(SoilData, false); 

    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(SoilData[0],0);
    lcd.print(F(" % humidity"));

   
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

/*====================================================================================
________________________________________FANS/PUMP_________________________________________
======================================================================================*/

#ifdef FanControl
void AutoFanControl(float* AirData){                    //Fan control with VPD   
    // Determine the new fan speed based on AirData
       byte NewFanSpeed = map(AirData[1], 24, 35, 5, 95);
    
        //Check if the fan speed needs to change
        if (abs((FanSpeed) - (NewFanSpeed + FanSpeedAdjst)) > 2 && AirData[1] >= 24) {
            #ifdef SerialPrint
            Serial.println(F("--------------------------------"));
            Serial.print(F("AirData[1]:"));
            Serial.println(AirData[1]);
            Serial.print(F("NewFanSpeed:"));
            Serial.println(NewFanSpeed);
            Serial.print(F("FanSpeedAdjst:"));
            Serial.println(FanSpeedAdjst);
            Serial.print("abs((FanSpeed) - (NewFanSpeed + FanSpeedAdjst)) = ");
            Serial.println(abs((FanSpeed) - (NewFanSpeed + FanSpeedAdjst)));
            #endif
            if ((NewFanSpeed + FanSpeedAdjst) < 10  || AirData[1] < 24) {       
                FanSpeed = 0;
            } 

            else if ((NewFanSpeed + FanSpeedAdjst) >= 95) {
                FanSpeed = 95;
            }   
            
            else { 
                FanSpeed = NewFanSpeed + FanSpeedAdjst;
            }

        #ifdef SerialPrint
            Serial.print(F("Final fan speed:"));
            Serial.println(FanSpeed);
        #endif
    //Apply the speed and print the values
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Fan ");
        lcd.print((FanSpeed));
        lcd.print("%");
        motor1.drive((FanSpeed)); // Set the new fan speed
        }
}

void FanSpeedAdjust(){
    boolean ExitMenu = false; 
    while(!ExitMenu) {
        lcd.clear();
        lcd.print(F("Adjust fan speed: "));
        lcd.setCursor(5,1);
        //lcd.print("+");
        lcd.print(FanSpeedAdjst);
        lcd.print(" %");
        delay(100);
        
    	byte keypress;
		keypress = ButtonPress();           //Returns the key that was pressed

        switch (keypress){
        case 0:
            ExitMenu = true;
        break;

        case 1:
            if (FanSpeedAdjst > 95) {
                FanSpeedAdjst = 100;
            } else {
                FanSpeedAdjst += 5;
            }
        break;

        case 2:
            if (FanSpeedAdjst < 5) {
                FanSpeedAdjst = 0;
            } else {
                FanSpeedAdjst -= 5;
            }
        break;
        }
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
            GetSoilData(SoilData, false);
        delay(5000);
        if (SoilData[0] >= 90 || millis() - OtherMillis >= 30000) {
            WaterStop = true;
        }
    }
    digitalWrite(A2, LOW);

    #ifdef SerialPrint
    lcd.clear();
    lcd.print(" Plant watered");
    delay(1000);
    #endif
}
#endif
/*====================================================================================
________________________________________DISPLAY_______________________________________
======================================================================================*/

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

    if (ErNum == 5) {
        lcd.print("Returning...");
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

byte MainMenu() {       //Shows the main menu, returns a byte corresponding to the option selected
    const char* MainMenuOptions[] = {
        "Return",           //0
        "Soil H% calib.",   //1
        "Soil Humidity",    //2
        "Fan Speed Adjst"   //3
    };
    #ifdef SerialPrint
        Serial.println(F("Entering menu"));
    #endif
    byte MaxOptions = 4;        //!!! adjust this based on the number of menu options (counting nº0)
	byte Options = 0;           //this sets the starting menu option
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
            	if (Options == 0){      //if the menu was 0 (return) then go back
                lcd.clear();
                ErrorMessages(5);
                MenuOutput = Options;
                } 
                else {                  //else return with the menuoutput which is equal to the option number
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

void OpenSecMenu(byte MenuOutput){

    switch (MenuOutput) {
    case 1:                  
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
        FanSpeedAdjust();
        #endif

        #ifndef FanControl
        ErrorMessages(4);
        #endif
    break;

    }
}

/*====================================================================================
________________________________________SD CARD_______________________________________
======================================================================================*/

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

/*-----------------------------------   UNUSED CODE ---------------------------------------------
    LEFT HERE FOR FUTURE USE OR WHATEVER
*/