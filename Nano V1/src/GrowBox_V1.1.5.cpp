/* 
    -This is intended for Arduino NANO 3.0 typically sold in aliexpres
    --------------------------------------------------------------------------------------------------------------------------*/
#define Version "1.1.5a"

//Configuration
    #define LowMemoryMode                   //Disables texts and functions that are less than necessary
    #define FanControl                      //Enables usage of all Fan related things. Disable when external Fan usage
    //#define SoilSensCalibr                //Enables function to calibrate soil moisture sensor in the menu
    #define Datalogging                     //Enables logging data into SD. Requires RTC module present also

    #define SoilHumWatering 70              //Threshold of soil humidity to water plant. Depends on sensor, plant and soil.
    
    #define TimeSoilMeasurement 1800000     //Time between Soil measurements (by default: 1800000 = 30mins)
    #define TimeAirMeasurement 1800000      //Time between Air measurements (by default: 1800000 = 30mins)

//Libraries
	#include <Arduino.h>
    #include <Wire.h> 
    #include <LiquidCrystal_I2C.h>    
    #include <DHT.h>                    //for DHT22 temperature/humidity sensor
        
    #ifdef Datalogging
    #include <SD.h>                     //for SD card datalogger
    #include "RTClib.h"                 //for RTC Adafruit's library
    #include <SPI.h>                    //required for SD's SPI communication
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

    //Constants for the LCD
        #define ARROW_LEFT  0x7E
        LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
        byte MenuOn;
        boolean PowerSave;
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

    //RTC and SD constants
        #ifdef Datalogger
        RTC_DS3231 rtc;                             //The type of RTC used
        const byte chipSelect = 4;                  //CS pin for SD card reader
        #endif
    //Constants for TB6612 motor driver
        #ifdef FanControl
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
        void ManualFanSet();
        void HumidityCheck();
        void ManualSoilCal();
        void PumpWater();
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
        
        MenuOn = 0;

    //start the SD card
        // see if the card is present and can be initialized:
        #ifdef Datalogger
        if (!SD.begin(chipSelect)) {
            ErrorMessages(1);
        }
        #if !defined(LowMemoryMode)
        lcd.clear();
        lcd.print("Initiating...");
        lcd.setCursor(0, 1);
        lcd.print("SD card present");
        delay(1500);
        #endif
        #endif
    
    //start RTC and set time
        #ifdef Datalogger
        if (! rtc.begin()) {
            ErrorMessages(2);
        }
        
        #if !defined(LowMemoryMode)
        lcd.clear();
        lcd.print("Initiating...");
        lcd.setCursor(0, 1);
        lcd.print("RTC: ");
        lcd.print(rtc.lostPower() == 1 ? "Has stopped" : "OK");       //Shows if the RTC has stopped at any time (no battery)
        #endif
        delay(1500);

        //Uncomment the following line to set the time when the code is uploaded
            //If left commented, Time will be set at the upload time everytime the MCU restarts
        //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        #endif
    //Start DHT22 sensor and set calibration values for soil sensor
        dht.begin();

        //Analog values for soil sensor. The lower the value the more humidity. 
        //                  !Requires calibration!
        //if you already know the upper and lower values write them, as it makes calibration later not required
        Calvalues.HighCal1 = 220;
        Calvalues.LowCal1 = 550; 


    //Show the date and time set
        #ifdef Datalogger
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
        motor1.drive(10 * 2.55);    //Auto start the fan extractor at 10% speed
        #endif

        pinMode(A2, OUTPUT);        //AnalogPin 2 is used for water pump
         
        PowerSave = false;          //Powersave mode is set as off at setup
        ScreenOff = millis();
}

void loop() {
    if (millis() - OtherMillis >= 5000 && MenuOn == 0 && !PowerSave) {      //Temp+Hum sensor measurement real time 5000 = 5s
        //Obtain RH and Temp values
            float AirData[3];      
            ReadDHT22(AirData);

        //Print into screen
            lcd.clear();
            lcd.print(AirData[0], 1);
            lcd.print(" RH, ");
            lcd.print(AirData[1], 1);
            lcd.print(" C");

            lcd.setCursor(3, 1);
            lcd.print(AirData[2],2);
            lcd.print(" kPa VPD");
            OtherMillis = millis();
            delay(2500);

        //Fan control with VPD
            #ifdef FanControl
            //TODO: change into a separate function
            //TODO: add humidity functionality. Now only works according to temperature 
            if (AirData[1] >= 28 && AirData[1] < 30)  {         //20% - 28-30ºC
                motor1.drive(20 * 2.55);
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("fan 20%"); 
            }
            if (AirData[1] >= 30 && AirData[1] < 32) {          //30% - 30-32ºC
                motor1.drive(30 * 2.55);
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("fan 30%"); 
            }
            else if (AirData[1] >= 26 && AirData[1] < 28) {     //10% - 26-28ºC
                motor1.drive(10 * 2.55);
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("fan 10%"); 
            } 
            else if (AirData[1] < 26){                          //0% - <26ºC
                motor1.drive(0);
            }
            if (AirData[1] >= 32) {          //50% - >32ºC
                motor1.drive(50 * 2.55);
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("fan 50%"); 
            }
            #endif
    }

    if (millis() - SoilMesmillis >= TimeSoilMeasurement) {      //Soil sensor measurement 1800000 = 30 mins
        //Takes data from the function
            float SoilData[1];
            SoilMeasurement(SoilData); 

        //Logs data into SD card and prints it in the screen
            #ifdef Datalogger
            LogData(1, 1, SoilData);
            delay(2000);
            #if !defined(LowMemoryMode)
            PrintData(2, SoilData);  
            delay(2000);
            #endif 
            #endif
        
        //checks if humidity was lower than a threshold
            if (SoilData[0] <= SoilHumWatering) { 
                PumpWater();
                #ifdef Datalogger
                LogData(3, 1, SoilData);
                #endif
            }
        //restarts the counting for next soil humidity datalog
            SoilMesmillis = millis();
    }
    
    #ifdef Datalogger           //Right now, function only serves datalogging purposes
    if (millis() - AirMesmillis >= 900000) {       //Temp+Hum sensor measurement 900000 = 15 min
        //obtain values
            float AirData[3];      
            ReadDHT22(AirData);
        
        
        //print data + send to SD card logging
            LogData(2, 3, AirData);
            delay(2000);
            #if !defined(LowMemoryMode)
            PrintData(2, AirData);  
            delay(2000);
            #endif
        

        //Reset the timer for next 15 mins
            AirMesmillis = millis();
    }
    #endif

    if (analogRead(A6) <= 1000) {                   //used to detect button press and start the menu
        ButtonMillis = millis();
        PowerSave = false;
        lcd.displayOn();

        while (analogRead(A6) <= 1000){         //wait for the button to stop being pressed
            delay(1);
        }
        
        if (millis() - ButtonMillis >= 1000) {   //if 2s has passed then go to the menu
            MenuOn = 1;
            byte MainMenuOption;
            MainMenuOption = MainMenu();

            delay(200);
            if (MainMenuOption == 1) {
                ManualSoilCal();
            }
            #ifdef FanControl
            if (MainMenuOption == 2) {
                ManualFanSet();
            }
            #endif

            else if (MainMenuOption == 3) {
                HumidityCheck();
            }
        }

        MenuOn = 0;
        ButtonMillis = millis();                //added some time to prompt the air values
        ScreenOff = millis();                   //Restarts the counting or LCD backlight off mode
    }

    if (millis() - ScreenOff >= 60000 && !PowerSave) {      //30000 = 30s. Used to turn off the screen after not touching any button
        PowerSave = true;
        lcd.displayOff();
    } 
}

void SoilMeasurement(float* SoilData) {             //Measurements of Soil moisture sensor
    //Sensor measurement creating a total to later calculate average converted to %
        float sensorReadings[3];  // array for raw measurements
        float percentArray[3];    // array for raw converted to %
        float total = 0;

        for (int i = 0; i < 3; i++) {
            sensorReadings[i] = analogRead(SPin1);
            percentArray[i] = map(sensorReadings[i], Calvalues.HighCal1, Calvalues.LowCal1, 100, 0);
            total += percentArray[i];
            delay(1000);
        }
        float average = total / 3;


    
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

#ifdef Datalogger
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

    #ifdef Datalogger   
    if (ErNum == 1) {          //SD card not present/failed at start
        lcd.print("SD card failed,");
        lcd.setCursor(0, 1);
        lcd.print("or not present");
    }

    else if (ErNum == 2) {               //RTC not present/failed at start
        lcd.print(" RTC not found");
        lcd.setCursor(0, 1);
        lcd.print("Check and reset");    
    }

    else if (ErNum == 3) {          //Unable to open SD card file 
        lcd.print("cannot open .txt");
        lcd.setCursor(0, 1);
        lcd.print(" Check SD card");          
    }
    #endif

    while (1) {}
}

byte ButtonPress() {
	//Sets the values for analog thresholds and keypad 
        ButtonMillis = millis();
		int KeyThresholds[3] = {540, 690, 775};
		byte keypad[3] = {0,1,2};
		byte keypress;
        boolean LongPress;

	//Waits until any button is pressed
		while (analogRead(A6) > 1000) {		
			delay(1);
		}

    //checks if the button was pressed for short time (500ms)
        if (millis() - ButtonMillis >= 200 && millis() - ButtonMillis < 1000) {
            LongPress = false;
        }
        
        else if (millis() - ButtonMillis > 1000) {
            LongPress = true;
        }

	//saves analog value and with a loop finds closest threshold to set it to the determined key
		int val = analogRead(A6);	
		for (int i = 0; i < 3; i++) {					//loop 3 times increasing i
			if (abs(val - KeyThresholds[i]) < 30) {		//If analog is in threshold, select key
				keypress = keypad[i];
			}
		}

	//waits until button is no longer pressed, and sends it back to caller
		while (analogRead(A6) < 1000) {
			delay(1);
		}
		return keypress;
        return LongPress;
}

//TODO: still in construction...
byte MainMenu() {       //Shows the main menu, retuens a byte corresponding to the option selected
    const char* MainMenuOptions[] = {
        "Return",          // Option 1
        "Soil H% calib.",  // Option 2
        "Soil Humidity"    // Option 3
    };

	byte Options = 0;
	boolean ExitMenu = false;
    byte MenuOutput;    

	while (!ExitMenu)  { 
        lcd.clear();   
        lcd.setCursor(0, 0);
        lcd.write(ARROW_LEFT);
        lcd.print(MainMenuOptions[Options]);
        lcd.setCursor(0, 1);
        lcd.print(MainMenuOptions[Options + 1]);

        delay(400);
		byte keypress;
		keypress = ButtonPress();           //Returns the key that was pressed

        switch (keypress) {
            case 0:                     //Enter button
            	lcd.clear();
                ExitMenu = true;
                MenuOutput = Options;
            break;
            case 1:                     //Up key
            Options = Options - 1;
            break;
            case 2:                     //Down key
            Options = Options + 1;
            break;
        }
    }
	lcd.clear();
    return Options;
}

#ifdef SoilSensCalibr
void ManualSoilCal() {
	lcd.clear();
    lcd.setCursor(0, 0);
	lcd.print("Dry sensor then");
    lcd.setCursor(0, 1);
    lcd.print(" place in water");
    delay(3000);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press to start");

    while (analogRead(A6) >= 1000) {
        delay(1);
    }
   
    int Analog1 = analogRead(SPin1);            //first, it sets the sensor to whatever value is now 
    Calvalues.LowCal1 = Analog1;
    Calvalues.HighCal1 = Analog1;
    bool Stable = 0;

    lcd.clear();
    lcd.print("Press when calbr");
    lcd.setCursor(0, 1);
    lcd.print("is completed...");
    delay(2000);

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
        if (analogRead(A6) <= 1000) {
            Stable = 1;
        }
        delay(500);
    }
    //print new calibration values
    lcd.clear();
    lcd.print("  Calibr. saved");
    delay(2000);
        
    #ifndef LowMemoryMode
    lcd.clear();
    lcd.print("New U-L values:");
    lcd.setCursor(0, 1);
    lcd.print(", U: ");
    lcd.print(Calvalues.LowCal1);
    lcd.print(", L: ");
    lcd.print(Calvalues.HighCal1);
    delay(3000);
    #endif

}
#endif

void PumpWater() {
    lcd.clear();
    lcd.print(" Watering plant");
    delay(500);

    digitalWrite(A2, HIGH);
    delay(60000);                   //TODO: Replace with a millis function to allow fan control or something
    digitalWrite(A2, LOW);

    #if !defined(LowMemoryMode)
    lcd.clear();
    lcd.print(" Plant watered");
    delay(1000);
    #endif
}

void HumidityCheck() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Measuring soil...");

    float SoilData[1];
    SoilMeasurement(SoilData); 

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(SoilData[0]);
    lcd.print(" % humidity");
    delay(2000);

    if (SoilData[0] <= SoilHumWatering + 5) {                //TODO: this watering is not saved in the .txt file!
        lcd.setCursor(0, 1);
        lcd.print("Press to water");

        byte keypress;
		keypress = ButtonPress();
        delay(4000);

        if (keypress == 0) {            //If enter key is pressed
            PumpWater();
		}
    }
}

#if !defined(LowMemoryMode)
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


*/