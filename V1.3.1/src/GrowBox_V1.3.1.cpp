/* Version: 1.3.1
    -This is intended for Arduino NANO 3.0 typically sold in aliexpres

    -Due to my incompetence as a programmer, this code causes memory constrains (specially SRAM) on Arduino NANO. 
        I have set up a #define (LowMemoryMode) that disables multiple functions and menus, 
        thus allowing enough free SRAM to run properly.
        Disabling LowMemoryMode is not garanteed to work in this version...
        They are kept for future versions and as tests
        
        For the future, I will optimize code and consider porting into STM32 (Bluepill/Blackpill) 
        and ESP32 processors (ESP32 S3), both of which have orders of magnitude greater memory size. 

    --------------------------------------------------------------------------------------------------------------------------*/
//Libraries
	#include <Arduino.h>
    #include <Wire.h> 
    #include <LiquidCrystal_I2C.h>
    #include <SPI.h>                    //required for SD's SPI communication
    #include "RTClib.h"                 //for RTC Adafruit's library
    #include <SD.h>                     //for SD card datalogger
    //#include <Adafruit_Sensor.h>      //for DHT22 nto required here, as it is included in DHT22 library already, but must have it in platformio.ini
    #include <DHT.h>                    //for DHT22 temperature/humidity sensor
    #include <SparkFun_TB6612.h>        //for the motor driver
    

//Constants
    //General constants
        //comment or uncomment the following defines to allow certain code to be included
            //#define DebugMode                 //When enabled, LCD will print lot more stuff to show what is happening at that time.
            #define LowMemoryMode               //When enabled, memory will me optimized by disabling functions and menus
            //#define FanControl                //Disabled when im not controlling fans

        //millis counting to perform a measure at determined timings 
            unsigned long AirMesmillis = millis();
            unsigned long SoilMesmillis = millis();
            unsigned long OtherMillis = millis();       //Used as general purpose
            unsigned long ScreenOff = millis();
            unsigned long ButtonMillis = millis();      //Used for button purposes...
        //Values for the keypad
            int KeyThresholds[3] = {525, 690, 775};     //Analog values detected by the pins of the keypad. Dependent on the resistors
            byte keypad[3] = {0,1,2};                   //this is the interpreted values after each key press
            byte MenuOn;                                //Variable that keeps other functions such as VPD display off. TODO: Not sure if really necessary

        boolean PowerSave;

    //Constants for the LCD
        #define ARROW_LEFT  0x7E
        LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
        
    //Constants for Sensors
        #define DHTPIN 5           // D pin for DHT22
        #define DHTTYPE DHT22      // DHT 22  (AM2302)
        DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor for normal 16mhz Arduino

        //-------------------------------FOR SOIL SENSOR-------------------------------//
        #define SPin1 A0           // A pin where Soil sensor is connected
        //the following defines top and bottom analog readings to specify the % scale of the soil sensor. Higher value = less water
        struct SoilCal {
            unsigned int HighCal1 : 10;
            unsigned int LowCal1 : 10;
        } Calvalues;

    //RTC and SD constants
        RTC_DS3231 rtc;                             //The type of RTC used
        const byte chipSelect = 4;                  //CS pin for SD card reader

    //Constants for TB6612 motor driver
        const int offsetA = 1;
        const int offsetB = 1;

        #define STBY 11                             //Not actually used here... TODO: check if there is any way to remove it 
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

    //start the SD card
        // see if the card is present and can be initialized:
        if (!SD.begin(chipSelect)) {
            ErrorMessages(1);
        }
        #if !defined(LowMemoryMode) || defined(DebugMode)
        lcd.setCursor(0, 1);
        lcd.print("SD card present");
        delay(1500);
        #endif
    //start RTC and set time
        if (! rtc.begin()) {
            ErrorMessages(2);
        }
        //#if !defined(LowMemoryMode) || defined(DebugMode)
        lcd.clear();
        lcd.print("Initiating...");
        lcd.setCursor(0, 1);
        lcd.print("RTC: ");
        lcd.print(rtc.lostPower() == 1 ? "Has stopped" : "OK");       //Shows if the RTC is running with its battery
        //#endif
        /*
        if (rtc.lostPower()) {                  //not sure if this really works... could cause trouble if RTC is turned off
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        */
        delay(1500);

        //Uncomment the following line to set the time when the code is uploaded
        //If left commented, Time will be set at the upload time everytime the MCU restarts
        //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    //Start DHT22 sensor
        dht.begin();

    //Set initial calibration values for soil meter.      
        //!!! if you already know the upper and lower values write it
        Calvalues.HighCal1 = 195;
        //Calvalues.HighCal2 = 345;         used for second sensor, not used in this version
        Calvalues.LowCal1 = 500;
        //Calvalues.LowCal2 = 750;
   
    //Set other parameters
        MenuOn = 0;
        pinMode(BIN2, OUTPUT);      //Used for pin BIN2 of the motor driver, as it requires the use of A1 as digital pin
        //motor2.drive(25);         //Auto start the fans at 10%
        pinMode(A2, OUTPUT);

        PowerSave = false;          //Powersave mode is set as off at setup
        ScreenOff = millis();
    
    //Show internal temp
        #if !defined(LowMemoryMode) || defined(DebugMode)
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Initiating...");
        lcd.setCursor(0, 1);
        lcd.print("RTC temp: ");
        lcd.print(rtc.getTemperature());
        delay(2000);
        #endif
    //Show the date and time set
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

            lcd.setCursor(1, 1);
            lcd.print(" ");
            lcd.print(AirData[2],2);
            lcd.print(" kPa VPD");

        OtherMillis = millis();
    }

    if (millis() - SoilMesmillis >= 1800000) {      //Soil sensor measurement 1800000 = 30 mins
        //Takes data from the function
            float SoilData[1];
            SoilMeasurement(SoilData); 
       
        /* DISABLED, //creates an int array with the data from soil sensor 1 and 2
            this was used for compilatign multiple sensors Data into same array to store them all together.
            Not removed from here for future reference 
            float SoilIntArray[4];
            for (int i = 0; i < 2; i++) {
            SoilIntArray[i] = Soil1Data[i];
            SoilIntArray[i + 2] = Soil2Data[i];
            }
        */

        //Logs data into SD card and prints it in the screen
            LogData(1, 1, SoilData);            //Changed size of array from 2 to 1, as StDev is not used in arduino nano due to memory constrains
            delay(2000);
            #if !defined(LowMemoryMode) || defined(DebugMode)
            PrintData(2, SoilData);  
            delay(2000);
            #endif 
        //checks if humidity was lower than a threshold
            if (SoilData[0] <= 65) {            //Humidity threshold for watering below this level
                #if !defined(LowMemoryMode) || defined(DebugMode)
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Humidity low");
                delay(1000);
                #endif
                PumpWater();
                LogData(3, 1, SoilData);

            }
        //restarts the counting for next soil humidity datalog
            SoilMesmillis = millis();
    }
     
    if (millis() - AirMesmillis >= 900000) {       //Temp+Hum sensor measurement 900000 = 15 min
        #if !defined(LowMemoryMode) || defined(DebugMode)
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("measuring Air...");
        delay(1000);
        #endif

        //obtain values from function
            float AirData[3];      
            ReadDHT22(AirData);

        //print data + send to SD card logging
            LogData(2, 3, AirData);
            delay(2000);
            #if !defined(LowMemoryMode) || defined(DebugMode)
            PrintData(2, AirData);  
            delay(2000);
            #endif

        //Use AirData to check if VPD is above or below the threshold...
            #if Testing
            if (AirData[2] <= 0.5 || AirData[2] >= 1.0) {} //TODO:
            #endif
        //Reset the timer for next 15 mins
            AirMesmillis = millis();
    }

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
            #if FanControl
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

    //(DISABLED) Calculate standard deviation  
        /*            
        float sqDevSum = 0;
        for (int i = 0; i < 3; i++) {
            sqDevSum += (average - percentArray[i]) * (average - percentArray[i]);
        }
        float stDev = sqrt(sqDevSum / (3 - 1));
        */
    
    // convert into an array
        SoilData[0] = average;
        //SoilData[1] = stDev;
}

void ReadDHT22(float* AirData) {                //Function to measure air temp. and humidity
    //Read data and store it to variables hum and temp
        float RH;   //Stores humidity value
        float temp;  //Stores temperature value
        RH = dht.readHumidity();
        temp = dht.readTemperature();
    
    //Calculate VPD
        byte offset = 1;    //Offset of leaf temperature difference from air temperature. 
                            //It is in negative! 2 = -2 ºC difference
        float VPD = (0.61078 * exp((temp - offset) / ((temp - offset) + 237.3) * 17.2694)) - ((0.61078 * exp(temp / (temp + 237.3) * 17.2694)) * RH / 100);
        
    // convert into an array
        AirData[0] = RH;
        AirData[1] = temp;
        AirData[2] = VPD;
        return;
}

void LogData(byte FileNum, byte Datasize, float* DataArray) {
    //Select folder based on FileNum
        char filename[20]; //TODO: not sure if there is a way to optimize
        if (FileNum == 1) {
            strcpy(filename, "SOILDATA.txt");
        } 
        else if (FileNum == 2) {
            strcpy(filename, "AIRDATA.txt");
        }
        //I think its best to save the waterings to the same file as SOIL DATA with a boolean value 
        else if (FileNum == 3) {
            strcpy(filename, "WATERING.txt");
        }
        /*
        else if (FileNum ==3) {
            strcpy(filename, "ERRORS.txt");
        }
        */
       
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

            //print that it was successful
            #ifndef DebugMode
                lcd.clear();
                lcd.print("Data logged.");
            #endif
        }
}

void ErrorMessages(byte ErNum) {
    PowerSave = false;
    lcd.displayOn();
    lcd.clear();

    if (ErNum == 1) {
        lcd.print("SD card failed,");
        lcd.setCursor(0, 1);
        lcd.print("or not present");
    }
    else if (ErNum == 2) {
        lcd.print(" RTC not found");
        lcd.setCursor(0, 1);
        lcd.print("Check and reset");    
    }
    else if (ErNum == 3) {
        lcd.print("cannot open .txt");
        lcd.setCursor(0, 1);
        lcd.print(" Check SD card");          
    }

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

byte MainMenu() {
    /*TODO: This is all badly optimized... This should be the first target for optimization
        -Change each menu option to a static character screen that gets called with each menu,
            this way it can reclycle the same options, only changing the arrow placement...
        -Change If/Else with Switch cases
        -Menu selection can be changed to an option that increases/decreases according to MainMenuSel, 
            instead of having to change it all the time. This could allow using a simple function in all menus, rather than separate ones
        
        -For scalability of the project, consider maybe using only one option per menu if option 1 is not possible.
            This way an option can be disabled if low memory or not used. 
    */

	byte MainMenuSel = 0;
	boolean ExitMenu = false;
    byte MainMenuOption;
	while (!ExitMenu)  {
		if (MainMenuSel == 0) {         //Menu 1
			lcd.clear();
            lcd.setCursor(0, 0);
			lcd.write(ARROW_LEFT);
			lcd.print("Return");
			lcd.setCursor(0, 1);
			lcd.print(" Soil H% calib. ");
			delay(500);
			byte keypress;
			keypress = ButtonPress();
			
			if (keypress == 0) {            //If enter key is pressed
				ExitMenu = true;
				lcd.clear();
				lcd.print("Returning");
				delay(1000);
                MainMenuOption = 0;
                return MainMenuOption;
			}
			else if (keypress == 1) {        //If up key is pressed 
				MainMenuSel = 3;
			}
			else if (keypress == 2) {       //If down key is pressed 
				MainMenuSel = 1;
			}	
		}
        else if (MainMenuSel == 1) {    //Menu 2, Soil humidity calibration
			lcd.clear();
            lcd.setCursor(0, 0);
			lcd.print(" Return");            //Soil Humidity
			lcd.setCursor(0, 1);
			lcd.write(ARROW_LEFT);
            lcd.print("Soil H% calib.");
            delay(300);
			byte keypress;
			keypress = ButtonPress();
			
			if (keypress == 0) {            //If enter key is pressed
                MainMenuOption = 1;
                ExitMenu = true;
                return MainMenuOption;
			}
			else if (keypress == 1) {        //If up key is pressed 
				MainMenuSel = 0;
			}
			else if (keypress == 2) {       //If down key is pressed 
				MainMenuSel = 3;
			}	
		}
        #if defined(FanControl) || !defined(LowMemoryMode)
		else if (MainMenuSel == 2) {    //Menu 3, Manual Fan control
			lcd.clear();
            lcd.setCursor(0, 0);
			lcd.print(" Return");
			lcd.setCursor(0, 1);
			lcd.write(ARROW_LEFT);
			lcd.print("Manual Fan ctrl");
            delay(300);
			byte keypress;
			keypress = ButtonPress();
			
			if (keypress == 0) {            //If enter key is pressed
                ExitMenu = true;
                MainMenuOption = 2;
                return MainMenuOption;
			}
			else if (keypress == 1) {        //If up key is pressed 
				MainMenuSel = 1;
			}
			else if (keypress == 2) {       //If down key is pressed 
				MainMenuSel = 3;
			}	
	
		}
        #endif

		else if (MainMenuSel == 3) {    //Menu 4, humidity measure
			lcd.clear();
			lcd.write(ARROW_LEFT);
            lcd.print("Soil Humidity");
            lcd.setCursor(0,1);
            lcd.print("MAIN to continue");
            delay(300);

			byte keypress;
			keypress = ButtonPress();
			
			if (keypress == 0) {            //If enter key is pressed
                MainMenuOption = 3;
                ExitMenu = true;
                return MainMenuOption;
			}
			else if (keypress == 1) {        //If up key is pressed 
				MainMenuSel = 1;
			}
			else if (keypress == 2) {       //If down key is pressed 
				MainMenuSel = 0;
			}	
		}
	}
	lcd.clear();
    return MainMenuOption;
}

void ManualSoilCal() {
    //Used to track if the measurement is stable (?)
	lcd.clear();
    lcd.setCursor(0, 0);
	lcd.print("Dry + Wet sensor");
    lcd.setCursor(0, 1);
    lcd.print(" Press to start");
    delay(500);

    while (analogRead(A6) >= 1000) {            //TODO: Add a time count so if a time passes, it goes back to loop
        delay(1);
    }
    /*
        After pressing the button, create while loop until a time has passed with the stable condition
        stable = true when the values are not surpassed for a certain amount of time. 
        So it reads the lowest/highest value and sets it as the limit
    */
   
    int Analog1 = analogRead(SPin1);            //first, it sets the sensor to whatever value is coming now 
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

void PumpWater() {
    //#if !defined(LowMemoryMode) || defined(DebugMode)
    lcd.clear();
    lcd.print(" Watering plant");
    delay(500);
    //#endif 

    digitalWrite(A2, HIGH);
    delay(60000);
    digitalWrite(A2, LOW);

    #if !defined(LowMemoryMode) || defined(DebugMode)
    lcd.clear();
    lcd.print(" Plant watered");
    delay(1000);
    #endif
}

#if FanControl
void ManualFanSet() {

	byte MainMenuSel = 0;
	boolean ExitMenu = false;
    byte FanSelect = 0;

    while (!ExitMenu)  {
		if (MainMenuSel == 0) {         //Menu 1
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Select Fan:");
            lcd.setCursor(0, 1);
            lcd.write(ARROW_LEFT);
            lcd.print("Fan1     ");
            lcd.print("Fan2");
        	delay(200);

            byte keypress;
			keypress = ButtonPress();
			
			if (keypress == 0) {
                FanSelect = 1;
                ExitMenu = true;
			}
			else {
				MainMenuSel = 1;
			}
        }
        else if (MainMenuSel == 1) {    //Menu 2
			lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Select Fan:");
            lcd.setCursor(0, 1);
            lcd.print(" Fan1    ");
            lcd.write(ARROW_LEFT);
            lcd.print("Fan2");
            delay(200);
            
			byte keypress;
			keypress = ButtonPress();
			
			if (keypress == 0) {
                FanSelect = 2;
                ExitMenu = true;
			}
			else {
				MainMenuSel = 0;
			}
        }   
    }

    boolean ExitFanSpeed = false;
    byte Speed = 10;            //Sets initial speed to 10%

    while (!ExitFanSpeed)  {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(" Press to set");
        lcd.setCursor(0, 0);
        lcd.print("Fan speed: ");
        lcd.print(Speed);
        lcd.print(" %");
        if (FanSelect == 1) {
            //motor1.drive(Speed * 2.55); 
        }
        else if (FanSelect == 2) {
            //motor2.drive(Speed * 2.55);
        }

        byte keypress;
		keypress = ButtonPress();

		if (keypress == 1) {
            Speed = Speed + 10;
        }
        else if (keypress == 2) {
            Speed = Speed - 10;
        }
        else if (keypress == 0) {
            ExitFanSpeed = true;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Fan speed at: ");
            lcd.setCursor(0, 1);
            lcd.print("   ");
            lcd.print(Speed);
            lcd.print(" %");
            delay(1500);
        }
    }

}
#endif

void HumidityCheck() {
    #ifndef LowMemoryMode
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Measuring soil...");
    #endif

    float SoilData[2];
    SoilMeasurement(SoilData);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(SoilData[0]);
    lcd.print(" % humidity");
    delay(1000);

    if (SoilData[0] <= 75) {                //TODO: this watering is not saved in the .txt file!
        lcd.setCursor(0, 1);
        lcd.scrollDisplayRight();
        lcd.print("press to water now");
        delay(500);

        byte keypress;
		keypress = ButtonPress();

        if (keypress == 0) {            //If enter key is pressed
            PumpWater();
            //DEBUG!
            //lcd.clear();
            //lcd.print("watering now");
		}
        else {
            lcd.clear();
            lcd.print("Returning...");
            delay(1000);
        }
    }
    lcd.noAutoscroll();
}
#if !defined(LowMemoryMode) || defined(DebugMode)
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
