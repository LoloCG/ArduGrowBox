//libraries
	#include <Arduino.h>
	#include <LiquidCrystal_I2C.h>    
//constants
	#define ARROW_LEFT  0x7E
	LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
	byte MenuOn;  
    boolean PowerSave;          //Turns the display off if true
    unsigned long ScreenOff = millis();
    unsigned long ButtonMillis = millis();
    #define red 4
    #define white 3
    #define blue 2

//functions
	byte MainMenu();
    byte ButtonPress();

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
    pinMode(red, INPUT);
    pinMode(white, INPUT);
    pinMode(blue, INPUT);
    delay(1000);

    lcd.clear();
}

void loop() {
    lcd.clear();  
  	if (digitalRead(red) == HIGH || digitalRead(white) == HIGH || digitalRead(blue) == HIGH) {                   //used to detect button press and start the menu
        //Start counting, and turn on the screen
        ButtonMillis = millis();
        PowerSave = false;
        lcd.displayOn();
        
        /* Used as debug purposes

            if (digitalRead(red)) {
                lcd.print("red");
            } else if (digitalRead(white)) {
                lcd.print("white");
            } else if (digitalRead(blue)) {
                lcd.print("blue");
            }
        */

        //wait for the button to stop being pressed
            while (digitalRead(red) == HIGH || digitalRead(white) == HIGH || digitalRead(blue) == HIGH){ 
                delay(1);
            }

        //if 1s has passed then go to the menu
            if (millis() - ButtonMillis >= 1000) {   
                MenuOn = 1;
                byte MenuOutput;
                MenuOutput = MainMenu();            //Calls the menu function waiting for it to retrieve MenuOutput as byte
            
            //does whatever with the output of the menu
            delay(200);
            if (MenuOutput == 1) {
                lcd.print("-> option 1");
            } else if (MenuOutput == 2) {
                lcd.print("-> option 2");
            } else if (MenuOutput == 3) {
                lcd.print("-> option 3");
            }}
            delay(2000);
        MenuOn = 0;
        ButtonMillis = millis();                //added some time to prompt the air values
        ScreenOff = millis();                   //Restarts the counting or LCD backlight off mode
    }

    if (millis() - ScreenOff >= 15000 && !PowerSave) {      //30000 = 30s. Used to turn off the screen after not touching any button
        PowerSave = true;
        lcd.displayOff();
    } 

}


byte ButtonPress() {
	//Longpress/shortpress has been disabled in this test
    //Sets the values for analog thresholds and keypad 
        ButtonMillis = millis();
		byte keypress;
        boolean LongPress;
    //wait for the button to be pressed
        while (digitalRead(red) == LOW && digitalRead(white) == LOW && digitalRead(blue) == LOW){ 
            delay(1);
        } 
    lcd.clear();
    lcd.print("button pressed");
    //Returns number based on key pressed
        //Original uses one imput with voltage divider keypad matrix
        //Here it used D2 to D4 as input.
        if (digitalRead(red) == HIGH){
            keypress = 0;
        } else if (digitalRead(white) == HIGH) {
            keypress = 1;
        } else if (digitalRead(blue) == HIGH) {
            keypress = 2;
        }

    //wait for the button to stop being pressed
        while (digitalRead(red) == HIGH && digitalRead(white) == HIGH && digitalRead(blue) == HIGH){ 
            delay(1);
        }
    
    return keypress;
    return LongPress;
}


byte MainMenu() {       //Shows the main menu, retuens a byte corresponding to the option selected
    const char* MainMenuOptions[] = {
        "Return",          // Option 0
        "Soil H% calib.",  // Option 1
        "Soil Humidity"    // Option 2
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
                if (Options < 2) {
                    Options = Options + 1;
                }
                else {
                    Options = MaxOptions - 1;
                }
            break;
            
            case 2:                     //Down key/blue
                if (Options > 0) {
                    Options--;
                }
                else {
                    Options = 0;
                }
            break;
        }
    }
	lcd.clear();
    return MenuOutput;
}