// Declaration of Libraries
// LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// Motor
#include "control.h"
#include <AccelStepper.h>

// Encoder
#include <ClickEncoder.h>
// Timer 1 for encoder
#include <TimerOne.h>
// Save Function
#include <EEPROMex.h>
#include <AccelStepper.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
bool refreshScreen = false;

byte enterChar[] = {
    B10000,
    B10000,
    B10100,
    B10110,
    B11111,
    B00110,
    B00100,
    B00000};

byte fastChar[] = {
    B00110,
    B01110,
    B00110,
    B00110,
    B01111,
    B00000,
    B00100,
    B01110};
byte slowChar[] = {
    B00011,
    B00111,
    B00011,
    B11011,
    B11011,
    B00000,
    B00100,
    B01110};

unsigned long previousMillis = 0; // will store last time LED was updated
// constants won't change:
const long interval = 1000; // interval at which to blink (milliseconds)

unsigned long previousMillis2 = 0; // will store last time LED was updated
// constants won't change:
const long interval2 = 200; // interval at which to blink (milliseconds)
unsigned long currentMillis2 = 0;

// Declaration of LCD Variables
const int numOfMainScreens = 3;
const int numOfSettingScreens = 6;
const int numOfTestMenu = 12;
int currentScreen = 0;

int currentSettingScreen = 0;
int currentTestMenuScreen = 0;

String screens[numOfMainScreens][2] = {
    {"SETTINGS", "ENTER TO EDIT"},
    {"RUN AUTO", "ENTER TO RUN AUTO"},
    {"TEST MACHINE", "ENTER TO TEST"}};

String settings[numOfSettingScreens][2] = {
    {"THRESHER", "MIN"},
    {"FEEDER", "SEC"},
    {"CONVEYOR", "SEC"},
    {"DRYING", "MIN"},
    {"GRINDING", "MIN"},
    {"SAVE SETTINGS", "ENTER TO SAVE"}};

String TestMenuScreen[numOfTestMenu] = {
    "THRESHER",
    "FEEDER",
    "LINEAR 1",
    "LINEAR 2",
    "CONVEYOR",
    "LINER 3",
    "HEATER",
    "FAN",
    "ELEVATOR",
    "GRINDER",
    "DISPENSING",
    "Back to Main Menu"};

double parametersTimer[numOfSettingScreens] = {1, 1, 1, 1, 1, 1};
double parametersTimerMaxValue[numOfSettingScreens] = {120, 120, 600, 120, 120, 120};

int threshTimeAdd = 10;
int feedTimeAdd = 20;
int conveyorTimeAdd = 30;
int dryingTimeAdd = 40;
int grindTimeAdd = 50;

void saveSettings()
{
    EEPROM.updateDouble(threshTimeAdd, parametersTimer[0]);
    EEPROM.updateDouble(feedTimeAdd, parametersTimer[1]);
    EEPROM.updateDouble(conveyorTimeAdd, parametersTimer[2]);
    EEPROM.updateDouble(dryingTimeAdd, parametersTimer[3]);
    EEPROM.updateDouble(grindTimeAdd, parametersTimer[4]);
}

void loadSettings()
{
    parametersTimer[0] = EEPROM.readDouble(grindTimeAdd);
    parametersTimer[1] = EEPROM.readDouble(feedTimeAdd);
    parametersTimer[2] = EEPROM.readDouble(conveyorTimeAdd);
    parametersTimer[3] = EEPROM.readDouble(dryingTimeAdd);
    parametersTimer[4] = EEPROM.readDouble(grindTimeAdd);
}

char *secondsToHHMMSS(int total_seconds)
{
    int hours, minutes, seconds;

    hours = total_seconds / 3600;         // Divide by number of seconds in an hour
    total_seconds = total_seconds % 3600; // Get the remaining seconds
    minutes = total_seconds / 60;         // Divide by number of seconds in a minute
    seconds = total_seconds % 60;         // Get the remaining seconds

    // Format the output string
    static char hhmmss_str[7]; // 6 characters for HHMMSS + 1 for null terminator
    sprintf(hhmmss_str, "%02d%02d%02d", hours, minutes, seconds);
    return hhmmss_str;
}

bool settingsFlag = false;
bool settingEditTimerFlag = false;
bool runAutoFlag = false;
bool testMenuFlag = false;

// Declaration of Variables
// Rotary Encoder Variables
boolean up = false;
boolean down = false;
boolean middle = false;
ClickEncoder *encoder;
int16_t last, value;
// Fast Scroll
bool fastScroll = false;

// Control Declaration
Control Thresher(41, 38, 100);
Control Feeder(45, 100, 100);
Control ConveyorTimer(101, 101, 101);
Control Elevator(43, 36, 100);
Control Grinder(44, 100, 100);
Control Rotary(42, 35, 100);
Control Conveyor(40, 37, 100);
Control Heater(46, 100, 100);
Control Fan(8, 100, 100);

Control Linear1(32, 100, 100);
Control Linear2(33, 100, 100);
Control Linear3(34, 100, 100);

void setTimers()
{
    Thresher.setTimer(secondsToHHMMSS(parametersTimer[0] * 60));
    Feeder.setTimer(secondsToHHMMSS(parametersTimer[1]));
    ConveyorTimer.setTimer(secondsToHHMMSS(parametersTimer[2]));
    Heater.setTimer(secondsToHHMMSS(parametersTimer[3] * 60));
    Grinder.setTimer(secondsToHHMMSS(parametersTimer[4] * 60));
}

// Sensor Variables
const int volumetric_sen = 5;
const int jar_sen = 6;

int dispenseState = 0;
bool initialMoveConveyor, initialMoveRotary = false;

bool testRunDispensing = false;

void resetDispensing()
{
    initialMoveConveyor = false;
    initialMoveRotary = false;
    testRunDispensing = false;
    dispenseState = 0;
}
long currentPos = 0;
long lastPos = 0;
int speedStep = 1000;
// AccelStepper stepConveyor(AccelStepper::FULL2WIRE, 22,23);
AccelStepper stepConveyor(AccelStepper::FULL2WIRE, 52, 53);

void setStepper()
{
    stepConveyor.setEnablePin(51);
    stepConveyor.setPinsInverted(false, false, false);
    stepConveyor.setMaxSpeed(speedStep);
    stepConveyor.setSpeed(speedStep);
    stepConveyor.setAcceleration(speedStep * 10);
    stepConveyor.enableOutputs();
    lastPos = stepConveyor.currentPosition();
}
int RunAutoFlag = 0;
void RunAuto()
{
    switch (RunAutoFlag)
    {
    case 1:
        RunThresher();
        break;
    case 2:
        RunFeeder();
        break;
    case 3:
        RunConveyor();
        break;
    case 4:
        RunHeater();
        break;
    case 5:
        RunGrinder();
        break;
    case 6:
        RunDispense();
        break;
    default:
        resetDispensing();
        stopAll();
        RunAutoFlag = 0;
        break;
    }
}

void RunThresher()
{
    Thresher.run();
    if (Thresher.isTimerCompleted() == true)
    {
        RunAutoFlag = 2;
        Feeder.start();
    }
}

void RunFeeder()
{
    Feeder.run();

    if (Feeder.isTimerCompleted() == true)
    {
        RunAutoFlag = 3;
        ConveyorTimer.start();
    }
}

void RunConveyor()
{
    ConveyorTimer.run();
    Linear1.relayOn();
    Linear2.relayOn();
    if (ConveyorTimer.isTimerCompleted() == true)
    {
        RunAutoFlag = 4;
        Linear1.relayOff();
        Linear2.relayOff();
        Heater.start();
        Fan.relayOn();
    }
    else
    {
        if (stepConveyor.distanceToGo() == 0)
        {
            stepConveyor.setCurrentPosition(0);
            stepConveyor.move(1000);
        }
    }
}
bool testConveyorFlag = false;
void testConveyor()
{
    if (stepConveyor.distanceToGo() == 0)
    {
        stepConveyor.setCurrentPosition(0);
        stepConveyor.move(1000);
    }
    stepConveyor.run();
}

void RunHeater()
{
    Heater.run();

    if (Heater.isTimerCompleted() == true)
    {
        RunAutoFlag = 5;
        Elevator.relayOn();
        Linear3.relayOn();
        Grinder.start();
        Fan.relayOff();
        stepConveyor.setCurrentPosition(0);
        stepConveyor.move(1000);
    }
}

void RunGrinder()
{
    Grinder.run();
    if (Grinder.isTimerCompleted() == true)
    {

        Elevator.relayOff();
        Linear3.relayOff();
        dispenseState = 1;
        initialMoveConveyor = true;
        RunAutoFlag = 6;
    }
    if (stepConveyor.distanceToGo() == 0)
    {
        stepConveyor.setCurrentPosition(0);
        stepConveyor.move(1000);
    }
}

void setSensors()
{
    pinMode(jar_sen, INPUT_PULLUP);
    pinMode(volumetric_sen, INPUT_PULLUP);
}

bool readSensorPNP(int pin)
{
    int result = digitalRead(pin);
    if (result == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool readSensorNPN(int pin)
{
    int result = digitalRead(pin);
    if (result == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}



void RunDispense()
{
    switch (dispenseState)
    {
    case 1:
        RunConveying();
        break;
    case 2:
        RunRotary();
        break;
    default:
        break;
    }
}

void RunConveying()
{
    if (initialMoveConveyor == true)
    {
        if (readSensorNPN(jar_sen) == 1)
        {
            Conveyor.relayOn();
            delay(300);
        }
        else
        {
            initialMoveConveyor = false;
        }
    }
    else
    {
        if (readSensorNPN(jar_sen) == 1)
        {
            dispenseState = 2;
            initialMoveRotary = true;
            Conveyor.relayOff();
            delay(300);
        }
        else
        {
            Conveyor.relayOn();
            
        }
    }
}

void RunRotary()
{
   if (readSensorNPN(jar_sen) == 1)
        {
        if (initialMoveRotary == true)
        {
            if (readSensorPNP(volumetric_sen) == 1)
            {
                Rotary.relayOn();
            }
            else
            {
                initialMoveRotary = false;
            delay(300);
            }
        }
        else
        {
            if (readSensorPNP(volumetric_sen) == 1)
            {
                dispenseState = 1;
                initialMoveConveyor = true;
                Rotary.relayOff();
            delay(2000);
            }
            else
            {
                Rotary.relayOn();
            }
        }
    }else{
        dispenseState=1;
        Rotary.relayOff();
    }
}

void stopAll()
{
    Thresher.relayOff();
    Thresher.stop();

    Feeder.relayOff();
    Feeder.stop();

    ConveyorTimer.relayOff();
    ConveyorTimer.stop();

    Elevator.relayOff();
    Elevator.stop();

    Grinder.relayOff();
    Grinder.stop();

    Rotary.relayOff();
    Rotary.stop();

    Conveyor.relayOff();
    Conveyor.stop();

    Heater.relayOff();
    Heater.stop();

    Fan.relayOff();
    Fan.stop();

    Linear1.relayOff();
    Linear1.stop();

    Linear2.relayOff();
    Linear2.stop();

    Linear3.relayOff();
    Linear3.stop();
}

// Functions for Rotary Encoder
void timerIsr()
{
    encoder->service();
}

void readRotaryEncoder()
{
    value += encoder->getValue();

    if (value / 2 > last)
    {
        last = value / 2;
        down = true;
        delay(100);
    }
    else if (value / 2 < last)
    {
        last = value / 2;
        up = true;
        delay(100);
    }
}

void readButtonEncoder()
{
    ClickEncoder::Button b = encoder->getButton();
    if (b != ClickEncoder::Open)
    { // Open Bracket for Click
        switch (b)
        { // Open Bracket for Double Click
        case ClickEncoder::Clicked:
            middle = true;
            break;

        case ClickEncoder::DoubleClicked:
            refreshScreen = 1;
            if (settingsFlag)
            {
                if (fastScroll == false)
                {
                    fastScroll = true;
                }
                else
                {
                    fastScroll = false;
                }
            }
            break;
        }
    }
}

void inputCommands()
{
    // LCD Change Function and Values
    //  To Right Rotary
    if (up == 1)
    {
        up = false;
        refreshScreen = true;
        if (settingsFlag == true)
        {
            if (settingEditTimerFlag == true)
            {
                if (parametersTimer[currentSettingScreen] >= parametersTimerMaxValue[currentSettingScreen] - 1)
                {
                    parametersTimer[currentSettingScreen] = parametersTimerMaxValue[currentSettingScreen];
                }
                else
                {
                    if (fastScroll == true)
                    {
                        parametersTimer[currentSettingScreen] += 1;
                    }
                    else
                    {
                        parametersTimer[currentSettingScreen] += 0.1;
                    }
                }
            }
            else
            {
                if (currentSettingScreen == numOfSettingScreens - 1)
                {
                    currentSettingScreen = 0;
                }
                else
                {
                    currentSettingScreen++;
                }
            }
        }
        else if (testMenuFlag == true)
        {
            if (currentTestMenuScreen == numOfTestMenu - 1)
            {
                currentTestMenuScreen = 0;
            }
            else
            {
                currentTestMenuScreen++;
            }
        }
        else
        {
            if (currentScreen == numOfMainScreens - 1)
            {
                currentScreen = 0;
            }
            else
            {
                currentScreen++;
            }
        }
    }

    // To Left Rotary
    if (down == 1)
    {
        down = false;
        refreshScreen = true;
        if (settingsFlag == true)
        {
            if (settingEditTimerFlag == true)
            {
                if (parametersTimer[currentSettingScreen] <= 0)
                {
                    parametersTimer[currentSettingScreen] = 0;
                }
                else
                {
                    if (fastScroll == true)
                    {
                        parametersTimer[currentSettingScreen] -= 1;
                    }
                    else
                    {
                        parametersTimer[currentSettingScreen] -= 0.1;
                    }
                }
            }
            else
            {
                if (currentSettingScreen <= 0)
                {
                    currentSettingScreen = numOfSettingScreens - 1;
                }
                else
                {
                    currentSettingScreen--;
                }
            }
        }
        else if (testMenuFlag == true)
        {
            if (currentTestMenuScreen <= 0)
            {
                currentTestMenuScreen = numOfTestMenu - 1;
            }
            else
            {
                currentTestMenuScreen--;
            }
        }
        else
        {
            if (currentScreen == 0)
            {
                currentScreen = numOfMainScreens - 1;
            }
            else
            {
                currentScreen--;
            }
        }
    }

    // Rotary Button Press
    if (middle == 1)
    {
        middle = false;
        refreshScreen = 1;
        if (currentScreen == 0 && settingsFlag == true)
        {
            if (currentSettingScreen == numOfSettingScreens - 1)
            {
                settingsFlag = false;
                saveSettings();
                loadSettings();
                setTimers();
                currentSettingScreen = 0;
            }
            else
            {
                if (settingEditTimerFlag == true)
                {
                    settingEditTimerFlag = false;
                }
                else
                {
                    settingEditTimerFlag = true;
                }
            }
        }
        else if (runAutoFlag == true)
        {
            runAutoFlag = false;
            stopAll();
            RunAutoFlag = 0;
        }
        else if (testMenuFlag == true)
        {
            if (currentTestMenuScreen == numOfTestMenu - 1)
            {
                currentTestMenuScreen = 0;
                testMenuFlag = false;
            }
            else if (currentTestMenuScreen == 0)
            {
                if (Thresher.getMotorState() == false)
                {
                    Thresher.relayOn();
                }
                else
                {
                    Thresher.relayOff();
                }
            }
            else if (currentTestMenuScreen == 1)
            {
                if (Feeder.getMotorState() == false)
                {
                    Feeder.relayOn();
                }
                else
                {
                    Feeder.relayOff();
                }
            }
            else if (currentTestMenuScreen == 2)
            {
                if (Linear1.getMotorState() == false)
                {
                    Linear1.relayOn();
                }
                else
                {
                    Linear1.relayOff();
                }
            }
            else if (currentTestMenuScreen == 3)
            {
                if (Linear2.getMotorState() == false)
                {
                    Linear2.relayOn();
                }
                else
                {
                    Linear2.relayOff();
                }
            }
            else if (currentTestMenuScreen == 4)
            {
                if (testConveyorFlag == false)
                {
                    testConveyorFlag = true;
                    stepConveyor.setCurrentPosition(0);
                    stepConveyor.move(1000);
                }
                else
                {
                    testConveyorFlag = false;
                }
            }
            else if (currentTestMenuScreen == 5)
            {
                if (Linear3.getMotorState() == false)
                {
                    Linear3.relayOn();
                }
                else
                {
                    Linear3.relayOff();
                }
            }
            else if (currentTestMenuScreen == 6)
            {
                if (Heater.getMotorState() == false)
                {
                    Heater.relayOn();
                }
                else
                {
                    Heater.relayOff();
                }
            }
            else if (currentTestMenuScreen == 7)
            {
                if (Fan.getMotorState() == false)
                {
                    Fan.relayOn();
                }
                else
                {
                    Fan.relayOff();
                }
            }
            else if (currentTestMenuScreen == 8)
            {
                if (Elevator.getMotorState() == false)
                {
                    Elevator.relayOn();
                }
                else
                {
                    Elevator.relayOff();
                }
            }
            else if (currentTestMenuScreen == 9)
            {
                if (Grinder.getMotorState() == false)
                {
                    Grinder.relayOn();
                }
                else
                {
                    Grinder.relayOff();
                }
            }
            else if (currentTestMenuScreen == 10)
            {
                if (testRunDispensing == true)
                {
                    testRunDispensing = false;
                    resetDispensing();
                }
                else
                {
                    testRunDispensing = true;
                    dispenseState = 1;
                    initialMoveConveyor = true;
                }
            }
        }
        else
        {
            if (currentScreen == 0)
            {
                settingsFlag = true;
            }
            else if (currentScreen == 1)
            {
                runAutoFlag = true;
                // Insert Commands for Run Auto
                RunAutoFlag = 1;
                Thresher.start();
                refreshScreen = 1;
            }
            else if (currentScreen == 2)
            {
                testMenuFlag = true;
            }
        }
    }
}

void PrintRunAuto(String job, char *time)
{
    lcd.clear();
    lcd.print("Running Auto");
    lcd.setCursor(0, 2);
    lcd.print("Status: " + job);
    lcd.setCursor(0, 3);
    lcd.print("Timer: ");
    lcd.setCursor(7, 3);
    lcd.print(time);
}

void printScreen()
{

    if (settingsFlag == true)
    {
        lcd.clear();
        lcd.print(settings[currentSettingScreen][0]);
        lcd.setCursor(0, 1);
        if (currentSettingScreen == numOfSettingScreens - 1)
        {
            lcd.setCursor(0, 3);
            lcd.write(0);
            lcd.setCursor(2, 3);
            lcd.print("Click to Save All");
        }
        else
        {
            lcd.setCursor(0, 1);
            lcd.print(parametersTimer[currentSettingScreen]);
            lcd.print(" ");
            lcd.print(settings[currentSettingScreen][1]);
            lcd.setCursor(0, 3);
            lcd.write(0);
            lcd.setCursor(2, 3);
            if (settingEditTimerFlag == false)
            {
                lcd.print("ENTER TO EDIT");
            }
            else
            {
                lcd.print("ENTER TO SAVE");
            }
            lcd.setCursor(19, 3);
            if (fastScroll == true)
            {
                lcd.write(1);
            }
            else
            {
                lcd.write(2);
            }
        }
    }
    else if (runAutoFlag == true)
    {
        switch (RunAutoFlag)
        {
        case 1:
            PrintRunAuto("Threshing", Thresher.getTimeRemaining());
            break;
        case 2:
            PrintRunAuto("Feeding", Feeder.getTimeRemaining());
            break;
        case 3:
            PrintRunAuto("Conveying", ConveyorTimer.getTimeRemaining());
            break;
        case 4:
            PrintRunAuto("Drying", Heater.getTimeRemaining());

            break;
        case 5:
            PrintRunAuto("Grinding", Grinder.getTimeRemaining());
            break;
        case 6:
            PrintRunAuto("Dispensing", "N/A");
            break;

        default:
            break;
        }
    }
    else if (testMenuFlag == true)
    {
        lcd.clear();
        lcd.print(TestMenuScreen[currentTestMenuScreen]);

        if (currentTestMenuScreen == numOfTestMenu - 1)
        {
            lcd.setCursor(0, 3);
            lcd.print("Click to Exit Test");
        }
        else
        {
            lcd.setCursor(0, 3);
            lcd.print("Click to Run Test");
        }
    }
    else
    {
        lcd.clear();
        lcd.print(screens[currentScreen][0]);
        lcd.setCursor(0, 3);
        lcd.write(0);
        lcd.setCursor(2, 3);
        lcd.print(screens[currentScreen][1]);
        refreshScreen = false;
    }
}

void setupJumper()
{
}

void setup()
{
     encoder = new ClickEncoder(3, 2, 4); // Actual
   // encoder = new ClickEncoder(3, 4, 2); // TestBench
    encoder->setAccelerationEnabled(false);
    Timer1.initialize(1000);
    Timer1.attachInterrupt(timerIsr);
    last = encoder->getValue();

    // LCD Setup
    lcd.init();
    lcd.createChar(0, enterChar);
    lcd.createChar(1, fastChar);
    lcd.createChar(2, slowChar);
    lcd.clear();
    lcd.backlight();
    refreshScreen = true;
    Serial.begin(9600);

    // saveSettings(); // Disable upon Initialiaze
    loadSettings();
    setTimers();
    setStepper();
    setSensors();
}

void loop()
{
   Serial.print("Sensor Jar : "); Serial.println(readSensorPNP(jar_sen));
   Serial.print("Sensor Volumetric : "); Serial.println(readSensorNPN(volumetric_sen));
    readRotaryEncoder();
    readButtonEncoder();
    inputCommands();
    stepConveyor.run();

    if (refreshScreen == true)
    {
        printScreen();
        refreshScreen = false;
    }

    if (runAutoFlag == true)
    {
        RunAuto();
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval)
        {
            // save the last time you blinked the LED
            previousMillis = currentMillis;
            refreshScreen = true;
        }
    }

    if (testMenuFlag == true)
    {
        if (testConveyorFlag == true)
        {
            testConveyor();
        }
        if (testRunDispensing == true)
        {
            RunDispense();
        }
    }
}