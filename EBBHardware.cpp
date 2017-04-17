#include "EBBHardware.h"
#include "config.h"

#include <EEPROM.h>

#define EEPROM_PEN_UP_POS 0
#define EEPROM_PEN_DOWN_POS 2
#define EEPROM_PEN_UP_RATE 4
#define EEPROM_PEN_DOWN_RATE 6

EBBHardware::EBBHardware(Stream& stream)
    : EBBParser(stream)
    , rotMotor(X_STEP_PIN, X_DIR_PIN)
    , penMotor(Y_STEP_PIN, Y_DIR_PIN)
    , penState(false)
    , penUpPos(5) // can be overwritten from EBB-Command SC
    , penDownPos(20) // can be overwritten from EBB-Command SC
    , servoRateUp(0)
    , servoRateDown(0)
    , motorEnabled(false)
    , prgButtonState(false)
#ifdef PRG_BUTTON_PIN
    , prgButtonToggle(PRG_BUTTON_PIN)
#endif
#ifdef PEN_TOGGLE_BUTTON_PIN
    , penToggle(PEN_TOGGLE_BUTTON_PIN)
#endif
#ifdef MOTORS_BUTTON_PIN
    , motorsToggle(MOTORS_BUTTON_PIN)
#endif
{
}

void EBBHardware::init()
{
    EEPROM.get(EEPROM_PEN_UP_POS, penUpPos);
    EEPROM.get(EEPROM_PEN_DOWN_POS, penDownPos);
    EEPROM.get(EEPROM_PEN_UP_RATE, servoRateUp);
    EEPROM.get(EEPROM_PEN_DOWN_RATE, servoRateDown);

    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);
    pinMode(ENGRAVER_PIN, OUTPUT);

    enableMotor(0, false);
    enableMotor(1, false);
    penServo.attach(SERVO_PIN);
    penServo.write(penState ? penUpPos : penDownPos);
}

void EBBHardware::processEvents()
{
#ifdef PRG_BUTTON_PIN
    if (prgButtonToggle.wasPressed()) {
        prgButtonState = true;
    }
#endif

#ifdef PEN_TOGGLE_BUTTON_PIN
    if (penToggle.wasPressed()) {
        setPenState(!getPenState());
    }
#endif

#ifdef MOTORS_BUTTON_PIN
    if (motorsToggle.wasPressed()) {
        if (motorEnabled) {
            enableMotor(0, false);
            enableMotor(1, false);
        } else {
            enableMotor(0, true);
            enableMotor(1, true);
        }
    }
#endif
    parseStream();

    penMotor.update();
    rotMotor.update();
}

void EBBHardware::enableMotor(int axis, bool state)
{
    const uint8_t pin = (axis == 0) ? X_ENABLE_PIN : Y_ENABLE_PIN;

    digitalWrite(pin, state ? LOW : HIGH);

    motorEnabled = state;
}

void EBBHardware::stepperMove(int duration, int numPenSteps, int numRotSteps)
{
    moveToDestination();

    // Genuine EggBots are almost all using 16 microstepping.
    // A standard nema 17 stepper has 200 steps.
    // With 16 microstepping 200 * 16 = 3200 steps for a full revolution.
    // If your EggDuino does not have the same number of steps,
    // you need to adjust the inkscape plugin STEP_SCALE variable inside eggbot_conf.py.

    // set Coordinates and Speed
    rotMotor.setTarget(numRotSteps, duration);
    penMotor.setTarget(numPenSteps, duration);
}

void EBBHardware::moveToDestination()
{
    while (penMotor.remainingSteps() || rotMotor.remainingSteps()) {
        penMotor.update(); // Moving.... moving... moving....
        rotMotor.update();
    }
}

void EBBHardware::setPinOutput(char port, int pin, int value)
{
    // PO,B,3,0 = disable engraver
    // PO,B,3,1 = enable engraver
    if (port == 'B' && pin == 3) {
        digitalWrite(ENGRAVER_PIN, value);
    }
}

void EBBHardware::setEngraverState(bool state, int power)
{
    digitalWrite(ENGRAVER_PIN, state);
}

void EBBHardware::setPenState(bool up)
{
    moveToDestination();

    if (up) {
        penServo.write(penUpPos, servoRateUp);
    } else {
        penServo.write(penDownPos, servoRateDown);
    }
    penState = up;
}

bool EBBHardware::getPenState()
{
    return penState;
}

void EBBHardware::setPenUpPos(int percent)
{
    // transformation from EBB to PWM-Servo
    penUpPos = (int)((float)(percent - 6000) / (float)133.3);
    EEPROM.put(EEPROM_PEN_UP_POS, penUpPos);
}

void EBBHardware::setPenDownPos(int percent)
{
    // transformation from EBB to PWM-Servo
    penDownPos = (int)((float)(percent - 6000) / (float)133.3);
    EEPROM.put(EEPROM_PEN_DOWN_POS, penDownPos);
}

void EBBHardware::setServoRateUp(int percentPerSecond)
{
    servoRateUp = percentPerSecond;
    EEPROM.put(EEPROM_PEN_UP_RATE, servoRateUp);
}

void EBBHardware::setServoRateDown(int percentPerSecond)
{
    servoRateDown = percentPerSecond;
    EEPROM.put(EEPROM_PEN_DOWN_RATE, servoRateDown);
}

bool EBBHardware::getPrgButtonState()
{
    bool state = prgButtonState;
    prgButtonState = false;
    return state;
}
