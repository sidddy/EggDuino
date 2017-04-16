#include "EBBHardware.h"
#include "config.h"

#include <EEPROM.h>

// devide EBB-Coordinates by this factor to get EGGduino-Steps
#define rotStepCorrection (16 / X_MICROSTEPPING)
#define penStepCorrection (16 / Y_MICROSTEPPING)

#define EEPROM_PEN_UP_POS 0
#define EEPROM_PEN_DOWN_POS 2
#define EEPROM_PEN_UP_RATE 4
#define EEPROM_PEN_DOWN_RATE 6

EBBHardware::EBBHardware(Stream& stream)
    : EBBParser(stream)
    , rotMotor(1, X_STEP_PIN, X_DIR_PIN)
    , penMotor(1, Y_STEP_PIN, Y_DIR_PIN)
    , penState(1)
    , penUpPos(5) // can be overwritten from EBB-Command SC
    , penDownPos(20) // can be overwritten from EBB-Command SC
    , servoRateUp(0)
    , servoRateDown(0)
    , rotStepError(0)
    , penStepError(0)
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

    rotMotor.setMaxSpeed(2000.0);
    rotMotor.setAcceleration(10000.0);
    penMotor.setMaxSpeed(2000.0);
    penMotor.setAcceleration(10000.0);
    enableMotor(0, 0);
    enableMotor(1, 0);
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
    EBBParser::processEvents();
}

void EBBHardware::enableMotor(int axis, int value)
{
    const uint8_t pin = (axis == 0) ? X_ENABLE_PIN : Y_ENABLE_PIN;
    switch (value) {
    case 0:
        digitalWrite(pin, HIGH);
        break;
    case 1:
    case 2:
    case 3:
    case 4:
        digitalWrite(pin, LOW);
        break;
    }
    motorEnabled = value;
}

void EBBHardware::stepperMove(int duration, int axis1, int axis2)
{
    // if coordinatessystems are identical
    if ((1 == rotStepCorrection) && (1 == penStepCorrection)) {
        // set Coordinates and Speed
        rotMotor.move(axis2);
        rotMotor.setSpeed(abs((float)axis2 * (float)1000 / (float)duration));
        penMotor.move(axis1);
        penMotor.setSpeed(abs((float)axis1 * (float)1000 / (float)duration));
    } else {
        // incoming EBB-Steps will be multiplied by 16, then Integer-maths is
        // done, result will be divided by 16
        // This make thinks here really complicated, but floating point-math
        // kills performance and memory, believe me... I tried...
        long rotSteps = ((long)axis2 * 16 / rotStepCorrection) + (long)rotStepError;
        // correct incoming EBB-Steps to our
        // microstep-Setting and multiply  by 16 to
        // avoid floatingpoint...
        long penSteps = ((long)axis1 * 16 / penStepCorrection) + (long)penStepError;

        // Calc Steps to go, which are possible on our machine
        int rotStepsToGo = (int)(rotSteps / 16);
        int penStepsToGo = (int)(penSteps / 16);

        // calc Position-Error, if there is one
        rotStepError = (long)rotSteps - ((long)rotStepsToGo * (long)16);
        penStepError = (long)penSteps - ((long)penStepsToGo * (long)16);

        // calc Speed in Integer Math
        long temp_rotSpeed = ((long)rotStepsToGo * (long)1000 / (long)duration);
        long temp_penSpeed = ((long)penStepsToGo * (long)1000 / (long)duration);

        float rotSpeed = (float)abs(temp_rotSpeed); // type cast
        float penSpeed = (float)abs(temp_penSpeed);

        // set Coordinates and Speed
        rotMotor.move(rotStepsToGo); // finally, let us set the target position...
        rotMotor.setSpeed(rotSpeed); // and the Speed!
        penMotor.move(penStepsToGo);
        penMotor.setSpeed(penSpeed);
    }
}

void EBBHardware::moveToDestination()
{
    while (penMotor.distanceToGo() || rotMotor.distanceToGo()) {
        penMotor.runSpeedToPosition(); // Moving.... moving... moving....
        rotMotor.runSpeedToPosition();
    }
}

void EBBHardware::moveOneStep()
{
    if (penMotor.distanceToGo() || rotMotor.distanceToGo()) {
        penMotor.runSpeedToPosition(); // Moving.... moving... moving....
        rotMotor.runSpeedToPosition();
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

void EBBHardware::setEngraverState(int state)
{
    digitalWrite(ENGRAVER_PIN, state);
}

void EBBHardware::setPenState(int upDown)
{
    if (upDown == 0) {
        penServo.write(penDownPos, servoRateDown, true);
    } else {
        penServo.write(penUpPos, servoRateUp, true);
    }
    penState = upDown;
}

int EBBHardware::getPenState()
{
    return penState;
}

void EBBHardware::setPenUpPos(int pos)
{
    // transformation from EBB to PWM-Servo
    penUpPos = (int)((float)(pos - 6000) / (float)133.3);
    EEPROM.put(EEPROM_PEN_UP_POS, penUpPos);
}

void EBBHardware::setPenDownPos(int pos)
{
    // transformation from EBB to PWM-Servo
    penDownPos = (int)((float)(pos - 6000) / (float)133.3);
    EEPROM.put(EEPROM_PEN_DOWN_POS, penDownPos);
}

void EBBHardware::setServoRateUp(int rate)
{
    servoRateUp = rate;
    EEPROM.put(EEPROM_PEN_UP_RATE, servoRateUp);
}

void EBBHardware::setServoRateDown(int rate)
{
    servoRateDown = rate;
    EEPROM.put(EEPROM_PEN_DOWN_RATE, servoRateDown);
}

int EBBHardware::getPrgButtonState()
{
    int state = prgButtonState;
    prgButtonState = 0;
    return state;
}

