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
    , penState(false)
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
    moveOneStep();
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

    // if coordinatessystems are identical
    if ((1 == rotStepCorrection) && (1 == penStepCorrection)) {
        // set Coordinates and Speed
        rotMotor.move(numRotSteps);
        rotMotor.setSpeed(abs((float)numRotSteps * (float)1000 / (float)duration));
        penMotor.move(numPenSteps);
        penMotor.setSpeed(abs((float)numPenSteps * (float)1000 / (float)duration));
    } else {
        // incoming EBB-Steps will be multiplied by 16, then Integer-maths is
        // done, result will be divided by 16
        // This make thinks here really complicated, but floating point-math
        // kills performance and memory, believe me... I tried...
        long rotSteps = ((long)numRotSteps * 16 / rotStepCorrection) + (long)rotStepError;
        // correct incoming EBB-Steps to our
        // microstep-Setting and multiply  by 16 to
        // avoid floatingpoint...
        long penSteps = ((long)numPenSteps * 16 / penStepCorrection) + (long)penStepError;

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

bool EBBHardware::moveOneStep()
{
    if (penMotor.distanceToGo() || rotMotor.distanceToGo()) {
        penMotor.runSpeedToPosition(); // Moving.... moving... moving....
        rotMotor.runSpeedToPosition();
    }
    return true;
}

void EBBHardware::moveToDestination()
{
    while (penMotor.distanceToGo() || rotMotor.distanceToGo()) {
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
