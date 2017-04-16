#pragma once

#include "config.h"
#include "EBBParser.h"
#include "Button.h"

#include <AccelStepper.h>
#include <VarSpeedServo.h>

class EBBHardware : public EBBParser {
public:
    EBBHardware(Stream& stream);

    void init();

    virtual void processEvents();

protected:
    virtual void enableMotor(int axis, int value);
    virtual void stepperMove(int duration, int penStepsEBB, int rotStepsEBB);

    virtual void setPenState(int upDown);
    virtual int getPenState();
    virtual void setPenUpPos(int pos);
    virtual void setPenDownPos(int pos);

    virtual void setServoRateUp(int rate);
    virtual void setServoRateDown(int rate);

    virtual int getPrgButtonState();

    virtual void moveToDestination();
    virtual void moveOneStep();

    virtual void setPinOutput(char port, int pin, int value);

    virtual void setEngraverState(int state);
    virtual void setEngraverPower(int power)
    {
    }

private:
    AccelStepper rotMotor;
    AccelStepper penMotor;
    VarSpeedServo penServo;

    int penState;
    short penUpPos; // eeprom!
    short penDownPos; // eeprom!

    short servoRateUp; // eeprom
    short servoRateDown; // eeprom

    long rotStepError;
    long penStepError;

    bool motorEnabled;

    bool prgButtonState;

// create Buttons
#ifdef PRG_BUTTON_PIN
    Button prgButtonToggle;
#endif
#ifdef PEN_TOGGLE_BUTTON_PIN
    Button penToggle;
#endif
#ifdef MOTORS_BUTTON_PIN
    Button motorsToggle;
#endif
};
