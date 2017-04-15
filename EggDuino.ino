#include "config.h"
#include "EBBParser.h"
#include "Button.h"

EBBParser ebb(Serial);

// Buttons
#define PRG_BUTTON_PIN A2 // PRG button ("Abort")
#define PEN_TOGGLE_BUTTON_PIN A1 // pen up/down button ("Hold")
#define MOTORS_BUTTON_PIN A0 // motors enable button ("Resume")

void toggleMotors()
{
    if (ebb.motorEnabled) {
        ebb.enableMotor(0, 0);
        ebb.enableMotor(1, 0);
    } else {
        ebb.enableMotor(0, 5);
        ebb.enableMotor(1, 5);
    }
}

void setprgButtonState()
{
    ebb.prgButtonState = true;
}

void togglePen()
{
    ebb.doTogglePen();
}

//create Buttons
#ifdef PRG_BUTTON_PIN
Button prgButtonToggle(PRG_BUTTON_PIN, setprgButtonState);
#endif
#ifdef PEN_TOGGLE_BUTTON_PIN
Button penToggle(PEN_TOGGLE_BUTTON_PIN, togglePen);
#endif
#ifdef MOTORS_BUTTON_PIN
Button motorsToggle(MOTORS_BUTTON_PIN, toggleMotors);
#endif

void setup()
{
    Serial.begin(9600);
    ebb.init();
}

void loop()
{
    ebb.processEvents();

#ifdef PEN_TOGGLE_BUTTON_PIN
    penToggle.check();
#endif

#ifdef MOTORS_BUTTON_PIN
    motorsToggle.check();
#endif

#ifdef PRG_BUTTON_PIN
    prgButtonToggle.check();
#endif
}
