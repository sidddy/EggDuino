#pragma once

// implemented Eggbot-Protocol-Version v13
// EBB-Command-Reference, I sourced from: http://www.schmalzhaus.com/EBB/EBBParser.html
// no homing sequence, switch-on position of pen will be taken as reference point.
// No collision-detection!!
// Supported Servos: I do not know, I use Arduino Servo Lib with TG9e- standard servo.
// Note: Maximum-Speed in Inkscape is 1000 Steps/s. You could enter more, but then Pythonscript sends nonsense.
// EBB-Coordinates are coming in for 16th-Microstepmode. The Coordinate-Transforms are done in weired integer-math. Be careful, when you diecide to modify settings.
#include <Stream.h>

#include <AccelStepper.h>
#include <VarSpeedServo.h>

class EBBParser {
public:
    EBBParser(Stream& stream);

    void init();
    void processEvents();

    bool motorsEnabled;
    bool prgButtonState;

    void motorsOff();
    void motorsOn();
    void doTogglePen();

private:
    void queryPen();
    void queryButton();
    void queryLayer();
    void setLayer(const char* arg);
    void queryNodeCount();
    void setNodeCount(const char* arg);
    void nodeCountIncrement();
    void nodeCountDecrement();
    void stepperMove(const char* arg1, const char* arg2, const char* arg3);
    void setPenState(const char* arg1, const char* arg2, const char* arg3);
    void togglePen(const char* arg);
    void enableMotors(const char* arg1, const char* arg2);
    void stepperAndServoModeConfigure(const char* arg1, const char* arg2);
    void pinOutput(const char* arg1, const char* arg2, const char* arg3);
    void setEngraver(const char* arg);
    void sendVersion();

    void setPenUp();
    void setPenDown();
    void prepareMove(int duration, int penStepsEBB, int rotStepsEBB);
    void moveToDestination();
    void moveOneStep();

    void readStream();
    void sendAck();
    void sendError();

    Stream& mStream;

    AccelStepper rotMotor;
    AccelStepper penMotor;
    VarSpeedServo penServo;

    int penMin;
    int penMax;
    int penUpPos;
    int penDownPos;
    int servoRateUp;
    int servoRateDown;
    long rotStepError;
    long penStepError;
    int penState;
    String readBuffer;
    unsigned int nodeCount;
    unsigned int layer;
};
