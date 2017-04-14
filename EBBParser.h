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

    bool motorEnabled;
    bool prgButtonState;

    void enableMotor(int axis, int value);
    void doTogglePen();

private:
    void parseStream();
    void sendAck();
    void sendError();

    void parseEM(const char* arg1, const char* arg2);
    void parseND();
    void parseNI();
    void parsePO(const char* arg1, const char* arg2, const char* arg3);
    void parseQB();
    void parseQL();
    void parseQN();
    void parseQP();
    void parseSC(const char* arg1, const char* arg2);
    void parseSE(const char* arg);
    void parseSL(const char* arg);
    void parseSM(const char* arg1, const char* arg2, const char* arg3);
    void parseSN(const char* arg);
    void parseSP(const char* arg1, const char* arg2, const char* arg3);
    void parseTP(const char* arg);
    void parseV();

    void setPenUp();
    void setPenDown();
    void prepareMove(int duration, int penStepsEBB, int rotStepsEBB);
    void moveToDestination();
    void moveOneStep();

    Stream& mStream;

    AccelStepper rotMotor;
    AccelStepper penMotor;
    VarSpeedServo penServo;

    int penMin;
    int penMax;
    short penUpPos; //eeprom
    short penDownPos; //eeprom
    short servoRateUp; //eeprom
    short servoRateDown; //eeprom
    long rotStepError;
    long penStepError;
    int penState;
    String readBuffer;
    unsigned int nodeCount;
    unsigned int layer;
};
