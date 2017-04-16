#pragma once

// implemented Eggbot-Protocol-Version v13
// EBB-Command-Reference, I sourced from: http://www.schmalzhaus.com/EBB/EBBParser.html
// no homing sequence, switch-on position of pen will be taken as reference point.
// No collision-detection!!
// Supported Servos: I do not know, I use Arduino Servo Lib with TG9e- standard servo.
// Note: Maximum-Speed in Inkscape is 1000 Steps/s. You could enter more, but then Pythonscript
// sends nonsense.
// EBB-Coordinates are coming in for 16th-Microstepmode. The Coordinate-Transforms are done in
// weired integer-math. Be careful, when you diecide to modify settings.
#include <Stream.h>

class EBBParser {
public:
    EBBParser(Stream& stream);

    virtual void processEvents();

protected:
    virtual void enableMotor(int axis, int value) = 0;
    virtual void stepperMove(int duration, int penStepsEBB, int rotStepsEBB) = 0;
    virtual void moveOneStep() = 0;
    virtual void moveToDestination() = 0;

    virtual void setPenState(int upDown) = 0;
    virtual int getPenState() = 0;
    virtual void setPenUpPos(int pos) = 0;
    virtual void setPenDownPos(int pos) = 0;

    virtual void setServoRateUp(int rate) = 0;
    virtual void setServoRateDown(int rate) = 0;

    virtual int getPrgButtonState() = 0;
    virtual void setPinOutput(char port, int pin, int value) = 0;

    virtual void setEngraverState(int state) = 0;
    virtual void setEngraverPower(int power) = 0;
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
    void parseSE(const char* arg1, const char* arg2, const char* arg3);
    void parseSL(const char* arg);
    void parseSM(const char* arg1, const char* arg2, const char* arg3);
    void parseSN(const char* arg);
    void parseSP(const char* arg1, const char* arg2, const char* arg3);
    void parseTP(const char* arg);
    void parseV();

    Stream& mStream;

    String readBuffer;
    unsigned int nodeCount;
    unsigned int layer;
};
