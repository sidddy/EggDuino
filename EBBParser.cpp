#include "EBBParser.h"
#include "config.h"

#include <avr/eeprom.h>

#define rotStepCorrection (16 / X_MICROSTEPPING) //devide EBB-Coordinates by this factor to get EGGduino-Steps
#define penStepCorrection (16 / Y_MICROSTEPPING) //devide EBB-Coordinates by this factor to get EGGduino-Steps

#define penUpPosEEAddress ((uint16_t*)0)
#define penDownPosEEAddress ((uint16_t*)2)
#define penUpRateEEAddress ((uint16_t*)4)
#define penDownRateEEAddress ((uint16_t*)6)

EBBParser::EBBParser()
    : motorsEnabled(false)
    , prgButtonState(false)
    , rotMotor(1, X_STEP_PIN, X_DIR_PIN)
    , penMotor(1, Y_STEP_PIN, Y_DIR_PIN)
    , penMin(0)
    , penMax(0)
    , penUpPos(5) //can be overwritten from EBB-Command SC
    , penDownPos(20) //can be overwritten from EBB-Command SC
    , servoRateUp(0)
    , servoRateDown(0)
    , rotStepError(0)
    , penStepError(0)
    , penState(penUpPos)
    , nodeCount(0)
    , layer(0)
{
    readBuffer.reserve(64);
}

void EBBParser::init()
{
    // enable eeprom wait in avr/eeprom.h functions
    SPMCSR &= ~SELFPRGEN;
    penUpPos = eeprom_read_word(penUpPosEEAddress);
    penDownPos = eeprom_read_word(penDownPosEEAddress);
    servoRateUp = eeprom_read_word(penUpRateEEAddress);
    servoRateDown = eeprom_read_word(penDownRateEEAddress);
    penState = penUpPos;

    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);
    pinMode(ENGRAVER_PIN, OUTPUT);

    rotMotor.setMaxSpeed(2000.0);
    rotMotor.setAcceleration(10000.0);
    penMotor.setMaxSpeed(2000.0);
    penMotor.setAcceleration(10000.0);
    motorsOff();
    penServo.attach(SERVO_PIN);
    penServo.write(penState);
}

void EBBParser::processEvents()
{
    moveOneStep();
    readSerial();
}

void EBBParser::sendAck()
{
    Serial.print("OK\r\n");
}

void EBBParser::sendError()
{
    Serial.print("!8 Err: Unknown command\r\n");
}

void EBBParser::motorsOff()
{
    digitalWrite(X_ENABLE_PIN, HIGH);
    digitalWrite(Y_ENABLE_PIN, HIGH);
    motorsEnabled = false;
}

void EBBParser::motorsOn()
{
    digitalWrite(X_ENABLE_PIN, LOW);
    digitalWrite(Y_ENABLE_PIN, LOW);
    motorsEnabled = true;
}

void EBBParser::prepareMove(int duration, int penStepsEBB, int rotStepsEBB)
{
    if (!motorsEnabled) {
        motorsOn();
    }
    if ((1 == rotStepCorrection) && (1 == penStepCorrection)) { // if coordinatessystems are identical
        //set Coordinates and Speed
        rotMotor.move(rotStepsEBB);
        rotMotor.setSpeed(abs((float)rotStepsEBB * (float)1000 / (float)duration));
        penMotor.move(penStepsEBB);
        penMotor.setSpeed(abs((float)penStepsEBB * (float)1000 / (float)duration));
    } else {
        //incoming EBB-Steps will be multiplied by 16, then Integer-maths is done, result will be divided by 16
        // This make thinks here really complicated, but floating point-math kills performance and memory, believe me... I tried...
        long rotSteps = ((long)rotStepsEBB * 16 / rotStepCorrection) + (long)rotStepError; //correct incoming EBB-Steps to our microstep-Setting and multiply  by 16 to avoid floatingpoint...
        long penSteps = ((long)penStepsEBB * 16 / penStepCorrection) + (long)penStepError;

        int rotStepsToGo = (int)(rotSteps / 16); //Calc Steps to go, which are possible on our machine
        int penStepsToGo = (int)(penSteps / 16);

        rotStepError = (long)rotSteps - ((long)rotStepsToGo * (long)16); // calc Position-Error, if there is one
        penStepError = (long)penSteps - ((long)penStepsToGo * (long)16);

        long temp_rotSpeed = ((long)rotStepsToGo * (long)1000 / (long)duration); // calc Speed in Integer Math
        long temp_penSpeed = ((long)penStepsToGo * (long)1000 / (long)duration);

        float rotSpeed = (float)abs(temp_rotSpeed); // type cast
        float penSpeed = (float)abs(temp_penSpeed);

        //set Coordinates and Speed
        rotMotor.move(rotStepsToGo); // finally, let us set the target position...
        rotMotor.setSpeed(rotSpeed); // and the Speed!
        penMotor.move(penStepsToGo);
        penMotor.setSpeed(penSpeed);
    }
}

void EBBParser::moveToDestination()
{
    while (penMotor.distanceToGo() || rotMotor.distanceToGo()) {
        penMotor.runSpeedToPosition(); // Moving.... moving... moving....
        rotMotor.runSpeedToPosition();
    }
}

void EBBParser::moveOneStep()
{
    if (penMotor.distanceToGo() || rotMotor.distanceToGo()) {
        penMotor.runSpeedToPosition(); // Moving.... moving... moving....
        rotMotor.runSpeedToPosition();
    }
}

void EBBParser::readSerial()
{
    if (!Serial.available())
        return;

    const char inChar = Serial.read();
    if (inChar != '\r') {
        if (readBuffer.length() < 64 && isprint(inChar))
            readBuffer += inChar;
        return;
    }

    char* str = readBuffer.begin();
    const char* cmd = strsep(&str, ",");
    const char* arg1 = strsep(&str, ",");
    const char* arg2 = strsep(&str, ",");
    const char* arg3 = strsep(&str, ",");

    if (strcmp(cmd, "v") == 0) {
        sendVersion();
    } else if (strcmp(cmd, "EM") == 0) {
        enableMotors(arg1, arg2);
    } else if (strcmp(cmd, "SC") == 0) {
        stepperModeConfigure(arg1, arg2);
    } else if (strcmp(cmd, "SP") == 0) {
        setPen(arg1, arg2);
    } else if (strcmp(cmd, "SM") == 0) {
        stepperMove(arg1, arg2, arg3);
    } else if (strcmp(cmd, "SE") == 0) {
        setEngraver(arg1);
    } else if (strcmp(cmd, "TP") == 0) {
        togglePen(arg1);
    } else if (strcmp(cmd, "PD") == 0) {
        sendAck();
    } else if (strcmp(cmd, "PO") == 0) {
        pinOutput(arg1, arg2, arg3);
    } else if (strcmp(cmd, "NI") == 0) {
        nodeCountIncrement();
    } else if (strcmp(cmd, "ND") == 0) {
        nodeCountDecrement();
    } else if (strcmp(cmd, "SN") == 0) {
        setNodeCount(arg1);
    } else if (strcmp(cmd, "QN") == 0) {
        queryNodeCount();
    } else if (strcmp(cmd, "SL") == 0) {
        setLayer(arg1);
    } else if (strcmp(cmd, "QL") == 0) {
        queryLayer();
    } else if (strcmp(cmd, "QP") == 0) {
        queryPen();
    } else if (strcmp(cmd, "QB") == 0) {
        queryButton();
    } else
        sendError();

    readBuffer = "";
}

void EBBParser::queryPen()
{
    char state = (penState == penUpPos) ? '0' : '1';
    Serial.print(String(state) + "\r\n");
    sendAck();
}

void EBBParser::queryButton()
{
    Serial.print(String(prgButtonState) + "\r\n");
    sendAck();
    prgButtonState = false;
}

void EBBParser::queryLayer()
{
    Serial.print(String(layer) + "\r\n");
    sendAck();
}

void EBBParser::setLayer(const char* arg)
{
    if (arg != NULL) {
        layer = atoi(arg);
        sendAck();
    } else
        sendError();
}

void EBBParser::queryNodeCount()
{
    Serial.print(String(nodeCount) + "\r\n");
    sendAck();
}

void EBBParser::setNodeCount(const char* arg)
{
    if (arg != NULL) {
        nodeCount = atoi(arg);
        sendAck();
    } else
        sendError();
}

void EBBParser::nodeCountIncrement()
{
    nodeCount++;
    sendAck();
}

void EBBParser::nodeCountDecrement()
{
    nodeCount--;
    sendAck();
}

void EBBParser::stepperMove(const char* arg1, const char* arg2, const char* arg3)
{
    moveToDestination();

    if (!arg1 || !arg2 || !arg3) {
        sendError();
        return;
    }

    int duration = atoi(arg1);
    int axis1 = atoi(arg2);
    int axis2 = atoi(arg3);

    sendAck();

    if ((axis1 == 0) && (axis2 == 0)) {
        delay(duration);
        return;
    }

    prepareMove(duration, axis1, axis2);
}

void EBBParser::setPenUp()
{
    penServo.write(penUpPos, servoRateUp, true);
    penState = penUpPos;
}

void EBBParser::setPenDown()
{
    penServo.write(penDownPos, servoRateDown, true);
    penState = penDownPos;
}

void EBBParser::setPen(const char* arg1, const char* arg2)
{
    moveToDestination();

    if (arg1 != NULL) {
        int cmd = atoi(arg1);
        switch (cmd) {
        case 0:
            sendAck();
            setPenDown();
            break;

        case 1:
            sendAck();
            setPenUp();
            break;

        default:
            sendError();
        }
    }
    if (arg2 != NULL) {
        int value = atoi(arg2);
        delay(value);
    }
}

void EBBParser::togglePen(const char* arg)
{
    moveToDestination();

    int value = (arg != NULL) ? atoi(arg) : 500;

    doTogglePen();
    sendAck();
    delay(value);
}

void EBBParser::doTogglePen()
{
    if (penState == penUpPos) {
        setPenDown();
    } else {
        setPenUp();
    }
}

void EBBParser::enableMotors(const char* arg1, const char* arg2)
{
    //values parsed
    if ((arg1 != NULL) && (arg2 == NULL)) {
        int cmd = atoi(arg1);
        switch (cmd) {
        case 0:
            motorsOff();
            sendAck();
            break;
        case 1:
            motorsOn();
            sendAck();
            break;
        default:
            sendError();
        }
    }
    //the following implementaion is a little bit cheated, because i did not know, how to implement different values for first and second argument.
    if ((arg1 != NULL) && (arg2 != NULL)) {
        int value = atoi(arg2);
        switch (value) {
        case 0:
            motorsOff();
            sendAck();
            break;
        case 1:
            motorsOn();
            sendAck();
            break;
        default:
            sendError();
        }
    }
}

void EBBParser::stepperModeConfigure(const char* arg1, const char* arg2)
{
    if ((arg1 != NULL) && (arg2 != NULL)) {
        int cmd = atoi(arg1);
        int value = atoi(arg2);
        switch (cmd) {
        case 4:
            penUpPos = (int)((float)(value - 6000) / (float)133.3); // transformation from EBB to PWM-Servo
            eeprom_update_word(penUpPosEEAddress, penUpPos);
            sendAck();
            break;
        case 5:
            penDownPos = (int)((float)(value - 6000) / (float)133.3); // transformation from EBB to PWM-Servo
            eeprom_update_word(penDownPosEEAddress, penDownPos);
            sendAck();
            break;
        case 6: //rotMin=value;    ignored
            sendAck();
            break;
        case 7: //rotMax=value;    ignored
            sendAck();
            break;
        case 11:
            servoRateUp = value / 5;
            eeprom_update_word(penUpRateEEAddress, servoRateUp);
            sendAck();
            break;
        case 12:
            servoRateDown = value / 5;
            eeprom_update_word(penDownRateEEAddress, servoRateDown);
            sendAck();
            break;
        default:
            sendError();
        }
    }
}

void EBBParser::pinOutput(const char* arg1, const char* arg2, const char* arg3)
{
    if (arg1 == NULL || arg2 == NULL || arg3 == NULL) {
        sendError();
        return;
    }
    //PO,B,3,0 = disable engraver
    //PO,B,3,1 = enable engraver
    if (arg1[0] == 'B' && arg2[0] == '3') {
        int val = atoi(arg3);
        digitalWrite(ENGRAVER_PIN, val);
    }
    sendAck();
}

//currently inkscape extension is using PO command for engraver instead of SE
void EBBParser::setEngraver(const char* arg)
{
    if (arg != NULL) {
        int val = atoi(arg);
        digitalWrite(ENGRAVER_PIN, val);
    }
    sendAck();
}

void EBBParser::sendVersion()
{
    Serial.print("EBBv13_and_above Protocol emulated by Eggduino-Firmware V1.x\r\n");
}

