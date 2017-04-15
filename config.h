#pragma once

// Rotational Stepper: ("X")
#define X_STEP_PIN 7
#define X_DIR_PIN 4
#define X_ENABLE_PIN 8
#define X_MICROSTEPPING 16 //MicrostepMode, only 1,2,4,8,16 allowed, because of Integer-Math in this Sketch

// Pen Stepper:        ("Y")
#define Y_STEP_PIN 6
#define Y_DIR_PIN 3
#define Y_ENABLE_PIN 8
#define Y_MICROSTEPPING 16 //MicrostepMode, only 1,2,4,8,16 allowed, because of Integer-Math in this Sketch

// Servo
#define SERVO_PIN A5 // "SpnEn"
#define ENGRAVER_PIN A4 // "SpnDir"

