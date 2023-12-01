/*
 * pid.c
 */

#include "main.h"
#include "motors.h"
<<<<<<< Updated upstream
#include <math.h>
=======
#include "encoders.h"
>>>>>>> Stashed changes


int angleError = 0;
int oldAngleError = 0;
float distanceError = 0;
float oldDistanceError = 0;
float kPw = .1;
float kDw = .1;
float kPx = .01;
float kDx = .5;

int rolloverDistanceError = 0;
int rolloverAngleError = 0;
int goalAngle = 0;
int goalDistanceEncoder = 0;

<<<<<<< Updated upstream
const int INST_LEN = 8;
int angleInstructions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int distanceInstructions[8] = {2000, 0, 630, 0, 1900, 0, 630, 0};
int instructionIndex = -1;
=======
int hasPIDran;
int PIDfinished;
>>>>>>> Stashed changes

void resetPID() {
	/*
	 * For assignment 3.1: This function does not need to do anything
	 * For assignment 3.2: This function should reset all the variables you define in this file to help with PID to their default
	 *  values. You should also reset your motors and encoder counts (if you tell your rat to turn 90 degrees, there will be a big
	 * difference in encoder counts after it turns. If you follow that by telling your rat to drive straight without first
	 * resetting the encoder counts, your rat is going to see a huge angle error and be very unhappy).
	 *
	 * You should additionally set your distance and error goal values (and your oldDistanceError and oldAngleError) to zero.
	 */
	angleError = 0;
	oldAngleError = 0;
	distanceError = 0;
	oldDistanceError = 0;
	resetEncoders();
<<<<<<< Updated upstream
=======
	hasPIDran = 0;
	PIDfinished = 0;
>>>>>>> Stashed changes
}

void updatePID() {
	/*
	 * This function will do the heavy lifting PID logic. It should do the following: read the encoder counts to determine an error,
	 * use that error along with some PD constants you determine in order to determine how to set the motor speeds, and then actually
	 * set the motor speeds.
	 *
	 * For assignment 3.1: implement this function to get your rat to drive forwards indefinitely in a straight line. Refer to pseudocode
	 * example document on the google drive for some pointers
	 *
	 * TIPS (assignment 3.1): Create kPw and kDw variables, and use a variable to store the previous error for use in computing your
	 * derivative term. You may get better performance by having your kDw term average the previous handful of error values instead of the
	 * immediately previous one, or using a stored error from 10-15 cycles ago (stored in an array?). This is because systick calls so frequently
	 * that the error change may be very small and hard to operate on.
	 *
	 * For assignment 3.2: implement this function so it calculates distanceError as the difference between your goal distance and the average of
	 * your left and right encoder counts. Calculate angleError as the difference between your goal angle and the difference between your left and
	 * right encoder counts. Refer to pseudocode example document on the google drive for some pointers.
	 */

	if (PIDdone() && instructionIndex < INST_LEN - 1) {
		instructionIndex++;
		resetEncoders();
		setPIDGoalA(angleInstructions[instructionIndex]);
		setPIDGoalD(distanceInstructions[instructionIndex]);
		if (instructionIndex == INST_LEN - 1) {
			setPIDGoalA(0);
			setPIDGoalD(0);
		}
	}

	// These should get updated by your setPIDGoal function

	angleError = goalAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	float angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
	oldAngleError = angleError;

	distanceError = goalDistanceEncoder - ((getLeftEncoderCounts() + getRightEncoderCounts()) / 2);
	float distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
	oldDistanceError = distanceError;

	setMotorLPWM(distanceCorrection + angleCorrection);
	setMotorRPWM(distanceCorrection - angleCorrection);

<<<<<<< Updated upstream
=======
	if (myAbs(distanceError) < 25 && myAbs(angleError) < 25) {
		hasPIDran++;
	}
>>>>>>> Stashed changes

}

float mmToEncoder(float mm) {
	return mm / (360 * 30 * 3.14159265);
}
void setPIDGoalD(int16_t distance) {
	/*
	 * For assignment 3.1: this function does not need to do anything.
	 * For assignment 3.2: this function should set a variable that stores the goal distance.
	 */
	rolloverAngleError = 0;
	goalDistanceEncoder = distance;// + rolloverDistanceError;
}

void setPIDGoalA(int16_t angle) {
	/*
	 * For assignment 3.1: this function does not need to do anything
	 * For assignment 3.2: This function should set a variable that stores the goal angle.
	 */
<<<<<<< Updated upstream
	goalAngle = angle*460/90; //460 = pi/2
=======
	goalAngle = angle*465/90 + rolloverAngleError; //460 = pi/2
>>>>>>> Stashed changes
}

int myAbs(int a) {
	if (a < 0) return -a;
	return a;
}
int PIDdone() { // There is no bool type in C. True/False values are represented as 1 or 0.
	/*
	 * For assignment 3.1: this function does not need to do anything (your rat should just drive straight indefinitely)
	 * For assignment 3.2:This function should return true if the rat has achieved the set goal. One way to do this by having updatePID() set some variable when
	 * the error is zero (realistically, have it set the variable when the error is close to zero, not just exactly zero). You will have better results if you make
	 * PIDdone() return true only if the error has been sufficiently close to zero for a certain number, say, 50, of SysTick calls in a row.
	 */
<<<<<<< Updated upstream
	if (myAbs(distanceError) < 25 && myAbs(angleError) < 25) {

		return 1;
	} else {
		return 0;
	}
=======
	while(1) {
		if (myAbs(distanceError) < 25 && myAbs(angleError) < 10 && PIDfinished == 1) {
			//rolloverDistanceError = distanceError;
			rolloverAngleError = angleError;
			resetPID();
			return 0;
			}
		}
	}

int PIDrest() {
	if (hasPIDran > 1) {
		PIDfinished = 1;
	}
	return 0;
>>>>>>> Stashed changes
}
