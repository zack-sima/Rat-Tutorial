/*
 * systick.c
 */

#include "main.h"
#include "pid.h"
#include "encoders.h"
#include "motors.h"

int angleError = 0;
int oldAngleError = 0;
float distanceError = 0;
float oldDistanceError = 0;
float kPw = 1;
float kDw = 1;
float kPx = 1;
float kDx = 1;

// These should get updated by your setPIDGoal functions
int goalAngle = 0;
int goalDistance = 0;

void UpdatePID() {
	angleError = goalAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	float angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
	oldAngleError = angleError;

	distanceError = goalDistance - (getLeftEncoderCounts() + getRightEncoderCounts()) / 2;
	float distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
	oldDistanceError = distanceError;

	setMotorLPWM(distanceCorrection + angleCorrection);
	setMotorRPWM(distanceCorrection - angleCorrection);
}
void SysTickFunction(void) {
	/*
	 * Anything in this function body will be executed every millisecond.
	 * Call you PID update function here.
	 */

	UpdatePID();


	/*
	 * This code prevents PID errors when the encoder counts get too high. For example, without it then
	 * if the wheels are spinning forwards and PID is trying to keep their encoder counts equal, and one of the wheels
	 * turns past 32767 and its encoder counts loop back to -32768, suddenly the difference between the encoder counts
	 * is massive and the wheels will start spinning at full speed in opposite directions to even back out.
	 *
	 * This code helps prevent that by reseting the encoder counts when any of the encoder values get too extreme.
	 * It also maintains the magnitude of the difference between the left and right encoders so that PID will work seamlessly.
	 *
	 * You may have to adjust it in the event you try to traverse more than 31000 encoder counts at once (~9 meters) without
	 * turning. But that'll be approximately never in Micromouse :)
	 */
	if (getRightEncoderCounts() > 31000 || getLeftEncoderCounts() > 31000
			|| getRightEncoderCounts() < -31000 || getLeftEncoderCounts() < -31000) {
		int16_t difference = getRightEncoderCounts() - getLeftEncoderCounts();
		resetEncoders();
		TIM2->CNT = (int16_t) difference; //set right encoder counts to difference
	}
}
