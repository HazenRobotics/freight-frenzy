package org.firstinspires.ftc.teamcode.drives;

public interface Drive {

	enum State {
		STOPPED,
		MOVING
	}

	/**
	 * Moves the robot forward and backward.
	 *
	 * @param power power at which to run the motors (+ is forward, - is backwards)
	 */
	void move( double power );

	/**
	 * Turns the robot on the spot
	 *
	 * @param power power at which to run the motors (+ is a right turn, - is a left turn)
	 */
	void turn( double power );

	/**
	 * Stops Motors
	 */
	void stop( );

	/**
	 * Sets power to the wheel motors
	 *
	 * @param move power for forward and back motion
	 * @param turn power for rotating the robot
	 */
	void drive( double move, double turn );

	/**
	 * @param distanceToMove
	 * @param circumference
	 * @return
	 */
	int convertDistTicks( double distanceToMove, double circumference );

	/**
	 * Returns the current state of the drive train
	 *
	 * @return current state
	 */
	State getState( );


}
