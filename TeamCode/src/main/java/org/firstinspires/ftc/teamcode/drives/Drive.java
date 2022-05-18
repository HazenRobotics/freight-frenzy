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
	 * @param distanceToTravel the distance to travel in inches
	 * @param circumference    the circumference of the wheel that has the encoder
	 * @param gearRatio        the ratio between the motor and the wheel
	 * @param ppr              the number of ticks or pulses of the motor in one revolution
	 * @return the number of ticks to travel
	 */
	int convertDistTicks( double distanceToTravel, double circumference, double gearRatio, double ppr );

	/**
	 * @param ticksToTravel the distance to move in inches
	 * @param circumference the circumference of the wheel that has the encoder
	 * @param gearRatio     the ratio between the motor and the wheel
	 * @param ppr           the number of ticks or pulses of the motor in one revolution
	 * @return the distance to travel
	 */
	double convertTicksDist( double ticksToTravel, double circumference, double gearRatio, double ppr );

	/**
	 * Returns the current state of the drive train
	 *
	 * @return current state
	 */
	State getState( );


}
