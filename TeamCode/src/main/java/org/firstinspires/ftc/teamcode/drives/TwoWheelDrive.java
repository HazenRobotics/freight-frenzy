package org.firstinspires.ftc.teamcode.drives;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelDrive implements Drive {

	public DcMotorEx left;
	public DcMotorEx right;

	final double PULSES_PER_REVOLUTION = 250;
	final double GEAR_RATIO = 0.25;

	private Drive.State currentState = Drive.State.STOPPED;

	public TwoWheelDrive( HardwareMap hw ) {
		this( hw, "left", "right" );
	}

	public TwoWheelDrive( HardwareMap hw, String leftName, String rightName ) {
		setUpMotors( hw, leftName, rightName );
	}

	/**
	 * convert a distance (in inches) to ticks
	 *
	 * @param distanceToTravel the distance to move in inches
	 * @param circumference    the circumference of the wheel that has the encoder
	 * @return totalTicks - the amount of ticks to move forward
	 */
	public int convertDistTicks( double distanceToTravel, double circumference ) {
		return (int) Math.round( ((distanceToTravel / circumference) * PULSES_PER_REVOLUTION) / GEAR_RATIO );
	}

	/**
	 * convert a number of ticks to distance (in inches)
	 *
	 * @param ticksToTravel the number of ticks to travel
	 * @param circumference the circumference of the wheel (2*pi*r or pi*d)
	 * @return the distance (in inches) in that number of ticks
	 */
	public int convertTicksDist( double ticksToTravel, double circumference ) {
		return (int) Math.round( (ticksToTravel * circumference * GEAR_RATIO) / PULSES_PER_REVOLUTION );
	}

	/**
	 * Sets up motors from the hardware map
	 *
	 * @param hw        robot's hardware map
	 * @param leftName  name of front left motor in the hardware map
	 * @param rightName name of front right motor in the hardware map
	 */
	private void setUpMotors( HardwareMap hw, String leftName, String rightName ) {
		left = hw.get( DcMotorEx.class, leftName );
		right = hw.get( DcMotorEx.class, rightName );

		setMotorDirections( FORWARD, REVERSE );
		setZeroPowerBehavior( BRAKE, BRAKE );
		//setRunMode(STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER );


	}

	@Override
	public void move( double power ) {
		drive( power, 0 );
	}

	@Override
	public void turn( double power ) {
		drive( 0, power );
	}

	@Override
	public void stop( ) {
		setMotorPower( 0, 0 );
	}

	@Override
	public void drive( double move, double turn ) {

		// You might have to play with the + or - depending on how your motors are installed
		double leftPower = move + turn;
		double rightPower = move - turn;

		setMotorPower( leftPower, rightPower );
	}

	@Override
	public Drive.State getState( ) {
		updateState( );
		return currentState;
	}

	private void updateState( ) {
		if( left.getPower( ) != 0 || right.getPower( ) != 0 )
			currentState = Drive.State.MOVING;
		else
			currentState = Drive.State.STOPPED;
	}

	/**
	 * Sets specified power to the motors
	 *
	 * @param leftPower  power at which to run the left motor.
	 * @param rightPower power at which to run the right motor.
	 */
	protected void setMotorPower( double leftPower, double rightPower ) {
		left.setPower( leftPower );
		right.setPower( rightPower );
	}

	/**
	 * Sets the direction of the motors
	 *
	 * @param leftDirection  direction of the left motor
	 * @param rightDirection direction of the right motor
	 */
	public void setMotorDirections( DcMotorSimple.Direction leftDirection, DcMotorSimple.Direction rightDirection ) {
		left.setDirection( leftDirection );
		right.setDirection( rightDirection );
	}

	/**
	 * Sets the zero power behavior of the motors
	 *
	 * @param leftBehavior  zero power behavior of the left motor
	 * @param rightBehavior zero power behavior of the right motor
	 */
	public void setZeroPowerBehavior( DcMotor.ZeroPowerBehavior leftBehavior, DcMotor.ZeroPowerBehavior rightBehavior ) {
		left.setZeroPowerBehavior( leftBehavior );
		right.setZeroPowerBehavior( rightBehavior );
	}

	/**
	 * Sets the run modes of the motors
	 *
	 * @param leftMode  run mode of the left motor
	 * @param rightMode run mode of the right motor
	 */
	public void setRunMode( DcMotor.RunMode leftMode, DcMotor.RunMode rightMode ) {
		left.setMode( leftMode );
		right.setMode( rightMode );
	}

	public double getLeftPower( ) {
		return left.getPower( );
	}


	public double getRightPower( ) {
		return right.getPower( );
	}


	public int getLeftPosition( ) {
		return left.getCurrentPosition( );
	}


	public int getRightPosition( ) {
		return right.getCurrentPosition( );
	}

}
