package org.firstinspires.ftc.teamcode.drives;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FourWheelDrive implements Drive {

	public DcMotorEx frontLeft;
	public DcMotorEx frontRight;
	public DcMotorEx backLeft;
	public DcMotorEx backRight;

	final double PULSES_PER_REVOLUTION = 250;
	final double GEAR_RATIO = 0.25;

	private State currentState = State.STOPPED;

	public FourWheelDrive( HardwareMap hw ) {
		this( hw, "frontLeft", "frontRight", "backLeft", "backRight" );
	}

	public FourWheelDrive( HardwareMap hw, String frontLeftName, String frontRightName, String backLeftName, String backRightName ) {
		setUpMotors( hw, frontLeftName, frontRightName, backLeftName, backRightName );
	}

	/**
	 * @param distanceToTravel the distance to move in inches
	 * @param circumference    the circumference of the wheel that has the encoder
	 * @return totalTicks - the amount of ticks to move forward
	 */
	public int convertDistTicks( double distanceToTravel, double circumference ) {
		double revolutions = distanceToTravel / circumference;
		int totalTicks = (int) Math.round( (revolutions * PULSES_PER_REVOLUTION) / GEAR_RATIO );

		return totalTicks;
	}

	public int convertTicksDist( double ticksToTravel, double circumference ) {
		double calculations = ticksToTravel * circumference * GEAR_RATIO;
		int totalDistance = (int) Math.round( calculations / PULSES_PER_REVOLUTION );

		return totalDistance;
	}

	/**
	 * Sets up motors from the hardware map
	 *
	 * @param hw             robot's hardware map
	 * @param frontRightName name of front right motor in the hardware map
	 * @param frontLeftName  name of front left motor in the hardware map
	 * @param backRightName  name of back right motor in the hardware map
	 * @param backLeftName   name of back left motor in the hardware map
	 */
	private void setUpMotors( HardwareMap hw, String frontLeftName, String frontRightName, String backLeftName, String backRightName ) {
		frontLeft = hw.get( DcMotorEx.class, frontLeftName );
		frontRight = hw.get( DcMotorEx.class, frontRightName );
		backLeft = hw.get( DcMotorEx.class, backLeftName );
		backRight = hw.get( DcMotorEx.class, backRightName );

		setMotorDirections( FORWARD, REVERSE, FORWARD, REVERSE );
		setZeroPowerBehavior( BRAKE, BRAKE, BRAKE, BRAKE );
		//setRunMode(STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER );


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
		setMotorPower( 0, 0, 0, 0 );
	}

	@Override
	public void drive( double move, double turn ) {

		// You might have to play with the + or - depending on how your motors are installed
		double frontLeftPower = move + turn;
		double frontRightPower = move - turn;
		double backLeftPower = move + turn;
		double backRightPower = move - turn;

		setMotorPower( frontLeftPower, frontRightPower, backLeftPower, backRightPower );
	}

	@Override
	public State getState( ) {
		updateState( );
		return currentState;
	}

	private void updateState( ) {
		if( frontLeft.getPower( ) != 0 || frontRight.getPower( ) != 0 || backLeft.getPower( ) != 0 || backRight.getPower( ) != 0 )
			currentState = State.MOVING;
		else
			currentState = State.STOPPED;
	}

	/**
	 * Sets specified power to the motors
	 *
	 * @param frontRightPower power at which to run the front right motor.
	 * @param frontLeftPower  power at which to run the front left motor.
	 * @param backRightPower  power at which to run the back right motor.
	 * @param backLeftPower   power at which to run the back left motor.
	 */
	protected void setMotorPower( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {
		frontLeft.setPower( frontLeftPower );
		frontRight.setPower( frontRightPower );
		backLeft.setPower( backLeftPower );
		backRight.setPower( backRightPower );
	}

	/**
	 * Sets the direction of the motors
	 *
	 * @param frontLeftDirection  direction of the front left motor
	 * @param frontRightDirection direction of the front right motor
	 * @param backLeftDirection   direction of the back left motor
	 * @param backRightDirection  direction of the back right motor
	 */
	public void setMotorDirections( Direction frontLeftDirection, Direction frontRightDirection, Direction backLeftDirection, Direction backRightDirection ) {
		frontLeft.setDirection( frontLeftDirection );
		frontRight.setDirection( frontRightDirection );
		backLeft.setDirection( backLeftDirection );
		backRight.setDirection( backRightDirection );
	}

	/**
	 * Sets the zero power behavior of the motors
	 *
	 * @param frontLeftBehavior  zero power behavior of the front left motor
	 * @param frontRightBehavior zero power behavior of the front right motor
	 * @param backLeftBehavior   zero power behavior of the back left motor
	 * @param backRightBehavior  zero power behavior of the back right motor
	 */
	public void setZeroPowerBehavior( ZeroPowerBehavior frontLeftBehavior, ZeroPowerBehavior frontRightBehavior, ZeroPowerBehavior backLeftBehavior, ZeroPowerBehavior backRightBehavior ) {
		frontLeft.setZeroPowerBehavior( frontLeftBehavior );
		frontRight.setZeroPowerBehavior( frontRightBehavior );
		backLeft.setZeroPowerBehavior( backLeftBehavior );
		backRight.setZeroPowerBehavior( backRightBehavior );
	}

	/**
	 * Sets the run modes of the motors
	 *
	 * @param frontLeftMode  run mode of the front left motor
	 * @param frontRightMode run mode of the front right motor
	 * @param backLeftMode   run mode of the back left motor
	 * @param backMode       run mode of the back right motor
	 */
	public void setRunMode( RunMode frontLeftMode, RunMode frontRightMode, RunMode backLeftMode, RunMode backMode ) {
		frontLeft.setMode( frontLeftMode );
		frontRight.setMode( frontRightMode );
		backLeft.setMode( backLeftMode );
		backRight.setMode( backMode );
	}

	public double getFrontLeftPower( ) {
		return frontLeft.getPower( );
	}

	public double getFrontRightPower( ) {
		return frontRight.getPower( );
	}

	public double getBackLeftPower( ) {
		return backLeft.getPower( );
	}

	public double getBackRightPower( ) {
		return backRight.getPower( );
	}

	public int getFrontLeftPosition( ) {
		return frontLeft.getCurrentPosition( );
	}

	public int getFrontRightPosition( ) {
		return frontRight.getCurrentPosition( );
	}

	public int getBackLeftPosition( ) {
		return backLeft.getCurrentPosition( );
	}

	public int getBackRightPosition( ) {
		return backRight.getCurrentPosition( );
	}
}
