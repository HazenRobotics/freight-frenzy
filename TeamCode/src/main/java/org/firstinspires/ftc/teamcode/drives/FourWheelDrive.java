package org.firstinspires.ftc.teamcode.drives;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FourWheelDrive implements Drive {

	public DcMotorEx frontLeft;
	public DcMotorEx frontRight;
	public DcMotorEx backLeft;
	public DcMotorEx backRight;

	final double PULSES_PER_REVOLUTION = 250;
	final double GEAR_RATIO = 0.25;

	private State currentState = State.STOPPED;

	public FourWheelDrive( HardwareMap hardwareMap ) {
		this( hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight" );
	}

	public FourWheelDrive( HardwareMap hardwareMap, String frontLeftName, String frontRightName, String backLeftName, String backRightName ) {
		setUpMotors( hardwareMap, frontLeftName, frontRightName, backLeftName, backRightName );
	}

	/**
	 * @param distanceToTravel the distance to move in inches
	 * @param circumference    the circumference of the wheel that has the encoder
	 * @return the amount of ticks to move forward
	 */
	public int convertDistTicks( double distanceToTravel, double circumference ) {
		return convertDistTicks( distanceToTravel, circumference, GEAR_RATIO );
	}

	/**
	 * @param ticksToTravel the distance to move in inches
	 * @param circumference the circumference of the wheel that has the encoder
	 * @return the distance to move forward
	 */
	public int convertTicksDist( double ticksToTravel, double circumference ) {
		return convertTicksDist( ticksToTravel, circumference, GEAR_RATIO );
	}

	/**
	 * @param distanceToTravel the distance to move in inches
	 * @param circumference    the circumference of the wheel that has the encoder
	 * @param gearRatio        the ratio between the motor and the wheel
	 * @return the amount of ticks to move forward
	 */
	@Override
	public int convertDistTicks( double distanceToTravel, double circumference, double gearRatio ) {
		return (int) Math.round( ((distanceToTravel / circumference) * PULSES_PER_REVOLUTION) / gearRatio );
	}

	/**
	 * @param ticksToTravel the distance to move in inches
	 * @param circumference the circumference of the wheel that has the encoder
	 * @param gearRatio     the ratio between the motor and the wheel
	 * @return the distance to move forward
	 */
	public int convertTicksDist( double ticksToTravel, double circumference, double gearRatio ) {
		double calculations = ticksToTravel * circumference * GEAR_RATIO;
		int totalDistance = (int) Math.round( calculations / PULSES_PER_REVOLUTION );

		return totalDistance;
	}

	/**
	 * Sets up motors from the hardware map
	 *
	 * @param hardwareMap    robot's hardware map
	 * @param frontRightName name of front right motor in the hardware map
	 * @param frontLeftName  name of front left motor in the hardware map
	 * @param backRightName  name of back right motor in the hardware map
	 * @param backLeftName   name of back left motor in the hardware map
	 */
	private void setUpMotors( HardwareMap hardwareMap, String frontLeftName, String frontRightName, String backLeftName, String backRightName ) {
		frontLeft = hardwareMap.get( DcMotorEx.class, frontLeftName );
		frontRight = hardwareMap.get( DcMotorEx.class, frontRightName );
		backLeft = hardwareMap.get( DcMotorEx.class, backLeftName );
		backRight = hardwareMap.get( DcMotorEx.class, backRightName );

		setMotorDirections( FORWARD, FORWARD, REVERSE, REVERSE );
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
		double backLeftPower = move + turn;
		double frontRightPower = move - turn;
		double backRightPower = move - turn;

		setMotorPower( frontLeftPower, backLeftPower, frontRightPower, backRightPower );
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
	 * @param frontLeftPower  power at which to run the front left motor.
	 * @param backLeftPower   power at which to run the back left motor.
	 * @param frontRightPower power at which to run the front right motor.
	 * @param backRightPower  power at which to run the back right motor.
	 */
	protected void setMotorPower( double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower ) {
		frontLeft.setPower( frontLeftPower );
		backLeft.setPower( backLeftPower );
		frontRight.setPower( frontRightPower );
		backRight.setPower( backRightPower );
	}

	/**
	 * Sets the direction of the motors
	 *
	 * @param frontLeftDirection  direction of the front left motor
	 * @param backLeftDirection   direction of the back left motor
	 * @param frontRightDirection direction of the front right motor
	 * @param backRightDirection  direction of the back right motor
	 */
	public void setMotorDirections( Direction frontLeftDirection, Direction backLeftDirection, Direction frontRightDirection, Direction backRightDirection ) {
		frontLeft.setDirection( frontLeftDirection );
		backLeft.setDirection( backLeftDirection );
		frontRight.setDirection( frontRightDirection );
		backRight.setDirection( backRightDirection );
	}

	/**
	 * Sets the zero power behavior of the motors
	 *
	 * @param frontLeftBehavior  zero power behavior of the front left motor
	 * @param backLeftBehavior   zero power behavior of the back left motor
	 * @param frontRightBehavior zero power behavior of the front right motor
	 * @param backRightBehavior  zero power behavior of the back right motor
	 */
	public void setZeroPowerBehavior( ZeroPowerBehavior frontLeftBehavior, ZeroPowerBehavior backLeftBehavior, ZeroPowerBehavior frontRightBehavior, ZeroPowerBehavior backRightBehavior ) {
		frontLeft.setZeroPowerBehavior( frontLeftBehavior );
		backLeft.setZeroPowerBehavior( backLeftBehavior );
		frontRight.setZeroPowerBehavior( frontRightBehavior );
		backRight.setZeroPowerBehavior( backRightBehavior );
	}

	/**
	 * Sets the run modes of the motors
	 *
	 * @param frontLeftMode  run mode of the front left motor
	 * @param backLeftMode   run mode of the back left motor
	 * @param frontRightMode run mode of the front right motor
	 * @param backRightMode  run mode of the back right motor
	 */
	public void setRunMode( RunMode frontLeftMode, RunMode frontRightMode, RunMode backLeftMode, RunMode backRightMode ) {
		frontLeft.setMode( frontLeftMode );
		backLeft.setMode( backLeftMode );
		frontRight.setMode( frontRightMode );
		backRight.setMode( backRightMode );
	}

	public double getFrontLeftPower( ) {
		return frontLeft.getPower( );
	}

	public double getBackLeftPower( ) {
		return backLeft.getPower( );
	}

	public double getFrontRightPower( ) {
		return frontRight.getPower( );
	}

	public double getBackRightPower( ) {
		return backRight.getPower( );
	}


	public double getFrontLeftVelocity( ) {
		return frontLeft.getVelocity( );
	}

	public double getBackLeftVelocity( ) {
		return backLeft.getVelocity( );
	}

	public double getFrontRightVelocity( ) {
		return frontRight.getVelocity( );
	}

	public double getBackRightVelocity( ) {
		return backRight.getVelocity( );
	}


	public double getFrontLeftVelocity( AngleUnit angleUnit ) {
		return frontLeft.getVelocity( angleUnit );
	}

	public double getBackLeftVelocity( AngleUnit angleUnit ) {
		return backLeft.getVelocity( angleUnit );
	}

	public double getFrontRightVelocity( AngleUnit angleUnit ) {
		return frontRight.getVelocity( angleUnit );
	}

	public double getBackRightVelocity( AngleUnit angleUnit ) {
		return backRight.getVelocity( angleUnit );
	}


	public int getFrontLeftPosition( ) {
		return frontLeft.getCurrentPosition( );
	}

	public int getBackLeftPosition( ) {
		return backLeft.getCurrentPosition( );
	}

	public int getFrontRightPosition( ) {
		return frontRight.getCurrentPosition( );
	}

	public int getBackRightPosition( ) {
		return backRight.getCurrentPosition( );
	}
}
