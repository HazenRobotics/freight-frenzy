package org.firstinspires.ftc.teamcode.drives;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Vector;

public class SwerveDrive implements Drive {

	public DcMotorEx leftTop;
	public DcMotorEx leftBottom;
	public DcMotorEx rightTop;
	public DcMotorEx rightBottom;

	final double PULSES_PER_REVOLUTION = 250; // 537.7

	private State currentState = State.STOPPED;

	public double wheelRadius;
	public double wheelGearRatio;
	public double rotateGearRadius;
	public double rotateGearRatio;

	int ticksInRotation = 0;

	public SwerveDrive( HardwareMap hardwareMap ) {
		setUpMotors( hardwareMap, "leftTop", "leftBottom", "rightTop", "rightBottom" );
	}

	public SwerveDrive( HardwareMap hardwareMap, String leftTopName, String rightTopName, String leftBottomName, String rightBottomName ) {
		setUpMotors( hardwareMap, leftTopName, rightTopName, leftBottomName, rightBottomName );
	}

	/**
	 * @param distanceToTravel the distance to move in inches
	 * @param circumference    the circumference of the wheel that has the encoder
	 * @param gearRatio        the ratio between the motor and the wheel
	 * @return the amount of ticks to move forward
	 */
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
		return (int) Math.round( ticksToTravel * circumference * gearRatio / PULSES_PER_REVOLUTION );
	}

	/**
	 * Sets up motors from the hardware map
	 *
	 * @param hardwareMap     robot's hardware map
	 * @param rightTopName    name of left top motor in the hardware map
	 * @param leftTopName     name of left bottom motor in the hardware map
	 * @param rightBottomName name of right top motor in the hardware map
	 * @param leftBottomName  name of right bottom motor in the hardware map
	 */
	private void setUpMotors( HardwareMap hardwareMap, String leftTopName, String rightTopName, String leftBottomName, String rightBottomName ) {
		leftTop = hardwareMap.get( DcMotorEx.class, leftTopName );
		rightTop = hardwareMap.get( DcMotorEx.class, rightTopName );
		leftBottom = hardwareMap.get( DcMotorEx.class, leftBottomName );
		rightBottom = hardwareMap.get( DcMotorEx.class, rightBottomName );

		setMotorDirections( FORWARD, FORWARD, REVERSE, REVERSE );
		setZeroPowerBehavior( BRAKE, BRAKE, BRAKE, BRAKE );
		//setRunMode(STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER );
	}

	public void setUpWheelRatios( double wheelRadius, double wheelGearRatio, double rotateGearRadius, double wheelGearRadius ) {

		this.wheelRadius = wheelRadius;
		this.wheelGearRatio = wheelGearRatio;

		this.rotateGearRadius = rotateGearRadius;
		this.rotateGearRatio = wheelGearRadius;

		ticksInRotation = convertDistTicks( 2 * Math.PI, 2 * Math.PI * rotateGearRadius, rotateGearRatio );

	}

	/**
	 * @param topMotor    the motor controlling the wheel hub's upper gear
	 * @param bottomMotor the motor controlling the wheel hub's lower gear
	 * @return the number of ticks the motor hub has rotated
	 */
	public int getWheelRotation( DcMotorEx topMotor, DcMotorEx bottomMotor ) {
		return topMotor.getCurrentPosition( ) - bottomMotor.getCurrentPosition( ) / 2;
	}

	/**
	 * @param topMotor    the motor controlling the wheel hub's upper gear
	 * @param bottomMotor the motor controlling the wheel hub's lower gear
	 * @return the number of radians the motor hub has rotated
	 */
	public int getWheelRotationRadians( DcMotorEx topMotor, DcMotorEx bottomMotor ) {
		return convertTicksDist( getWheelRotation( topMotor, bottomMotor ), 2 * Math.PI * rotateGearRadius, rotateGearRatio );

	}

	/**
	 * @param angle the input angle between 0 and 2π
	 * @return the angle normalized between 0 and 1
	 */
	public double normalizeAngle( double angle ) {
		return normalize( angle, 0, 2 * Math.PI, 0, 1 );
	}

	/**
	 * @param inputNum the number to be normalized
	 * @param oldMin   the minimum the input number could be
	 * @param oldMax   the maximum the input number could be
	 * @param newMin   the minimum the output number could be
	 * @param newMax   the maximum the output number could be
	 * @return the input number normalized to the new range
	 */
	public double normalize( double inputNum, double oldMin, double oldMax, double newMin, double newMax ) {
		return (inputNum - oldMin) / (oldMax - oldMin) * (newMax - newMin) + newMin;
	}

	/**
	 * @param vectorAngle the angle from a vector
	 * @return the angle rotated 90° counterclockwise
	 */
	public double rotateAngle( double vectorAngle ) {
		return Angle.norm( vectorAngle + Math.PI / 2 );
	}

	/**
	 * @param vector the vector to get an angle from
	 * @return the angle rotated 90° counterclockwise
	 */
	public double rotateAngle( Vector2d vector ) {
		return rotateAngle( vector.angle( ) );
	}

	public void move( double drive, double strafe, double rotate ) {
		move( new Vector2d( strafe, drive ), rotate );
	}

	public void move( Vector2d strafe, double rotate ) {

		int leftRotation = getWheelRotationRadians( leftTop, leftBottom );
		int rightRotation = getWheelRotationRadians( rightTop, rightBottom );

		double targetHubAngle = rotateAngle( strafe );
		double targetRotateAngle = 0; // perpendicular to robot's point of rotation

		int width = 14; // inches between wheels

		/*
		drive: similar power to everything
			strafe: if wheel too far left, turn hub right more
					if wheel too far right, turn hub left more
		if rotate right:
		 */

		// too far left: leftRotation < rotateAngle( vector.angle( ) )
		// too far right: leftRotation > rotateAngle( vector.angle( ) )

		double leftPower = strafe.norm( );
		leftPower -= strafe.norm( ) * normalizeAngle( targetHubAngle - leftRotation ); // slower rotates clockwise
		leftPower -= rotate * normalizeAngle( targetRotateAngle - leftRotation );

		double rightPower = strafe.norm( );
		rightPower -= strafe.norm( ) * normalizeAngle( targetHubAngle - rightRotation ); // slower rotates clockwise
		rightPower -= rotate * normalizeAngle( targetRotateAngle - rightRotation );

		// if the target angle = the current angle (of the hubs) then it will just drive

		leftTop.setPower( strafe.norm( ) );
		leftBottom.setPower( leftPower );

		rightTop.setPower( strafe.norm( ) );
		rightBottom.setPower( rightPower );

	}

	@Override
	public void drive( double drive, double strafe ) {

		Vector2d vector = new Vector2d( strafe, drive );

		int leftRotation = getWheelRotationRadians( leftTop, leftBottom );
		int rightRotation = getWheelRotationRadians( rightTop, rightBottom );

		/*
		drive: similar power to everything
			strafe: if wheel too far left, rotate right more
					if wheel too far right, rotate left more
		 */

		// too far left: leftRotation < rotateAngle( vector.angle( ) )
		// too far right: leftRotation > rotateAngle( vector.angle( ) )

		double leftPower = vector.norm( ) - vector.norm( ) * normalizeAngle( rotateAngle( vector ) - leftRotation ); // slower rotates clockwise
		double rightPower = vector.norm( ) - vector.norm( ) * normalizeAngle( rotateAngle( vector ) - rightRotation ); // slower rotates clockwise

		leftTop.setPower( vector.norm( ) );
		leftBottom.setPower( leftPower );

		rightTop.setPower( vector.norm( ) );
		rightBottom.setPower( rightPower );

	}

	@Override
	public void move( double power ) {

		rotateWheelToPos( power, 0, leftTop, leftBottom, true );
		rotateWheelToPos( power, 0, rightTop, rightBottom, true );

		leftTop.setPower( power );
		leftBottom.setPower( power );
		rightTop.setPower( power );
		rightBottom.setPower( power );
	}

	@Override
	public void turn( double power ) {

		int leftRotation = getWheelRotationRadians( leftTop, leftBottom );
		int rightRotation = getWheelRotationRadians( rightTop, rightBottom );

		double targetRotateAngle = 0; // perpendicular to robot's point of rotation

		leftTop.setPower( power );
		leftBottom.setPower( power * normalizeAngle( targetRotateAngle - leftRotation ) );

		rightTop.setPower( power );
		rightBottom.setPower( power * normalizeAngle( targetRotateAngle - rightRotation ) );

	}

	public void moveWheel( double power, double distance, DcMotorEx topMotor, DcMotorEx bottomMotor ) {

		topMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		int target = convertDistTicks( distance, 2 * Math.PI * wheelRadius, rotateGearRadius );
		topMotor.setTargetPosition( target );
		topMotor.setTargetPosition( target );

		topMotor.setPower( power );
		bottomMotor.setPower( -power );

		new Thread( ( ) -> {
			while( topMotor.isBusy( ) || bottomMotor.isBusy( ) ) ;
			topMotor.setPower( 0 );
			bottomMotor.setPower( 0 );
		} ).start( );
	}

	public void driveWheel( double power, double radians, DcMotorEx topMotor, DcMotorEx bottomMotor ) {

		topMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		int target = convertDistTicks( radians, Math.PI * rotateGearRadius * 2, wheelGearRatio );
		topMotor.setTargetPosition( target );
		topMotor.setTargetPosition( target );

		topMotor.setPower( power );
		bottomMotor.setPower( -power );

		new Thread( ( ) -> {
			while( topMotor.isBusy( ) || bottomMotor.isBusy( ) ) ;
			topMotor.setPower( 0 );
			bottomMotor.setPower( 0 );
		} ).start( );
	}

	/**
	 * @param power                the power to rotate the wheel with
	 * @param targetAngle          the distance in radians to rotate
	 * @param topMotor             the motor controlling the wheel hub's upper gear
	 * @param bottomMotor          the motor controlling the wheel hub's lower gear
	 * @param moveClosestDirection true: rotate closer towards the target angle (overrides ±power), false: rotates based on ± of power
	 */
	public void rotateWheelToPos( double power, double targetAngle, DcMotorEx topMotor, DcMotorEx bottomMotor, boolean moveClosestDirection ) {

		// rotation = difference between motors / 2. Distance = Math.max( motors ) - difference
		int distanceRotated = topMotor.getCurrentPosition( ) - bottomMotor.getCurrentPosition( ) / 2;
//		int distanceTravelled = Math.max( topMotor.getCurrentPosition( ), bottomMotor.getCurrentPosition( ) ) - distanceRotated;

		if( moveClosestDirection )
			power = Math.abs( power ) * (distanceRotated % ticksInRotation > ticksInRotation / 2 ? 1 : -1);

		rotateWheel( power, targetAngle, topMotor, bottomMotor );
	}

	public void rotateWheel( double power, double radians, DcMotorEx topMotor, DcMotorEx bottomMotor ) {

		topMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		int target = convertDistTicks( radians, 2 * Math.PI * rotateGearRadius, rotateGearRadius );
		topMotor.setTargetPosition( target );
		topMotor.setTargetPosition( -target );

		topMotor.setPower( power );
		bottomMotor.setPower( -power );

		new Thread( ( ) -> {
			while( topMotor.isBusy( ) || bottomMotor.isBusy( ) ) ;
			topMotor.setPower( 0 );
			bottomMotor.setPower( 0 );
		} ).start( );
	}

	@Override
	public void stop( ) {
		setMotorPower( 0, 0, 0, 0 );
	}

	@Override
	public State getState( ) {
		updateState( );
		return currentState;
	}

	private void updateState( ) {
		if( leftTop.getPower( ) != 0 || rightTop.getPower( ) != 0 || leftBottom.getPower( ) != 0 || rightBottom.getPower( ) != 0 )
			currentState = State.MOVING;
		else
			currentState = State.STOPPED;
	}

	/**
	 * Sets specified power to the motors
	 *
	 * @param leftTopPower     power at which to run the front left motor.
	 * @param leftBottomPower  power at which to run the back left motor.
	 * @param rightTopPower    power at which to run the front right motor.
	 * @param rightBottomPower power at which to run the back right motor.
	 */
	protected void setMotorPower( double leftTopPower, double leftBottomPower, double rightTopPower, double rightBottomPower ) {
		leftTop.setPower( leftTopPower );
		leftBottom.setPower( leftBottomPower );
		rightTop.setPower( rightTopPower );
		rightBottom.setPower( rightBottomPower );
	}

	/**
	 * Sets the direction of the motors
	 *
	 * @param leftTopDirection     direction of the front left motor
	 * @param leftBottomDirection  direction of the back left motor
	 * @param rightTopDirection    direction of the front right motor
	 * @param rightBottomDirection direction of the back right motor
	 */
	public void setMotorDirections( Direction leftTopDirection, Direction leftBottomDirection, Direction rightTopDirection, Direction rightBottomDirection ) {
		leftTop.setDirection( leftTopDirection );
		leftBottom.setDirection( leftBottomDirection );
		rightTop.setDirection( rightTopDirection );
		rightBottom.setDirection( rightBottomDirection );
	}

	/**
	 * Sets the zero power behavior of the motors
	 *
	 * @param leftTopBehavior     zero power behavior of the front left motor
	 * @param leftBottomBehavior  zero power behavior of the back left motor
	 * @param rightTopBehavior    zero power behavior of the front right motor
	 * @param rightBottomBehavior zero power behavior of the back right motor
	 */
	public void setZeroPowerBehavior( ZeroPowerBehavior leftTopBehavior, ZeroPowerBehavior leftBottomBehavior, ZeroPowerBehavior rightTopBehavior, ZeroPowerBehavior rightBottomBehavior ) {
		leftTop.setZeroPowerBehavior( leftTopBehavior );
		leftBottom.setZeroPowerBehavior( leftBottomBehavior );
		rightTop.setZeroPowerBehavior( rightTopBehavior );
		rightBottom.setZeroPowerBehavior( rightBottomBehavior );
	}

	/**
	 * Sets the run modes of the motors
	 *
	 * @param leftTopMode     run mode of the front left motor
	 * @param leftBottomMode  run mode of the back left motor
	 * @param rightTopMode    run mode of the front right motor
	 * @param rightBottomMode run mode of the back right motor
	 */
	public void setRunMode( RunMode leftTopMode, RunMode rightTopMode, RunMode leftBottomMode, RunMode rightBottomMode ) {
		leftTop.setMode( leftTopMode );
		leftBottom.setMode( leftBottomMode );
		rightTop.setMode( rightTopMode );
		rightBottom.setMode( rightBottomMode );
	}
}
