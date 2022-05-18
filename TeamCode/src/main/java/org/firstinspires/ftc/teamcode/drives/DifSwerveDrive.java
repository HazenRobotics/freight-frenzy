package org.firstinspires.ftc.teamcode.drives;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DifSwerveDrive implements Drive {

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

	public double maxAngularVelocity;
	public double storedMaxAngularPow;
	public double storedMaxAngularVel;

	public double wheelBase;

	int ticksInRotation = 250;

	final double TWO_PI = 2 * Math.PI;

	double MAX_VELOCITY = 5 * TWO_PI; // Radians per seconds

	public DifSwerveDrive( HardwareMap hardwareMap ) {
		this( hardwareMap, "leftTop", "leftBottom", "rightTop", "rightBottom" );
	}

	public DifSwerveDrive( HardwareMap hardwareMap, String leftTopName, String rightTopName, String leftBottomName, String rightBottomName ) {
		setUpMotors( hardwareMap, leftTopName, rightTopName, leftBottomName, rightBottomName );
	}

	@Override
	public int convertDistTicks( double distanceToTravel, double circumference, double gearRatio, double ppr ) {
		return convertDistTicks( distanceToTravel, circumference, gearRatio, ppr );
	}

	@Override
	public double convertTicksDist( double ticksToTravel, double circumference, double gearRatio, double ppr ) {
		return convertTicksDist( ticksToTravel, circumference, gearRatio, ppr );
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

		setMotorDirections( FORWARD, FORWARD, FORWARD, FORWARD );
		setZeroPowerBehavior( BRAKE, BRAKE, BRAKE, BRAKE );
		//setRunMode(STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER );
	}

	public void setUpWheelRatios( double wheelRadius, double wheelGearRatio, double rotateGearRadius, double wheelGearRadius ) {

		this.wheelRadius = wheelRadius;
		this.wheelGearRatio = wheelGearRatio;

		this.rotateGearRadius = rotateGearRadius;
		this.rotateGearRatio = wheelGearRadius;

		ticksInRotation = convertDistTicks( TWO_PI, TWO_PI * rotateGearRadius, rotateGearRatio, PULSES_PER_REVOLUTION );

		storedMaxAngularPow = 2 / wheelBase;
		storedMaxAngularVel = 2 * maxAngularVelocity / wheelBase;
	}

	/**
	 * @param maxAngularVelocity the new max rotation speed of a wheel pod in rad / s
	 */
	public void setMaxAngularVelocity( double maxAngularVelocity ) {
		this.maxAngularVelocity = maxAngularVelocity;
		storedMaxAngularPow = 2 / wheelBase;
		storedMaxAngularVel = 2 * maxAngularVelocity / wheelBase;
	}

	public void setWheelBase( double wheelBase ) {
		this.wheelBase = wheelBase;
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
	 * @param angleUnit   the angle unit to measure the wheel's rotation
	 * @return the number of angleUnit the motor hub has rotated
	 */
	public double getWheelRotation( DcMotorEx topMotor, DcMotorEx bottomMotor, AngleUnit angleUnit ) {
		double radians = convertTicksDist( getWheelRotation( topMotor, bottomMotor ), TWO_PI * rotateGearRadius, rotateGearRatio, PULSES_PER_REVOLUTION );
		return (angleUnit == AngleUnit.RADIANS) ? radians : Math.toDegrees( radians );
	}

	/**
	 * @param angle the input angle between 0 and 2π
	 * @return the angle normalized between 0 and 1
	 */
	public double normalizeAngle( double angle ) {
		return normalize( angle, 0, TWO_PI, 0, 1 );
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
	public double normalizeVectorAngle( double vectorAngle ) {
		return Angle.norm( vectorAngle + Math.PI / 2 );
	}

	/**
	 * @param vector the vector to get an angle from
	 * @return the angle rotated 90° counterclockwise
	 */
	public double normalizeVectorAngle( Vector2d vector ) {
		return normalizeVectorAngle( vector.angle( ) );
	}

	// drive logic
	/*
	drive: similar power to everything
		strafe: if wheel too far left, turn hub right more
				if wheel too far right, turn hub left more
	if rotate right:
		move wheels toward 0 (straight of the robot) and set wheel powers opposite each other
			if wheel too far left, turn hub right more
			if wheel too far right, turn hub left more
	 */

	public void move( double drive, double strafe, double rotate ) {
		move( new Vector2d( strafe, drive ), rotate );
	}
	public void move( Vector2d strafe, double rotate ) {

		double leftRotation = getWheelRotation( leftTop, leftBottom, AngleUnit.RADIANS );
		double rightRotation = getWheelRotation( rightTop, rightBottom, AngleUnit.RADIANS );

		double targetHubAngle = normalizeVectorAngle( strafe );
//		double targetRotateAngle = 0; // perpendicular to robot's point of rotation

		double normPower = strafe.norm( ) * MAX_VELOCITY;
		double angularPower = rotate * storedMaxAngularPow;

		double leftDrivePow = normPower + angularPower;
		double rightDrivePow = normPower - angularPower;

		double leftStrafePow = normPower * normalizeAngle( targetHubAngle - leftRotation );
		double rightStrafePow = normPower * normalizeAngle( targetHubAngle - rightRotation );

		double leftRotatePow = angularPower * normalizeAngle( /*targetRotateAngle*/ -leftRotation );
		double rightRotatePow = angularPower * normalizeAngle( /*targetRotateAngle*/ -rightRotation );

		// if the target angle = the current angle (of the hubs) then it will just drive

		leftTop.setPower( leftDrivePow + leftStrafePow + leftRotatePow );
		leftBottom.setPower( leftDrivePow - leftStrafePow - leftRotatePow );

		rightTop.setPower( rightDrivePow + rightStrafePow + rightRotatePow );
		rightBottom.setPower( rightDrivePow - rightStrafePow - rightRotatePow );
	}

	public void moveVelocity( Vector2d strafe, double rotate ) {

		double leftRotation = getWheelRotation( leftTop, leftBottom, AngleUnit.RADIANS );
		double rightRotation = getWheelRotation( rightTop, rightBottom, AngleUnit.RADIANS );

		double targetHubAngle = normalizeVectorAngle( strafe );
//		double targetRotateAngle = 0; // perpendicular to robot's point of rotation

		double normVelocity = strafe.norm( ) * MAX_VELOCITY;
		double angularVelocity = rotate * storedMaxAngularVel;

		double leftDriveVel = normVelocity + angularVelocity;
		double rightDriveVel = normVelocity - angularVelocity;

		double leftStrafeVel = normVelocity * normalizeAngle( targetHubAngle - leftRotation );
		double rightStrafeVel = normVelocity * normalizeAngle( targetHubAngle - rightRotation );

		double leftRotateVel = angularVelocity * normalizeAngle( /* targetRotateAngle */ -leftRotation );
		double rightRotateVel = angularVelocity * normalizeAngle( /* targetRotateAngle */ -rightRotation );

		// if the target angle = the current angle (of the hubs) then it will just drive

		leftTop.setVelocity( leftDriveVel + leftStrafeVel + leftRotateVel, AngleUnit.RADIANS );
		leftBottom.setVelocity( leftDriveVel - leftStrafeVel - leftRotateVel, AngleUnit.RADIANS );

		rightTop.setVelocity( rightDriveVel + rightStrafeVel + rightRotateVel, AngleUnit.RADIANS );
		rightBottom.setVelocity( rightDriveVel - rightStrafeVel - rightRotateVel, AngleUnit.RADIANS );

	}

	@Override
	public void drive( double drive, double strafe ) {

		drive( new Vector2d( strafe, drive ) );
	}

	public void drive( Vector2d strafe ) {

		double targetHubAngle = normalizeVectorAngle( strafe );
		double power = strafe.norm( );

		double leftStrafePow = normalizeAngle( targetHubAngle - getWheelRotation( leftTop, leftBottom, AngleUnit.RADIANS ) );
		double rightStrafePow = normalizeAngle( targetHubAngle - getWheelRotation( rightTop, rightBottom, AngleUnit.RADIANS ) );

		// if the target angle = the current angle (of the hubs) then it will just drive

		leftTop.setPower( power * (1 + leftStrafePow) ); // norm + norm * strafe
		leftBottom.setPower( power * (1 - leftStrafePow) ); // norm - norm * strafe

		rightTop.setPower( power * (1 + rightStrafePow) ); // norm + norm * strafe
		rightBottom.setPower( power * (1 - rightStrafePow) ); // norm - norm * strafe
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

		double leftRotation = getWheelRotation( leftTop, leftBottom, AngleUnit.RADIANS );
		double rightRotation = getWheelRotation( rightTop, rightBottom, AngleUnit.RADIANS );

		double targetRotateAngle = 0; // perpendicular to robot's point of rotation

		leftTop.setPower( power );
		leftBottom.setPower( power * normalizeAngle( targetRotateAngle - leftRotation ) );

		rightTop.setPower( power );
		rightBottom.setPower( power * normalizeAngle( targetRotateAngle - rightRotation ) );

	}

	public void moveWheel( double power, double distance, DcMotorEx topMotor, DcMotorEx bottomMotor ) {

		topMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		int target = convertDistTicks( distance, TWO_PI * wheelRadius, rotateGearRadius, PULSES_PER_REVOLUTION );
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

		int target = convertDistTicks( radians, TWO_PI * rotateGearRadius, wheelGearRatio, PULSES_PER_REVOLUTION );
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

		int distanceRotated = getWheelRotation( topMotor, bottomMotor );
//		int distanceTravelled = Math.max( topMotor.getCurrentPosition( ), bottomMotor.getCurrentPosition( ) ) - distanceRotated;

		if( moveClosestDirection )
			power = Math.abs( power ) * (distanceRotated % ticksInRotation > ticksInRotation / 2 ? 1 : -1);

		rotateWheel( power, targetAngle, topMotor, bottomMotor );
	}

	public void rotateWheel( double power, double radians, DcMotorEx topMotor, DcMotorEx bottomMotor ) {

		topMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		int target = convertDistTicks( radians, TWO_PI * rotateGearRadius, rotateGearRadius, PULSES_PER_REVOLUTION );
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
