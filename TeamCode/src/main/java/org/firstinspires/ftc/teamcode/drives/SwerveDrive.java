package org.firstinspires.ftc.teamcode.drives;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveDrive implements Drive {

	public DcMotorEx leftTop;
	public DcMotorEx leftBottom;
	public DcMotorEx rightTop;
	public DcMotorEx rightBottom;

	final double PULSES_PER_REVOLUTION = 250; // 537.7

	private State currentState = State.STOPPED;

	public double wheelGearRatio;
	public double rotateGearRatio;
	public double rotateGearRadius;

	public SwerveDrive( HardwareMap hardwareMap ) {
		setUpMotors( hardwareMap, "leftTop", "leftBottom", "rightTop", "rightBottom" );
	}

	public SwerveDrive( HardwareMap hardwareMap, double wheelGearRatio, double rotateGearRatio, double rotateGearRadius ) {
		this( hardwareMap, "leftTop", "leftBottom", "rightTop", "rightBottom", wheelGearRatio, rotateGearRatio, rotateGearRadius );
	}

	public SwerveDrive( HardwareMap hardwareMap, String leftTopName, String rightTopName, String leftBottomName, String rightBottomName ) {
		this( hardwareMap, leftTopName, rightTopName, leftBottomName, rightBottomName, 1, 1, 3.2655 );
	}

	public SwerveDrive( HardwareMap hardwareMap, String leftTopName, String rightTopName, String leftBottomName, String rightBottomName, double wheelGearRatio, double rotateGearRatio, double rotateGearRadius ) {
		setUpMotors( hardwareMap, leftTopName, rightTopName, leftBottomName, rightBottomName );
		setUpGearRatios( wheelGearRatio, rotateGearRatio, rotateGearRadius );
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
	 * @param rightTopName    name of front right motor in the hardware map
	 * @param leftTopName     name of front left motor in the hardware map
	 * @param rightBottomName name of back right motor in the hardware map
	 * @param leftBottomName  name of back left motor in the hardware map
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

	private void setUpGearRatios( double wheelGearRatio, double wheelGearRadius, double rotateGearRadius ) {

		this.wheelGearRatio = wheelGearRatio;
		this.rotateGearRatio = wheelGearRadius;
		this.rotateGearRadius = rotateGearRadius;
	}


	@Override
	public void drive( double move, double turn ) {

		// You might have to play with the + or - depending on how your motors are installed
		double leftTopPower = move + turn;
		double leftBottomPower = move + turn;
		double rightTopPower = move - turn;
		double rightBottomPower = move - turn;

		setMotorPower( leftTopPower, leftBottomPower, rightTopPower, rightBottomPower );
	}

	@Override
	public void move( double power ) {
		leftTop.setPower( power );
		leftBottom.setPower( power );
		rightTop.setPower( power );
		rightBottom.setPower( power );
	}

	@Override
	public void turn( double power ) {
		leftTop.setPower( power );
		leftBottom.setPower( -power );
		rightTop.setPower( power );
		rightBottom.setPower( -power );
	}

	public void moveWheel( double power, double radians, DcMotorEx topMotor, DcMotorEx bottomMotor ) {

		topMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		int target = convertDistTicks( radians, Math.PI * rotateGearRadius * 2, rotateGearRadius );
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

	public void rotateWheelToPos( double power, DcMotorEx topMotor, DcMotorEx bottomMotor ) {
		int distanceRotated = topMotor.getCurrentPosition( ) - bottomMotor.getCurrentPosition( ) / 2;
		int distanceTravelled = Math.max( topMotor.getCurrentPosition( ), bottomMotor.getCurrentPosition( ) ) - distanceRotated;

//		rotateWheel();

		// rotation = difference between motors / 2. Distance = Math.max( motors ) - difference
	}

	public void rotateWheel( double power, double radians, DcMotorEx topMotor, DcMotorEx bottomMotor ) {

		topMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		int target = convertDistTicks( radians, Math.PI * rotateGearRadius * 2, rotateGearRadius );
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

	public double getLeftTopPower( ) {
		return leftTop.getPower( );
	}

	public double getLeftBottomPower( ) {
		return leftBottom.getPower( );
	}

	public double getRightTopPower( ) {
		return rightTop.getPower( );
	}

	public double getRightBottomPower( ) {
		return rightBottom.getPower( );
	}


	public double getLeftTopVelocity( ) {
		return leftTop.getVelocity( );
	}

	public double getLeftBottomVelocity( ) {
		return leftBottom.getVelocity( );
	}

	public double getRightTopVelocity( ) {
		return rightTop.getVelocity( );
	}

	public double getRightBottomVelocity( ) {
		return rightBottom.getVelocity( );
	}


	public double getLeftTopVelocity( AngleUnit angleUnit ) {
		return leftTop.getVelocity( angleUnit );
	}

	public double getLeftBottomVelocity( AngleUnit angleUnit ) {
		return leftBottom.getVelocity( angleUnit );
	}

	public double getRightTopVelocity( AngleUnit angleUnit ) {
		return rightTop.getVelocity( angleUnit );
	}

	public double getRightBottomVelocity( AngleUnit angleUnit ) {
		return rightBottom.getVelocity( angleUnit );
	}


	public int getLeftTopPosition( ) {
		return leftTop.getCurrentPosition( );
	}

	public int getLeftBottomPosition( ) {
		return leftBottom.getCurrentPosition( );
	}

	public int getRightTopPosition( ) {
		return rightTop.getCurrentPosition( );
	}

	public int getRightBottomPosition( ) {
		return rightBottom.getCurrentPosition( );
	}
}
