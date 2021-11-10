package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;

public class Lift {

	public static final double MAX_VELOCITY = 30;

	final double PULSES_PER_REVOLUTION = 537.7;
	final double GEAR_RATIO = 1/*9.2*/;

	DcMotorEx motor;

	static int liftPosition = 0;

	double spoolRadius;
	double groundBucketHeight;

	boolean allowLoops = true;

	double liftAngle; // the angle of the lift from the ground angle unit
	AngleUnit angleUnit; // the angle unit for the lift angle i.e. degrees or radians

	public Lift( HardwareMap hw ) {
		this( hw, "lift", 10.25,
				(32 / 25.4) / 2, 45, AngleUnit.DEGREES ); // diameter of 45mm
	}

	public Lift( HardwareMap hw, String motorName, double groundBucketHeight,
				 double spoolRadius, double liftAngle, AngleUnit angleUnit ) {
		setup( hw, motorName, groundBucketHeight, spoolRadius, liftAngle, angleUnit );
	}

	public void setModeTeleOp( ) {
//		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
//		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
//		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
	}

	public void setup( HardwareMap hw, String leftMotorName, double groundBucketHeight,
					   double spoolRadius, double liftAngle, AngleUnit angleUnit ) {

		motor = hw.get( DcMotorEx.class, leftMotorName );

		motor.setDirection( DcMotorSimple.Direction.REVERSE );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		liftPosition = convertDistTicks( groundBucketHeight, 2 * spoolRadius * Math.PI );

		setGroundBucketHeight( groundBucketHeight );
		setSpoolRadius( spoolRadius );
		setLiftAngle( liftAngle );
		setAngleUnit( angleUnit );

	}

	// basic lift setters

	/**
	 * @param power    the power at which to move the lift
	 * @param distance the distance to move the lift in inches
	 */
	public void runDistancePow( double power, double distance ) {

		// reset encoder count kept by the motor.
		stopAndRest( );
		motor.setTargetPosition( convertDistTicks( distance, 2 * spoolRadius * Math.PI ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		setPower( power );

		while( isBusy( ) && allowLoops ) ;

		setPower( 0 );
	}

	/**
	 * creates a new thread that sets the power to zero after the motor has reached its position
	 *
	 * @param power    the power at which to move the lift
	 * @param distance the distance to move the lift in inches
	 */
	public void runDistancePowAsync( double power, double distance ) {

		// reset encoder count kept by the motor.
		stopAndRest( );
		motor.setTargetPosition( convertDistTicks( distance, 2 * spoolRadius * Math.PI ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		setPower( power );

		new Thread( ( ) -> { // create a new thread so that it doesn't interfere with other mechanisms
			while( isBusy( ) && allowLoops ) ;
			setPower( 0 );
		} ).start( );
	}

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param distance the distance to move the lift in inches
	 */
	public void runDistanceVel( double velocity, double distance ) {

		// reset encoder count kept by the motor.
		stopAndRest( );
		motor.setTargetPosition( convertDistTicks( distance, 2 * spoolRadius * Math.PI ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		setVelocity( velocity );

		while( isBusy( ) && allowLoops ) ;

		setVelocity( 0 );
	}

	/**
	 * creates a new thread that sets the velocity to zero after the motor has reached its position
	 *
	 * @param velocity the velocity at which to move the lift
	 * @param distance the distance to move the lift in inches
	 */
	public void runDistanceVelAsync( double velocity, double distance ) {

		// reset encoder count kept by the motor.
		stopAndRest( );
		motor.setTargetPosition( convertDistTicks( distance, 2 * spoolRadius * Math.PI ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		setVelocity( velocity );

		new Thread( ( ) -> { // create a new thread so that it doesn't interfere with other mechanisms
			while( isBusy( ) && allowLoops ) ;
			setVelocity( 0 );
		} ).start( );
	}

	// simple lift setters

	public void setDefaultHeightPow( double power ) {
		setLiftHeightPow( power, groundBucketHeight );
	}

	public void setDefaultHeightVel( double velocity ) {
		setLiftHeightVel( velocity, groundBucketHeight );
		new Thread( ( ) -> {
			while( isBusy( ) && allowLoops ) ;
			motor.setMotorDisable( );
		} ).start( );
	}

	/**
	 * @param power  the power at which to move the lift
	 * @param height the height from the ground to the bottom of the bucket (closed) to move the lift to in inches
	 */
	public void setLiftHeightPow( double power, double height ) {
		if( height - groundBucketHeight < 0 )
			height = groundBucketHeight;
		double distanceToMove = calcLiftDistanceFromHeight( height - convertTicksDist( liftPosition, 2 * spoolRadius * Math.PI ) );
		runDistancePowAsync( power, distanceToMove );
	}

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param height   the height from the ground to the new pos, bottom of the bucket (closed), to move the lift to, in inches
	 */
	public void setLiftHeightVel( double velocity, double height ) {
		if( height - groundBucketHeight < 0 )
			height = groundBucketHeight;
		stopAndRest( );
		double distanceToMove = calcLiftDistanceFromHeight( height - convertTicksDist( liftPosition, 2 * spoolRadius * Math.PI ) );
		if( distanceToMove < 0 )
			velocity *= -1;
		runDistanceVelAsync( velocity, distanceToMove );
	}


	public void exitLoops( long waitTimeMillis ) {
		motor.setPower( 0 );
		allowLoops = false;
		long start = System.currentTimeMillis( );
		while( System.currentTimeMillis( ) < start + waitTimeMillis ) ;
		allowLoops = true;
	}

	public void stopAndRest( ) {

		Log.d( "LOGGER", "motor position: " + motor.getCurrentPosition( ) );
		liftPosition += motor.getCurrentPosition( );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		// stop and reset encoder sets the encoder position to zero
	}

	public void runAfterBusy( Runnable runnable ) {
		while( isBusy( ) && allowLoops ) ;
		runnable.run( );
	}

	// converters and calculators

	/**
	 * @param distance      the distance to move in inches
	 * @param circumference the circumference of the wheel that has the encoder
	 * @return the number of ticks in that distance
	 */
	public int convertDistTicks( double distance, double circumference ) {
		return (int) Math.round( ((distance / circumference) * PULSES_PER_REVOLUTION) / GEAR_RATIO );
	}

	/**
	 * @param ticks         the distance to move in ticks
	 * @param circumference the circumference of the wheel that has the encoder
	 * @return the distance in that number of ticks
	 */
	public double convertTicksDist( double ticks, double circumference ) {
		return (ticks * circumference * GEAR_RATIO) / PULSES_PER_REVOLUTION;
	}

	/*
						/|
			 (lift) c  / |
	  		          /  |  b the height of this side of the triangle
(bottom of bucket)   /___|  h (height given from the ground)
      B (this angle) ^   |
					_____|
 		  	       (ground)

		* given h
		* find c (the distance to set the lift to)
		* g is the distance the bucket is off the ground
		* B is the lift angle (55°)

		sin(θ) = b/c
		b = h-g
		θ = B

		c = (h-g)/(sin(B)
		liftPosition = (height - groundBucketHeight/( sin(liftAngle) )

	 */

	public double calcLiftDistanceFromHeight( double height ) {
		Robot.writeToDefaultFile( "calcLiftDistanceFromHeight: " + (height / Math.sin( getLiftAngle( AngleUnit.RADIANS ) )), true, true );
		return height / Math.sin( getLiftAngle( AngleUnit.RADIANS ) );
	}

	public double calcBucketDistanceFromPosition( double liftPosition ) {
		Robot.writeToDefaultFile( "calcLiftDistanceFromHeight: " + (liftPosition * Math.cos( getLiftAngle( AngleUnit.RADIANS ) )), true, true );
		return liftPosition * Math.cos( getLiftAngle( AngleUnit.RADIANS ) );
	}

	public double calcBucketDistanceFromHeight( double height ) {
		Robot.writeToDefaultFile( "calcLiftDistanceFromHeight: " + (height * Math.tan( getLiftAngle( AngleUnit.RADIANS ) )), true, true );
		return height / Math.tan( getLiftAngle( AngleUnit.RADIANS ) );
	}

	// getters and setters

	public boolean isBusy( ) {
		return motor.isBusy( );
	}

	// getters and setters for power and velocity

	public double getPower( ) {
		return motor.getPower( );
	}

	public void setPower( double power ) {
		motor.setPower( power );
	}

	public void setTeleOPower( double power ) {
		setModeTeleOp( );
		setPower( power );
	}

	public double getVelocity( ) {
		return motor.getVelocity( angleUnit );
	}

	public void setVelocity( double velocity ) {
		motor.setVelocity( velocity, angleUnit );
	}

	// getters for the lift position

	public static int getPosition( boolean statics ) {
		return liftPosition;
	}

	public int getPosition( ) {
		return liftPosition + motor.getCurrentPosition( );
	}

	public double getPositionInch( ) {
		return convertTicksDist( getPosition( ), 2 * spoolRadius * Math.PI );
	}

	public double getMotorPositionInch( ) {
		return convertTicksDist( motor.getCurrentPosition( ), 2 * spoolRadius * Math.PI );
	}

	public double getBucketDistance( ) {
		return calcBucketDistanceFromPosition( getPosition( ) );
	}

	// setters and getters for angleUnit

	public void setAngleUnit( AngleUnit angleUnit ) {
		this.angleUnit = angleUnit;
	}

	public AngleUnit getAngleUnit( ) {
		return angleUnit;
	}

	// setters and getters for spoolRadius

	public double getSpoolRadius( ) {
		return spoolRadius;
	}

	public void setSpoolRadius( double newRadius ) {
		spoolRadius = newRadius;
	}

	// setters and getters for groundBucketHeight
	public void setGroundBucketHeight( double height ) {
		groundBucketHeight = height;
	}

	public double getGroundBucketHeight( ) {
		return groundBucketHeight;
	}

	// setters and getters for liftAngle
	public void setLiftAngle( double newAngle ) {
		liftAngle = newAngle;
	}

	public double getLiftAngle( ) {
		return liftAngle;
	}

	public double getLiftAngle( AngleUnit angle ) {

		if( angle.equals( angleUnit ) )
			return liftAngle;
		else if( angle.equals( AngleUnit.RADIANS ) )
			return Math.toRadians( liftAngle );
		else if( angle.equals( AngleUnit.DEGREES ) )
			return Math.toDegrees( liftAngle );
		return liftAngle;
	}
}
