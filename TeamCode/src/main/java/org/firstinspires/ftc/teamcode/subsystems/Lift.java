package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Lift {

	public static final double MAX_VELOCITY = 30;

	final double PULSES_PER_REVOLUTION = 537.7;
	final double GEAR_RATIO = 1/*9.2*/;

	DcMotorEx motor;

	double spoolRadius;
	double groundBucketHeight;

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

	public void setup( HardwareMap hw, String leftMotorName, double groundBucketHeight,
					   double spoolRadius, double liftAngle, AngleUnit angleUnit ) {

		motor = hw.get( DcMotorEx.class, leftMotorName );

		motor.setDirection( DcMotorSimple.Direction.REVERSE );

		setGroundBucketHeight( groundBucketHeight );
		setSpoolRadius( spoolRadius );
		setLiftAngle( liftAngle );
		setAngleUnit( angleUnit );
	}

	public void setVelocity( double velocity ) {

		motor.setVelocity( velocity, angleUnit );
	}

	public int getCurrentPosition( ) {
		return motor.getCurrentPosition( );
	}

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param position the position to move the lift to in inches
	 */
	public void runToPositionVel( double velocity, double position ) {

		// reset encoder count kept by the motor.
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		motor.setTargetPosition( convertDistTicks( position, 2 * spoolRadius * Math.PI ) );

		setVelocity( velocity );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		while( isBusy( ) ) ;

		setVelocity( 0 );
	}

	/**
	 * @param power    the power at which to move the lift
	 * @param position the position to move the lift to in inches
	 */
	public void runToPositionPow( double power, double position ) {

		// reset encoder count kept by the motor.
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		motor.setTargetPosition( convertDistTicks( position, 2 * spoolRadius * Math.PI ) );

		setPower( power );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		while( isBusy( ) ) ;

		setPower( 0 );
	}

	/**
	 * @param power the power at which to move the lift
	 * @param position the position to move the lift to in inches
	 */
	public void setPositionAsync( double power, double position ) {

		// reset encoder count kept by the motor.
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		motor.setTargetPosition( convertDistTicks( position, 2 * spoolRadius * Math.PI ) );

		setPower( power );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		new Thread( ( ) -> { // create a new thread so that it doesn't interfere with other mechanisms
			while( isBusy( ) ) ;
			setPower( 0 );
		} ).start( );
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
		* B is the lift angle (either 40 or 45°)

		sin(θ) = b/c
		b = h-g
		θ = B

		c = (h-g)/(sin(B)
		liftPosition = (height - groundBucketHeight/( sin(liftAngle) )

	 */

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param height   the height from the ground to the bottom of the bucket (closed) to move the lift to in inches
	 */
	public void setLiftHeightVel( double velocity, double height ) {
		if( height - groundBucketHeight < 0 )
			return;
		double liftPosition = (height - groundBucketHeight) / (Math.sin( Math.toRadians( liftAngle ) ));
		runToPositionVel( velocity, liftPosition );
	}

	/**
	 * @param power  the power at which to move the lift
	 * @param height the height from the ground to the bottom of the bucket (closed) to move the lift to in inches
	 */
	public void setLiftHeightPow( double power, double height ) {
		if( height - groundBucketHeight < 0 )
			return;
		double liftPosition = (height - groundBucketHeight) / (Math.sin( Math.toRadians( liftAngle ) ));
		runToPositionPow( power, liftPosition );
	}

	public boolean isBusy( ) {
		return motor.isBusy( );
	}

	/**
	 * @param distanceToTravel the distance to move in inches
	 * @param circumference    the circumference of the wheel that has the encoder
	 * @return totalTicks - the amount of ticks to move forward
	 */
	public int convertDistTicks( double distanceToTravel, double circumference ) {
		return (int) Math.round( ((distanceToTravel / circumference) * PULSES_PER_REVOLUTION) / GEAR_RATIO );
	}

	public int convertTicksDist( double ticksToTravel, double circumference ) {
		return (int) Math.round( (ticksToTravel * circumference * GEAR_RATIO) / PULSES_PER_REVOLUTION );
	}

	// setters and getters for angleUnit
	public void setAngleUnit( AngleUnit angleUnit ) {
		this.angleUnit = angleUnit;
	}

	public AngleUnit getAngleUnit( ) {
		return angleUnit;
	}

	// setters and getters for spoolRadius
	public void setSpoolRadius( double newRadius ) {
		spoolRadius = newRadius;
	}

	// setters and getters for groundBucketHeight
	public void setGroundBucketHeight( double newBucketHeight ) {
		groundBucketHeight = newBucketHeight;
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

	public void setPower( double power ) {
		motor.setPower( power );
	}
}
