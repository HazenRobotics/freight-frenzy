package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

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
		this( hw, "liftLeft", 5,
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

	private void setTargetPosition( int position ) {
		motor.setTargetPosition( position );
		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
	}

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param position the position to move the lift to in inches
	 */
	public void setLiftPosition( double velocity, double position ) {
		setTargetPosition( convertTicksDist( position, 2 * spoolRadius * Math.PI ) );
		setVelocity( velocity );
		while( isBusy( ) ) ;
		setVelocity( 0 );
	}

	public void moveLift( double power, double distanceToTravel ) {

		// reset encoder count kept by left motor.
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		motor.setTargetPosition( convertDistTicks( distanceToTravel, 2 * spoolRadius * Math.PI ) );

		motor.setPower( power );

		Log.e( "GOAL:", "power: " + power + ", ticks: " + convertDistTicks( distanceToTravel, 2 * spoolRadius * Math.PI ) );
		Log.e( "ACTUAL:", "power: " + motor.getPower( ) + ", ticks: " + motor.getTargetPosition( ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		// wait while opmode is active and left motor is busy running to position.
		while( /*opModeIsActive( ) &&*/ motor.isBusy( ) ) {
//			telemetry.addData( "encoder-fwd", motor.getCurrentPosition( ) + "  busy=" + motor.isBusy( ) );
//			telemetry.update( );
//			idle( );
		}

		// set motor power to zero to turn off motors. The motors stop on their own but
		// power is still applied so we turn off the power.
		motor.setPower( 0.0 );
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
	public void setLiftHeight( double velocity, double height ) {
		double liftPosition = (height - groundBucketHeight) / (Math.sin( Math.toRadians( liftAngle ) ));
		setLiftPosition( velocity, liftPosition );
	}

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param position the position to move the lift to in inches
	 */
	public void setPositionAsync( double velocity, double position ) {
		setTargetPosition( convertTicksDist( position, 2 * spoolRadius * Math.PI ) );
		setVelocity( velocity );
		new Thread( ( ) -> { // create a new thread so that it doesn't interfere with other mechanisms
			while( isBusy( ) ) ;
			setVelocity( 0 );
		} ).start( );
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
