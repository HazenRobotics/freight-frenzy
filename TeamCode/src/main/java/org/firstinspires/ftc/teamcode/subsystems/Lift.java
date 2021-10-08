package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Lift {

	public static final double MAX_VELOCITY = 30;

	DcMotorEx leftMotor;
	DcMotorEx rightMotor;

	double wheelRadius;
	double groundBucketHeight;
	double liftAngle;

	AngleUnit angleUnit;

	final double PULSES_PER_REVOLUTION = 537.7;
	final double GEAR_RATIO = 19.2;

	public Lift( HardwareMap hw ) {
		this( hw, 600, 45, 0.5, AngleUnit.DEGREES,
				"leftLiftMotor", "rightLiftMotor" );
	}

	public Lift( HardwareMap hw, double groundBucketHeight, double liftAngle, double spoolRadius,
				 AngleUnit angleUnit, String leftMotorName, String rightMotorName ) {
		setup( hw, leftMotorName, rightMotorName );
		setLiftAngle( liftAngle );
		setGroundBucketHeight( groundBucketHeight );
		setWheelRadius( spoolRadius );
		setAngleUnit( angleUnit );
	}

	public void setup( HardwareMap hw, String leftMotorName, String rightMotorName ) {

		leftMotor = hw.get( DcMotorEx.class, leftMotorName );
		rightMotor = hw.get( DcMotorEx.class, rightMotorName );

		rightMotor.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	public void setVelocity( double velocity ) {

		leftMotor.setVelocity( velocity, angleUnit );
		rightMotor.setVelocity( velocity, angleUnit );
	}

	private void setTargetPosition( int position ) {
		leftMotor.setTargetPosition( position );
		rightMotor.setTargetPosition( position );
	}

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param position the position to move the lift to in inches
	 */
	public void setLiftPosition( double velocity, double position ) {
		setTargetPosition( convertDistTicks( position, 2 * wheelRadius * Math.PI ) );
		setVelocity( velocity );
		while( isBusy( ) ) ;
		setVelocity( 0 );
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
		setTargetPosition( convertDistTicks( position, 2 * wheelRadius * Math.PI ) );
		setVelocity( velocity );
		new Thread( ( ) -> { // create a new thread so that it doesn't interfere with other mechanisms
			while( isBusy( ) ) ;
			setVelocity( 0 );
		} ).start( );
	}

	public boolean isBusy( ) {
		return leftMotor.isBusy( ) || rightMotor.isBusy( );
	}

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

	// setters and getters for angleUnit
	public void setAngleUnit( AngleUnit angleUnit ) {
		this.angleUnit = angleUnit;
	}

	public AngleUnit getAngleUnit( ) {
		return angleUnit;
	}

	// setters and getters for spoolRadius
	public void setWheelRadius( double newRadius ) {
		wheelRadius = newRadius;
	}

	public double getWheelRadius( ) {
		return wheelRadius;
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
}
