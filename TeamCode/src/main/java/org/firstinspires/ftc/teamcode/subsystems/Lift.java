package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;

public class Lift {

	final double PULSES_PER_REVOLUTION = 537.7;
	final double GEAR_RATIO = 1;

	DcMotorEx motor;

	static int liftPosition = 0;

	double spoolRadius;
	double groundBucketHeight;

	public static final double LIFT_SWITCH_LIMIT = 0.75;

	boolean allowLoops = true;

	double liftAngle; // the angle of the lift from the ground in 'angleUnit's
	AngleUnit angleUnit; // the angle unit for the lift angle i.e. degrees or radians

	/**
	 * Creates the default Lift with:
	 * -a motorName of "lift",
	 * -a groundBucketHeight of 3.25,
	 * -a spoolRadius of 0.63",
	 * -a liftAngle of 55°, and
	 * -an AngleUnit of DEGREES
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public Lift( HardwareMap hardwareMap ) {
		setup( hardwareMap, "lift", 2.375,
				(38.2 / 25.4) / 2, 55, AngleUnit.DEGREES ); // diameter of 45mm
	}

	/**
	 * @param hardwareMap        the hardwareMap of the current running OpMode
	 * @param motorName          the name of the lift motor in the hardware map
	 * @param groundBucketHeight the height of the bottom of the lift to the ground
	 * @param spoolRadius        the radius of the spool attached om 'angleUnit's
	 * @param liftAngle          the angle of the lift from the ground in
	 * @param angleUnit          the angle unit to make calculations and input variables
	 */
	public Lift( HardwareMap hardwareMap, String motorName, double groundBucketHeight,
				 double spoolRadius, double liftAngle, AngleUnit angleUnit ) {
		setup( hardwareMap, motorName, groundBucketHeight, spoolRadius, liftAngle, angleUnit );
	}

	public void setup( HardwareMap hardwareMap, String leftMotorName, double groundBucketHeight,
					   double spoolRadius, double liftAngle, AngleUnit angleUnit ) {

		motor = hardwareMap.get( DcMotorEx.class, leftMotorName );

		motor.setDirection( DcMotorSimple.Direction.REVERSE );
		resetLift( );

		setGroundBucketHeight( groundBucketHeight );
		setSpoolRadius( spoolRadius );
		setLiftAngle( liftAngle );
		setAngleUnit( angleUnit );
	}

	/**
	 * stops and resets the physical motor and its encoder and sets liftPosition to 0
	 */
	public void resetLift( ) {
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		liftPosition = 0;
	}

	// basic lift setters

	/**
	 * runs the lift at a set velocity for a certain distance, then waits for the motor to get there asynchronously if stopAsync is true
	 *
	 * @param power     the power at which to move the lift
	 * @param distance  the distance to move the lift in inches
	 * @param stopAsync wait for the robot to finish asynchronously or not
	 */
	public void moveDistancePower( double power, double distance, boolean stopAsync ) {

		// reset encoder count kept by the motor.
		stopAndReset( );
		if( distance < 0 )
			power *= -1;
		motor.setTargetPosition( convertDistTicks( distance, 2 * spoolRadius * Math.PI ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		setPower( power );

		if( stopAsync ) {
			// create a new thread so that it doesn't interfere with other mechanisms
			new Thread( ( ) -> {
				waitForMoveFinish( );
				setPower( 0 );
				disableMotorIfUnused( );
			} ).start( );
		} else {
			waitForMoveFinish( );
			setPower( 0 );
			disableMotorIfUnused( );
		}
	}

	/**
	 * runs the lift at a set velocity for a certain distance, then waits for the motor to get there asynchronously if stopAsync is true
	 *
	 * @param velocity  the velocity at which to move the lift
	 * @param distance  the distance to move the lift in inches
	 * @param stopAsync wait for the robot to finish asynchronously or not
	 */
	public void moveDistanceVelocity( double velocity, double distance, boolean stopAsync ) {

		// reset encoder count kept by the motor.
		stopAndReset( );

		motor.setTargetPosition( convertDistTicks( distance, 2 * spoolRadius * Math.PI ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		if( distance < 0 ) velocity *= -1;
		setVelocity( velocity );

		if( stopAsync ) {
			// create a new thread so that it doesn't interfere with other mechanisms
			new Thread( ( ) -> {
				waitForMoveFinish( );
				setVelocity( 0 );
				disableMotorIfUnused( );
			} ).start( );
		} else {
			waitForMoveFinish( );
			setVelocity( 0 );
			disableMotorIfUnused( );
		}
	}

	public void waitForMoveFinish( ) {
		while( isBusy( ) && motor.getCurrent( CurrentUnit.AMPS ) < 7.9 ) ;
	}

	// simple lift setters

	public void setDefaultHeightPow( double power ) {
		setHeightPower( power, groundBucketHeight );
		new Thread( ( ) -> {
			waitForMoveFinish( );
			disableMotorIfUnused( );
		} ).start( );
	}

	public void setDefaultHeightVel( double velocity ) {
		setHeightVelocity( velocity, groundBucketHeight );
		new Thread( ( ) -> {
			waitForMoveFinish( );
			disableMotorIfUnused( );
		} ).start( );
	}

	/**
	 * @param power  the power at which to move the lift
	 * @param height the height from the ground to the bottom of the bucket (in the intake position) to move the lift to in inches
	 */
	public void setHeightPower( double power, double height ) {
		if( height - groundBucketHeight < 0 )
			height = groundBucketHeight;
		double distanceToMove = calcLiftDistanceFromHeight( height ) - convertTicksDist( liftPosition, 2 * spoolRadius * Math.PI );
		moveDistancePower( power, distanceToMove, true );
	}

	/**
	 * @param velocity the velocity at which to move the lift
	 * @param height   the height from the ground to the new pos, bottom of the bucket (in the intake position), to move the lift to, in inches
	 */
	public void setHeightVelocity( double velocity, double height ) {
		if( height < groundBucketHeight )
			height = groundBucketHeight;

		stopAndReset( );
		double distanceToMove = calcLiftDistanceFromHeight( height - groundBucketHeight ) - convertTicksDist( liftPosition, 2 * spoolRadius * Math.PI );
		moveDistanceVelocity( velocity, distanceToMove, true );
	}

	// util methods

	/**
	 * if the motor is below LIFT_SWITCH_LIMIT it will disable it to conserve the motor
	 */
	public void disableMotorIfUnused( ) {
		if( getPositionInch( ) <= LIFT_SWITCH_LIMIT )
			motor.setMotorDisable( );
	}

	/**
	 * exits/disables all while loops that the lift may be stuck in then enables them after waitTimeMillis
	 *
	 * @param waitTimeMillis the time to wait before allowing loops again (in milliseconds)
	 */
	public void exitLoops( long waitTimeMillis ) {
		motor.setPower( 0 );
		allowLoops = false;
		long start = System.currentTimeMillis( );
		new Thread( ( ) -> {
			while( System.currentTimeMillis( ) < start + waitTimeMillis ) ;
			allowLoops = true;
		} ).start( );
	}

	/**
	 * adds the current motor position to liftPosition then stops and resets the encoder
	 */
	public void stopAndReset( ) {

		Log.d( "LOGGER", "motor position: " + motor.getCurrentPosition( ) );
		liftPosition += motor.getCurrentPosition( );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		// stop and reset encoder sets the encoder position to zero
	}

	public void runAfterBusy( Runnable runnable ) {
		waitForMoveFinish( );
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
		Robot.writeToDefaultFile( "calcBucketDistanceFromPosition: " + (liftPosition * Math.cos( getLiftAngle( AngleUnit.RADIANS ) )), true, true );
		return liftPosition * Math.cos( getLiftAngle( AngleUnit.RADIANS ) );
	}

	public double calcBucketDistanceFromHeight( double height ) {
		Robot.writeToDefaultFile( "calcBucketDistanceFromHeight: " + (height * Math.tan( getLiftAngle( AngleUnit.RADIANS ) )), true, true );
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
		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		setPower( power );
	}

	public double getVelocity( ) {
		return motor.getVelocity( angleUnit );
	}

	public void setVelocity( double velocity ) {
		motor.setVelocity( velocity, angleUnit );
	}

	// getters for the lift position

	public int getTargetPosition( ) {
		return motor.getTargetPosition( );
	}

	public double getTargetPositionInch( ) {
		return convertTicksDist( motor.getTargetPosition( ), 2 * spoolRadius * Math.PI );
	}

	public static int getPosition( boolean statics ) {
		return liftPosition;
	}

	public int getPosition( ) {
		return liftPosition + motor.getCurrentPosition( );
	}

	public double getPositionInch( ) {
		return convertTicksDist( getPosition( ), 2 * spoolRadius * Math.PI );
	}

	public int getMotorPosition( ) {
		return motor.getCurrentPosition( );
	}

	public double getMotorPositionInch( ) {
		return convertTicksDist( getMotorPosition( ), 2 * spoolRadius * Math.PI );
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

	/**
	 * @param angle the angle unit to get the lift's angle in
	 * @return the angle of the lift in the angle unit requested
	 */
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
