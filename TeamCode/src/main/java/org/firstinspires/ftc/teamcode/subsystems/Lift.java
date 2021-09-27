package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Lift implements Subsystem {

	DcMotorEx rightMotor;
	DcMotorEx leftMotor;

	double wheelRadius;

	AngleUnit angleUnit;

	Thread liftIsBusy = new Thread( ( ) -> {

	});

	final double PULSES_PER_REVOLUTION = 537.7;
	final double GEAR_RATIO = 19.2;

	public Lift( HardwareMap hw ) {
		this( hw, 0.5, AngleUnit.DEGREES );
	}

	public Lift( HardwareMap hw, double spoolRadius, AngleUnit angleUnit ) {
		setup( hw );
		setWheelRadius( spoolRadius );
		setAngleUnit( angleUnit );
	}

	public void setup( HardwareMap hw ) {

		leftMotor = hardwareMap.get( DcMotorEx.class, "leftMotor" );
		rightMotor = hardwareMap.get( DcMotorEx.class, "rightMotor" );

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
	public void setPosition( double velocity, double position ) {
		setTargetPosition( convertDistTicks( position, 2 * wheelRadius * Math.PI ) );
		setVelocity( velocity );
		while( isBusy( ) ) ;
		setVelocity( 0 );
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
}
