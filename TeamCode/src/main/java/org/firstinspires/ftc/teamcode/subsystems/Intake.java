package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

	DcMotorEx intakeMotor;

	public static final double CURRENT_DRAW_MINIMUM = 1.0;

	private int intakenBlocks = 0;

	/**
	 * creates a default noodle intake with a intakeName of "intake"
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public Intake( HardwareMap hardwareMap ) {
		setup( hardwareMap, "intake" );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param intakeName  the name of the intake motor in the hardware map
	 */
	public Intake( HardwareMap hardwareMap, String intakeName ) {
		setup( hardwareMap, intakeName );
	}

	public void setup( HardwareMap hardwareMap, String intakeName ) {

		intakeMotor = hardwareMap.get( DcMotorEx.class, intakeName );

		intakeMotor.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	// getters and setters

	public double getPower( ) {
		return intakeMotor.getPower( );
	}

	public void setPower( double power ) {

		intakeMotor.setPower( power );
		if( power > 0 )
			new Thread( this::startIntakeChecking ).start( );
	}

	/**
	 * @return the current of the intake motor in amps
	 */
	public double getCurrent( ) {
		return getCurrent( CurrentUnit.AMPS );
	}

	public double getCurrent( CurrentUnit currentUnit ) {
		return intakeMotor.getCurrent( currentUnit );
	}

	public boolean intaking( ) {
		return getCurrent( ) > CURRENT_DRAW_MINIMUM;
	}

	public int getIntakenBlocks( ) {
		return intakenBlocks;
	}

	/**
	 * @param power      to set the motor
	 * @param maxIntaken max number of blocks to intake
	 * @param maxTime    max time to run the intake
	 */
	public void intakeBlocks( double power, int maxIntaken, int maxTime ) {
		setPower( power );
		intakeBlocks( maxIntaken, maxTime );
	}

	/**
	 * @param maxIntaken max number of blocks to intake
	 * @param maxTime    max time to run the intake
	 */
	public void intakeBlocks( int maxIntaken, int maxTime ) {

		new Thread( ( ) -> {

			int intakeTimeLimit = 10 * 1000; // 10 seconds
			double startTime = System.currentTimeMillis( );
			while( getPower( ) > 0 && startTime + intakeTimeLimit > System.currentTimeMillis( ) ) { // while the power is up, and it has been less than 10 seconds
				if( intakenBlocks >= maxIntaken )
					break;
				try {
					Thread.sleep( 50 );
				} catch( InterruptedException ignored ) {
				}
			}
			try {
				Thread.sleep( maxTime );
			} catch( InterruptedException ignored ) {
			}
			setPower( 0 );


		} ).start( );
	}

	private void startIntakeChecking( ) {

		intakenBlocks = 0;

		// wait 500 milliseconds before checking
		double startTime = System.currentTimeMillis( );
		while( getPower( ) > 0 && startTime + 500 > System.currentTimeMillis( ) ) { // while the power is up, and it has been less than 500 milliseconds
			try {
				Thread.sleep( 50 );
			} catch( InterruptedException ignored ) {
			}
		}

		int intakeTimeLimit = 15 * 1000; // 30 seconds
		startTime = System.currentTimeMillis( );

		boolean previouslyIntaking = false;
		while( getPower( ) > 0 && startTime + intakeTimeLimit > System.currentTimeMillis( ) && intakenBlocks >= 0 ) { // while the power is up, and it has been less than 30 seconds
			if( intaking( ) )
				previouslyIntaking = true;
			else if( previouslyIntaking ) {
				intakenBlocks++;
				previouslyIntaking = false;
			}

			try {
				Thread.sleep( 50 );
			} catch( InterruptedException ignored ) {
			}
		}
	}

}