package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SquareIntake {

	DcMotor leftIntakeMotor;
	DcMotor rightIntakeMotor;

	/**
	 * creates a default square intake with a leftIntakeName of "intakeLeft" & a rightIntakeName of "intakeRight"
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public SquareIntake( HardwareMap hardwareMap ) {
		setup( hardwareMap, "intakeLeft", "intakeRight" );
	}

	/**
	 * @param hardwareMap     the hardwareMap of the current running OpMode
	 * @param leftIntakeName  the name of the left intake motor in the hardware map
	 * @param rightIntakeName the name of the right intake motor in the hardware map
	 */
	public void SquareIntake( HardwareMap hardwareMap, String leftIntakeName, String rightIntakeName ) {
		setup( hardwareMap, leftIntakeName, rightIntakeName );
	}

	public void setup( HardwareMap hardwareMap, String leftMotorName, String rightMotorName ) {

		leftIntakeMotor = hardwareMap.dcMotor.get( leftMotorName );
		rightIntakeMotor = hardwareMap.dcMotor.get( rightMotorName );

		rightIntakeMotor.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	// getters and setters

	public double getLeftPower( ) {
		return leftIntakeMotor.getCurrentPosition( );
	}

	public double getRightPower( ) {
		return rightIntakeMotor.getCurrentPosition( );
	}

	public double[] getPowers( ) {
		return new double[]{ getLeftPower( ), getRightPower( ) };
	}

	public void setPower( double power ) {

		leftIntakeMotor.setPower( power );
		rightIntakeMotor.setPower( power );
	}

}