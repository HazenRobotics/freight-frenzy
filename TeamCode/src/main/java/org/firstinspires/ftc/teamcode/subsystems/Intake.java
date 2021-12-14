package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

	DcMotorEx intakeMotor;

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
	}

	public double getCurrent( ) {
		return getCurrent( CurrentUnit.AMPS );
	}

	public double getCurrent( CurrentUnit currentUnit ) {
		return intakeMotor.getCurrent( currentUnit );
	}
}