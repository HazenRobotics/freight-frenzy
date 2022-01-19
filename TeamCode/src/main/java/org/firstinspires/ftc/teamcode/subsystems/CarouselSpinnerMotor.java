package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CarouselSpinnerMotor {

	DcMotorEx spinner;

	/**
	 * creates a default carousel spinner with a spinnerName of "spinner"
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public CarouselSpinnerMotor( HardwareMap hardwareMap ) {
		setup( hardwareMap, "spinner" , false );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param spinnerName the name of the spinner motor in the hardware map
	 */
	public CarouselSpinnerMotor( HardwareMap hardwareMap, String spinnerName ) {
		setup( hardwareMap, spinnerName, false );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param spinnerName the name of the spinner motor in the hardware map
	 */
	public CarouselSpinnerMotor( HardwareMap hardwareMap, String spinnerName, boolean runWithEncoders ) {
		setup( hardwareMap, spinnerName, runWithEncoders );
	}

	private void setup( HardwareMap hardwareMap, String spinnerName, boolean runWithEncoders ) {

		spinner = hardwareMap.get( DcMotorEx.class, spinnerName );
		if( runWithEncoders ) {
			spinner.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		}
	}

	// getters and setters

	public double getPower( ) {
		return spinner.getPower( );
	}

	public void setPower( double power ) {
		spinner.setPower( power );
	}

	public void setVelocity( double velocity ) {
		setVelocity( velocity, AngleUnit.DEGREES );
	}

	public void setVelocity( double velocity, AngleUnit angleUnit ) {
		spinner.setVelocity( velocity, angleUnit );
	}

	public double getVelocity( ) {
		return getVelocity( AngleUnit.DEGREES );
	}

	public double getVelocity( AngleUnit angleUnit ) {
		return spinner.getVelocity( angleUnit );
	}
}
