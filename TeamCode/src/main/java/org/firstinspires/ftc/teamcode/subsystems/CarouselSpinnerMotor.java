package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinnerMotor {

	DcMotor spinner;

	/**
	 * creates a default carousel spinner with a spinnerName of "spinner"
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public CarouselSpinnerMotor( HardwareMap hardwareMap ) {
		setup( hardwareMap, "spinner" );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param spinnerName the name of the spinner motor in the hardware map
	 */
	public CarouselSpinnerMotor( HardwareMap hardwareMap, String spinnerName ) {
		setup( hardwareMap, spinnerName );
	}

	private void setup( HardwareMap hardwareMap, String spinnerName ) {

		spinner = hardwareMap.dcMotor.get( spinnerName );
	}

	// getters and setters

	public double getPower( ) {
		return spinner.getPower( );
	}

	public void setPower( double power ) {
		spinner.setPower( power );
	}

}
