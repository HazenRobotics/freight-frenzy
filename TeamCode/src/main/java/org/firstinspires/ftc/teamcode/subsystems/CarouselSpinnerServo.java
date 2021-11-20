package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinnerServo {

	CRServo spinner;

	/**
	 * creates a default carousel spinner with a spinnerName of "spinner"
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public CarouselSpinnerServo( HardwareMap hardwareMap ) {
		setup( hardwareMap, "spinner" );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param spinnerName the name of the spinner servo in the hardware map
	 */
	public CarouselSpinnerServo( HardwareMap hardwareMap, String spinnerName ) {
		setup( hardwareMap, spinnerName );
	}

	private void setup( HardwareMap hardwareMap, String spinnerName ) {

		spinner = hardwareMap.crservo.get( spinnerName );
	}

	// getters and setters

	public double getPower( ) {
		return spinner.getPower( );
	}

	public void setPower( double power ) {
		spinner.setPower( power );
	}

}
