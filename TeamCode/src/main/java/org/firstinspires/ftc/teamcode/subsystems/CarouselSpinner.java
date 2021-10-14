package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinner {

	CRServo spinner;

	// I don't think we'll need these since we have to use timed methods when rotating it.
//	double wheelRadius = 2;
//	final double GEAR_RATIO = 67;

	public CarouselSpinner( HardwareMap hw ) {
		setup( hw );
	}

	private void setup( HardwareMap hw ) {

		spinner = hw.crservo.get( "spinner" );
	}

	public void setPower( double power ) {
		spinner.setPower( power );
	}

	public double getPower( ) {
		return spinner.getPower( );
	}

}
