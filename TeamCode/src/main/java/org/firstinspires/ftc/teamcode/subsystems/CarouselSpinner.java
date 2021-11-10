package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinner {

	CRServo spinner;

	public CarouselSpinner( HardwareMap hardwareMap ) {
		setup( hardwareMap, "spinner" );
	}

	public CarouselSpinner( HardwareMap hardwareMap, String name ) {
		setup( hardwareMap, name );
	}

	private void setup( HardwareMap hardwareMap, String name ) {

		spinner = hardwareMap.crservo.get( name );
	}

	public void setPower( double power ) {
		spinner.setPower( power );
	}

	public double getPower( ) {
		return spinner.getPower( );
	}

}
