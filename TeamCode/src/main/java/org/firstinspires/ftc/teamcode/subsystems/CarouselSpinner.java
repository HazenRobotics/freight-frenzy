package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinner {

	CRServo spinner;

	public CarouselSpinner( HardwareMap hw ) {
		setup( hw, "spinner" );
	}

	public CarouselSpinner( HardwareMap hw, String name ) {
		setup( hw, name );
	}

	private void setup( HardwareMap hw, String name ) {

		spinner = hw.crservo.get( name );
	}

	public void setPower( double power ) {
		spinner.setPower( power );
	}

	public double getPower( ) {
		return spinner.getPower( );
	}

}
