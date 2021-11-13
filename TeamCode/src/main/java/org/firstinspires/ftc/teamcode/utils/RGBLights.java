package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RGBLights {

	RevBlinkinLedDriver blinkinLedDriver;

	public enum StatusLights {
		ERROR,
		SUCCESS,
		WAITING,
		NORMAL,
		CELEBRATION
	}

	public RGBLights( HardwareMap hardwareMap, String name ) {
		blinkinLedDriver = hardwareMap.get( RevBlinkinLedDriver.class, name );
	}

	public void setPattern( BlinkinPattern pattern ) {
		blinkinLedDriver.setPattern( pattern );
	}

	public void showStatus( StatusLights status ) {
		switch( status ) {
			case ERROR:
				setPattern( BlinkinPattern.SHOT_RED );
				break;
			case SUCCESS:
				setPattern( BlinkinPattern.GREEN );
				break;
			case WAITING:
				setPattern( BlinkinPattern.STROBE_GOLD );
				break;
			case NORMAL:
				setPattern( BlinkinPattern.CP1_2_SPARKLE_1_ON_2 );
				break;
			case CELEBRATION:
				setPattern( BlinkinPattern.CONFETTI );
				break;
		}
	}
}
