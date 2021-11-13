package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Capper {

	Servo capper;

	/**
	 * creates a default capper with a capperName of "capper"
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public Capper( HardwareMap hardwareMap ) {
		setup( hardwareMap, "capper" );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param capperName the name of the capper servo in the hardware map
	 */
	public Capper( HardwareMap hardwareMap, String capperName ) {

		setup( hardwareMap, capperName );
	}

	public void setup( HardwareMap hardwareMap, String capperName ) {

		capper = hardwareMap.servo.get( capperName );
	}

	// getters and setters

	public double getPosition( ) {
		return capper.getPosition( );
	}

	public void setPosition( double position ) {
		capper.setPosition( position );
	}
}

