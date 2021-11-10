package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Capper {

	// maybe objects for bucket length (for rotation radius)
	// or for the (length of the string connecting) distance between the servo and connection to the bucket (probably at the end of the bucket)

	public Servo capper;

	public Capper( HardwareMap hw ) {
		setup( hw, "capper" );
	}

	public Capper( HardwareMap hw, String pusherName ) {

		setup( hw, pusherName );
	}

	public void setup( HardwareMap hw, String pusherName ) {

		capper = hw.servo.get( pusherName );
	}

	public void setPosition( double position ) {
		capper.setPosition( position );
	}

	public double getPosition( ) {
		return capper.getPosition( );
	}
}

