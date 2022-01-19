package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

	Servo grabber;

	/**
	 * creates a default grabber with a grabberName of "grabber"
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public Grabber( HardwareMap hardwareMap ) {
		setup( hardwareMap, "grabber" );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param grabberName the name of the grabber servo in the hardware map
	 */
	public Grabber( HardwareMap hardwareMap, String grabberName ) {

		setup( hardwareMap, grabberName );
	}

	public void setup( HardwareMap hardwareMap, String grabberName ) {

		grabber = hardwareMap.servo.get( grabberName );
	}

	// getters and setters

	/**
	 * @param position - position to set the servo to
	 */
	public void setPosition( double position ) {
		grabber.setPosition( position );
	}

	/**
	 * @param position - position to set the servo to asynchronously
	 */
	public void setPositionAsync( double position ) {
		new Thread( ( ) -> {
			grabber.setPosition( position );
		} ).start( );
	}

	public double getPosition( ) {
		return grabber.getPosition( );
	}

}

