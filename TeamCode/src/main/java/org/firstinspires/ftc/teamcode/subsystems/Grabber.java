package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Grabber {

	CRServo grabber;

	private boolean open = true;

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

		grabber = hardwareMap.crservo.get( grabberName );
	}

	// getters and setters
	public void open( ) {
		open = true;
		setPowerTime( 1, 1000 );
	}

	public void close( ) {
		open = false;
		setPowerTime( -1, 1000 );
	}

	/**
	 * @param power - power to set the servo to for an amount of time
	 * @param time  time to set power in milliseconds
	 */
	public void setPowerTime( double power, int time ) {
		new Thread( ( ) -> {
			grabber.setPower( power );
			long startTime = System.currentTimeMillis( );
			while( System.currentTimeMillis( ) < startTime + time ) ;
			stop( );
		} ).start( );
	}

	public void stop( ) {
		setPower( 0 );
	}

	public void setPower( double power ) {
		grabber.setPower( power );
	}

	public double getPower( ) {
		return grabber.getPower( );
	}

	public boolean isOpen( ) {
		return open;
	}

	public boolean isClosed( ) {
		return !open;
	}

}

