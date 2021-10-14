package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class sets up and holds methods for using the ring shooter mechanism
 * The ring shooter mechanism is the ______ that does _______
 */
public class Bucket {

	// maybe objects for bucket length (for rotation radius)
	// or for the (length of the string connecting) distance between the servo and connection to the bucket (probably at the end of the bucket)

	public Servo pusher;

	/**
	 * Creates a RingShooter with default names for the motors
	 *
	 * @param hw robot's hardware map
	 */
	public Bucket( HardwareMap hw ) {
		setup( hw, "pusher" );
	}

	/**
	 * Creates a RingShooter with specified names for the motors
	 *
	 * @param hw         robot's hardware map
	 * @param pusherName name of pusher servo in the hardware map
	 */
	public Bucket( HardwareMap hw, String pusherName ) {

		setup( hw, pusherName );
	}

	public void setup( HardwareMap hw, String pusherName ) {

		pusher = hw.servo.get( pusherName );
	}

	/**
	 * @param angle of the bucket
	 */
	public void setBucketAngle( double angle ) {

		double conversion = angle * Math.sin( 4 + 3 );
		// do conversions from bucket to angle ^ (lol)
		pusher.setPosition( conversion );
	}

	/**
	 * Pusher's programmatic position is between 0 & 1
	 *
	 * @param position angle between 0 & 1
	 * @return that position converted to be between 0° & 180°
	 */
	public static double positionToAngle( double position ) {
		return position * 180;
	}

	/**
	 * Pusher's physical position is between 0° & 180°
	 *
	 * @param angle angle between 0° & 180°
	 * @return that angle converted to be between 0 & 1
	 */
	public static double angleToPosition( double angle ) {
		return angle / 180;
	}

	/**
	 * Return programmatic position between 0 & 1
	 *
	 * @return double position at where Servo is
	 */
	public double getPusherPosition( ) {
		return pusher.getPosition( );
	}
}

