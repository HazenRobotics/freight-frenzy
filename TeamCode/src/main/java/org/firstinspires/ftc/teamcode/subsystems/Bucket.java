package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.Robot;

public class Bucket {

	public Servo bucket;

	public double maxAngle;

	public double bucketRange;

	/**
	 * creates a default bucket with:
	 * -a motorName of "bucket",
	 * -a maxAngle of -35°, and
	 * -a bucketRange of 180°
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public Bucket( HardwareMap hardwareMap ) {
		setup( hardwareMap, "bucket", -35, 180 );
	}

	/**
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 * @param bucketName  the name of the bucket in the hardware map
	 * @param maxAngle    the max angle the servo can go relative to the ground (its angle at a position of 1)
	 * @param bucketRange the angle range of the servo (default 180°)
	 */
	public Bucket( HardwareMap hardwareMap, String bucketName, double maxAngle, double bucketRange ) {

		setup( hardwareMap, bucketName, maxAngle, bucketRange );
	}

	public void setup( HardwareMap hardwareMap, String bucketName, double maxAngle, double bucketRange ) {

		bucket = hardwareMap.servo.get( bucketName );
		this.maxAngle = maxAngle;
		this.bucketRange = bucketRange;
	}

	/**
	 * @param angleFromGround of the bucket
	 */
	public void setAngle( double angleFromGround ) {
		Robot.writeToDefaultFile( "setAngle " + angleFromGround + ", " + angleToPosition( angleFromGround ), true, true );
		setPosition( angleToPosition( angleFromGround ) );
	}

	/*
	 * 	public static double exampleGetPositionFromAngle( double angle ) {
	 *		// range: 0 = 145, 1 = -35
	 *		// shifted = range of 180
	 *		return 1 - ((angle + 35) / 180);
	 *	}
	 */

	public void setPosition( double position ) {
		bucket.setPosition( position );
	}

	/**
	 * Bucket's programmatic position is between 0 & 1
	 *
	 * @param position angle between 0 & 1
	 * @return that position converted to be between maxAngle° & maxAngle - angleLimit (default: 180)°
	 */
	public double positionToAngle( double position ) {
		return bucketRange * (position + 1) + maxAngle;
	}

	/**
	 * Bucket's physical position is between 0° & angleLimit (default: 180)°
	 *
	 * @param angle angle between 0° & angleLimit (default: 180)°
	 * @return that angle converted to be between 0 & 1
	 */
	public double angleToPosition( double angle ) {
		Robot.writeToDefaultFile( "Calc: " + (1 - ((angle - maxAngle) / bucketRange)), true, true );
		return 1 - ((angle - maxAngle) / bucketRange);
	}

	// p = 1 - ((angle-max)/r)
	// p + 1 = (angle-max)/r
	// r(p + 1) = angle - max
	// angle = r(p + 1) + max

	/**
	 * @return the position (between 0 & 1) where Servo is
	 */
	public double getPosition( ) {
		return bucket.getPosition( );
	}

	/**
	 * @return the position (between 0 & maxAngle) where Servo is
	 */
	public double getAngle( ) {
		return positionToAngle( getPosition( ) );
	}

	// getters and setters

	public double getMaxAngle( ) {
		return maxAngle;
	}

	public void setMaxAngle( double maxAngle ) {
		this.maxAngle = maxAngle;
	}

	public double getBucketRange( ) {
		return bucketRange;
	}

	public void setBucketRange( double bucketRange ) {
		this.bucketRange = bucketRange;
	}

}

