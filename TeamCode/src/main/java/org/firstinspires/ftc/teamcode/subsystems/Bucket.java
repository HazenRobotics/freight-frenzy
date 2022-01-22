package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {

	public Servo bucket;

	/**
	 * the lowest angle the bucket can go to
	 */
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

		setMaxAngle( maxAngle );
		setBucketRange( bucketRange );
	}

	/**
	 * @param angleFromGround of the bucket
	 */
	public void setAngle( double angleFromGround ) {
		setPosition( angleToPosition( angleFromGround ) );
	}

	public void setPosition( double position ) {
		bucket.setPosition( position );
	}

	/**
	 * Bucket's programmatic position is between 0 & 1
	 *
	 * @param position angle between 0 & 1
	 * @return that position converted to be between maxAngle° & maxAngle - bucketRange (default: 180)°
	 */
	public double positionToAngle( double position ) {
		return bucketRange * (1 - position) + maxAngle;
	}

	/**
	 * Bucket's programmatic position is between 0 & 1
	 *
	 * @param position    angle between 0 & 1
	 * @param maxAngle    the max angle of the bucket
	 * @param bucketRange the range of the bucket in degrees
	 * @return that position converted to be between maxAngle° & maxAngle - bucketRange (default: 180)°
	 */
	public static double positionToAngle( double position, double maxAngle, double bucketRange ) {
		return bucketRange * (1 - position) + maxAngle;
	}

	/**
	 * Bucket's physical position is between 0° & bucketRange (default: 180)°
	 *
	 * @param angle angle between 0° & bucketRange (default: 180)°
	 * @return that angle converted to be between 0 & 1
	 */
	public double angleToPosition( double angle ) {
		return angleToPosition( angle, maxAngle, bucketRange );
	}

	/**
	 * Bucket's physical position is between 0° & bucketRange (default: 180)°
	 *
	 * @param angle       angle between 0° & bucketRange (default: 180)°
	 * @param maxAngle    the max angle of the bucket
	 * @param bucketRange the range of the bucket in degrees
	 * @return that angle converted to be between 0 & 1
	 */
	public static double angleToPosition( double angle, double maxAngle, double bucketRange ) {
		return 1 - ((angle - maxAngle) / bucketRange);
	}

	// p = 1 - (angle - max)/r
	// p + (angle - max)/r = 1
	// (angle - max)/r = 1 - p
	// angle - max = r(1 - p)
	// angle = r(1 - p) + max

	/**
	 * @return the position (between 0 & 1) where Servo is
	 */
	public double getPosition( ) {
		return bucket.getPosition( );
	}

	/**
	 * @return the position (between 0 & bucketRange) where Servo is
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

