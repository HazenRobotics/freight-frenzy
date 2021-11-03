package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {

	public Servo bucket;

	public int maxAngle;

	public Bucket( HardwareMap hw ) {
		setup( hw, "bucket", 0 );
	}

	public Bucket( HardwareMap hw, int maxAngle ) {

		setup( hw, "bucket", maxAngle );
	}

	public Bucket( HardwareMap hw, String bucketName ) {

		setup( hw, bucketName, 0 );
	}

	public Bucket( HardwareMap hw, String bucketName, int maxAngle ) {

		setup( hw, bucketName, maxAngle );
	}

	public void setup( HardwareMap hw, String bucketName, int maxAngle ) {

		bucket = hw.servo.get( bucketName );
		this.maxAngle = maxAngle;
	}

	/**
	 * @param angleFromGround of the bucket
	 */
	public void setBucketAngle( double angleFromGround ) {

		bucket.setPosition( angleToPosition( angleFromGround - maxAngle ) );
	}

	/**
	 * Bucket's programmatic position is between 0 & 1
	 *
	 * @param position angle between 0 & 1
	 * @return that position converted to be between 0° & 180°
	 */
	public static double positionToAngle( double position ) {
		return position * 180;
	}

	/**
	 * Bucket's physical position is between 0° & 180°
	 *
	 * @param angle angle between 0° & 180°
	 * @return that angle converted to be between 0 & 1
	 */
	public static double angleToPosition( double angle ) {
		return (angle % 180) / 180;
	}

	/**
	 * Return programmatic position between 0 & 1
	 *
	 * @return double position at where Servo is
	 */
	public double getBucketPosition( ) {
		return bucket.getPosition( );
	}
}

