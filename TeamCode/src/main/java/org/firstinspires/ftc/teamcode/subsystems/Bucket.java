package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {

	public Servo bucket;

	public double maxAngle;

	public double angleLimit;

	public Bucket( HardwareMap hw ) {
		setup( hw, "bucket", 0, 180 );
	}

	public Bucket( HardwareMap hw, String bucketName, double maxAngle, double angleLimit ) {

		setup( hw, bucketName, maxAngle, angleLimit );
	}

	public void setup( HardwareMap hw, String bucketName, double maxAngle, double angleLimit ) {

		bucket = hw.servo.get( bucketName );
		this.maxAngle = maxAngle;
		this.angleLimit = angleLimit;
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
	 * @return that position converted to be between 0° & angleLimit (default: 180)°
	 */
	public double positionToAngle( double position ) {
		return position * angleLimit;
	}

	/**
	 * Bucket's physical position is between 0° & angleLimit (default: 180)°
	 *
	 * @param angle angle between 0° & angleLimit (default: 180)°
	 * @return that angle converted to be between 0 & 1
	 */
	public double angleToPosition( double angle ) {
		return (angle % angleLimit) / angleLimit;
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

