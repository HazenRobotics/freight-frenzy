package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.Robot;

public class Bucket {

	public Servo bucket;

	public double maxAngle;

	public double angleLimit;

	public Bucket( HardwareMap hw ) {
		setup( hw, "bucket", -35, 180 );
	}

	public Bucket( HardwareMap hw, String bucketName, double maxAngle, double angleLimit ) {

		setup( hw, bucketName, maxAngle, angleLimit );
	}

	/**
	 *
	 * @param hw the hardware map of the robot OpMode
	 * @param bucketName the name of the bucket in the hardware map
	 * @param maxAngle the max angle the servo can go relative to the ground (its angle at a position of 1)
	 * @param angleLimit the angle range of the servo (default 180°)
	 */
	public void setup( HardwareMap hw, String bucketName, double maxAngle, double angleLimit ) {

		bucket = hw.servo.get( bucketName );
		this.maxAngle = maxAngle;
		this.angleLimit = angleLimit;
	}

	/**
	 * @param angleFromGround of the bucket
	 */
	public void setAngle( double angleFromGround ) {
		Robot.writeToDefaultFile( "setAngle " + angleFromGround + ", " + angleToPosition( angleFromGround ), true, true );
		Log.e( "BUCKET_TEST", "setAngle " + angleFromGround + ", " + angleToPosition( angleFromGround ) );
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
		Log.e( "ANG_POS", "Calc: " + (1 - ((angle - maxAngle) / angleLimit)) );
		return 1 - ((angle - maxAngle) / angleLimit);
	}

	/**
	 * Return programmatic position between 0 & 1
	 *
	 * @return double position at where Servo is
	 */
	public double getPosition( ) {
		return bucket.getPosition( );
	}
}

