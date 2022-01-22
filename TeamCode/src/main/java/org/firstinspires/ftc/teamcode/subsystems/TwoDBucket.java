package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.Robot;

public class TwoDBucket extends Bucket {

	// Andrew's Vocabulary
	// extreme
	// expensive
	// expansive
	// existential
	// extra ordinary
	// extreme
	// explicit

	public Servo horBucket;

	/**
	 * the lowest angle the bucket can go to
	 */
	public double horMaxAngle;

	public double horBucketRange;

	public double horBucketCenter;
	/**
	 * creates a default bucket with:
	 * -a motorName of "bucket",
	 * -a maxAngle of -35°, and
	 * -a bucketRange of 180°
	 *
	 * @param hardwareMap the hardwareMap of the current running OpMode
	 */
	public TwoDBucket( HardwareMap hardwareMap ) {
		super( hardwareMap );
		horSetup( hardwareMap, "horBucket", 135, 135 );
	}

	/**
	 * @param hardwareMap    the hardwareMap of the current running OpMode
	 * @param verBucketName  the name of the vertical servo in the hardware map
	 * @param horBucketName  the name of the horizontal servo in the hardware map
	 * @param verMaxAngle    the max angle the vertical servo can go relative to the ground (its angle at a position of 1)
	 * @param horMaxAngle    the max angle the horizontal servo can go relative to the ground (its angle at a position of 1)
	 * @param verBucketRange the angle range of the vertical servo (default 180°)
	 * @param horBucketRange the angle range of the horizontal servo (default 180°)
	 */
	public TwoDBucket( HardwareMap hardwareMap, String verBucketName, String horBucketName,
					   double verMaxAngle, double horMaxAngle, double verBucketRange, double horBucketRange ) {

		super( hardwareMap, verBucketName, verMaxAngle, verBucketRange );

		horSetup( hardwareMap, horBucketName, horMaxAngle, horBucketRange );
	}

	public void horSetup( HardwareMap hardwareMap, String horBucketName, double horMaxAngle, double horBucketRange) {

		horBucket = hardwareMap.servo.get( horBucketName );

		setHorMaxAngle( horMaxAngle );
		setHorBucketRange( horBucketRange );
	}


	/**
	 * @param verAngle        angle to set the vertical servo to (0 being the ground and - being below it)
	 * @param horAngle        angle to set the horizontal servo to (0 being the middle and - being left)
	 * @param secondMoveDelay time to wait (milliseconds) until making the second move
	 * @param rotateHorFirst  rotate the horizontal servo, wait, then rotate the ver (false inverts)
	 */
	public void setAngle( double verAngle, double horAngle, long secondMoveDelay, boolean rotateHorFirst ) {

		if( rotateHorFirst ) {
			setHorAngle( horAngle );
			Robot.waitTime( secondMoveDelay );
			setVerAngle( verAngle );
		} else {
			setVerAngle( verAngle );
			Robot.waitTime( secondMoveDelay );
			setHorAngle( horAngle );
		}
	}

	// vertical servo stuff
	/**
	 * @param angleFromGround of the bucket
	 */
	public void setVerAngle( double angleFromGround ) {
		super.setAngle( angleFromGround );
	}

	/**
	 * @return the position (between 0 & bucketRange) where vertical servo is
	 */
	public double getVerAngle( ) {
		return super.getAngle( );
	}

	public void setVerPosition( double position ) {
		super.setPosition( position );
	}

	/**
	 * @return the position (between 0 & 1) where horizontal servo is
	 */
	public double getVerPosition( ) {
		return super.getPosition( );
	}

	// horizontal servo stuff
	/**
	 * @param angleFromCenter of the bucket
	 */
	public void setHorAngle( double angleFromCenter ) {
		setHorPosition( angleToPosition( angleFromCenter, horMaxAngle, horBucketRange ) );
	}

	/**
	 * @return the position (between 0 & bucketRange) where horizontal servo is
	 */
	public double getHorAngle( ) {
		return positionToAngle( getPosition( ), horMaxAngle, horBucketRange );
	}

	public void setHorPosition( double position ) {
		horBucket.setPosition( position );
	}

	/**
	 * @return the position (between 0 & 1) where horizontal servo is
	 */
	public double getHorPosition( ) {
		return horBucket.getPosition( );
	}


	// variable getters and setters


	// horizontal max angles(from this class)
	public double getHorMaxAngle( ) {
		return horMaxAngle;
	}

	public void setHorMaxAngle( double horMaxAngle ) {
		this.horMaxAngle = horMaxAngle;
	}


	// vertical max angle (from super class)
	public double getVerMaxAngle( ) {
		return super.maxAngle;
	}

	public void setVerMaxAngle( double verMaxAngle ) {
		super.maxAngle = verMaxAngle;
	}


	// horizontal bucket range (from this class)
	public double getHorBucketRange( ) {
		return horBucketRange;
	}

	public void setHorBucketRange( double horBucketRange ) {
		this.horBucketRange = horBucketRange;
	}


	// vertical bucket range (from super class)
	public double getVerBucketRange( ) {
		return super.bucketRange;
	}

	public void setVerBucketRange( double verBucketRange ) {
		super.bucketRange = verBucketRange;
	}
}

