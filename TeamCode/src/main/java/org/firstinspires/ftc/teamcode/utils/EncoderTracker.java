package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class EncoderTracker {

	HardwareMap hardwareMap;

	DcMotorEx parallelEncoder1;
	DcMotorEx parallelEncoder2;
	DcMotorEx perpendicularEncoder;

	boolean threeEncoders;

	double encoderRadius;

	public EncoderTracker( HardwareMap hw ) {
		this( hw, "intake", "perpendicular" );
	}

	public EncoderTracker( HardwareMap hw, int encoderRad ) {

		this( hw, encoderRad, "intake", "perpendicular" );

		encoderRadius = encoderRad;
	}

	public EncoderTracker( HardwareMap hw, String parallelName, String perpendicularName ) {

		hardwareMap = hw;

		initEncoders( parallelName, perpendicularName );
	}

	public EncoderTracker( HardwareMap hw, String parallel1Name, String parallel2Name, String perpendicularName ) {

		hardwareMap = hw;

		initEncoders( parallel1Name, parallel2Name, perpendicularName );
	}

	public EncoderTracker( HardwareMap hw, int encoderRad, String parallelName, String perpendicularName ) {

		this( hw, parallelName, perpendicularName );

		encoderRadius = encoderRad;
	}

	private void initEncoders( String parallelName, String perpendicularName ) {

		threeEncoders = false;
		parallelEncoder1 = hardwareMap.get( DcMotorEx.class, parallelName );
		perpendicularEncoder = hardwareMap.get( DcMotorEx.class, perpendicularName );
	}

	private void initEncoders( String parallel1Name, String parallel2Name, String perpendicularName ) {

		threeEncoders = true;
		parallelEncoder1 = hardwareMap.get( DcMotorEx.class, parallel1Name );
		parallelEncoder2 = hardwareMap.get( DcMotorEx.class, parallel2Name );
		perpendicularEncoder = hardwareMap.get( DcMotorEx.class, perpendicularName );
	}

	// *************** all encoder methods ***************

	/**
	 * returns the Longitudinal (forward/backward) distance on the encoders from where it started
	 *
	 * @return the position of the parallel encoder
	 */
	public int getLongitudinalPosition( ) {
		if( threeEncoders )
			return (parallelEncoder1.getCurrentPosition( ) + parallelEncoder2.getCurrentPosition( )) / 2;
		return parallelEncoder1.getCurrentPosition( );
	}

	/**
	 * returns the Lateral (left/right) distance on the encoders from where it started
	 *
	 * @return the position of the perpendicular encoder
	 */
	public int getLateralPosition( ) {
		return perpendicularEncoder.getCurrentPosition( );
	}

	/**
	 * @return the velocity of the parallel encoder in ticks per second
	 */
	public double getLongitudinalVelocity( ) {
		if( threeEncoders )
			return (parallelEncoder1.getVelocity( ) + parallelEncoder2.getVelocity( )) / 2;
		return parallelEncoder1.getVelocity( );
	}

	/**
	 * @param angleUnit the unit to get velocity in
	 * @return the velocity of the parallel encoder in @angleUnit
	 */
	public double getLongitudinalVelocity( AngleUnit angleUnit ) {
		if( threeEncoders )
			return (parallelEncoder1.getVelocity( angleUnit ) + parallelEncoder2.getVelocity( angleUnit )) / 2;
		return parallelEncoder1.getVelocity( angleUnit );
	}

	/**
	 * @return the velocity of the perpendicular encoder in ticks per second
	 */
	public double getLateralVelocity( ) {
		return perpendicularEncoder.getVelocity( );
	}

	/**
	 * @param angleUnit the unit to get velocity
	 * @return the velocity of the perpendicular encoder in @angleUnit
	 */
	public double getLateralVelocity( AngleUnit angleUnit ) {
		return perpendicularEncoder.getVelocity( angleUnit );
	}

	/**
	 * convert a distance (in inches) to ticks
	 *
	 * @param distance            the distance you want to convert to ticks
	 * @param wheelDiameter       the diameter of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the number of ticks in that distance
	 */
	public static int convertDistTicks( double distance, double wheelDiameter, double pulsesPerRevolution, double gearRatio ) {
		double revolutions = distance / Math.PI * wheelDiameter;
		int totalTicks = (int) Math.round( (revolutions * pulsesPerRevolution) / gearRatio );

		return totalTicks;
	}

	/**
	 * convert a number of ticks to distance (in inches)
	 *
	 * @param ticks               the ticks you want to convert to distance
	 * @param wheelDiameter       the diameter of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the distance (in inches) in that number of ticks
	 */
	public static double convertTicksDist( double ticks, double wheelDiameter, double pulsesPerRevolution, double gearRatio ) {
		double circumference = Math.PI * wheelDiameter;
		double totalDistance = (ticks * circumference * gearRatio) / pulsesPerRevolution;

		return totalDistance;
	}

	/**
	 * will sleep the robot for [millis] milliseconds
	 *
	 * @param millis
	 */
	public void sleep( long millis ) {
		long startTime = System.currentTimeMillis( );
		while( System.currentTimeMillis( ) < startTime + millis ) ;
	}


}
