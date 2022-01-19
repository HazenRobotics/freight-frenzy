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

	/**
	 * pulses per revolution
	 */
	double ppr;
	double gearRatio;
	double encoderRadius;

	public EncoderTracker( HardwareMap hardwareMap ) {
		setup( hardwareMap, 1, 1, 1 );
		initEncoders( "intake", "perpendicular" );
	}

	public EncoderTracker( HardwareMap hardwareMap, String parallelName, String perpendicularName, double encoderRadius, double ppr, double gearRatio ) {

		setup( hardwareMap, ppr, gearRatio, encoderRadius );
		initEncoders( parallelName, perpendicularName );
	}

	public EncoderTracker( HardwareMap hardwareMap, String parallel1Name, String parallel2Name, String perpendicularName, double encoderRadius, double ppr, double gearRatio ) {

		setup( hardwareMap, ppr, gearRatio, encoderRadius );
		initEncoders( parallel1Name, parallel2Name, perpendicularName );
	}

	private void setup( HardwareMap hardwareMap, double ppr, double gearRatio, double encoderRadius ) {

		this.hardwareMap = hardwareMap;

		this.ppr = ppr;
		this.gearRatio = gearRatio;
		this.encoderRadius = encoderRadius;
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

	// conversions

	public int convertDistTicks( double distance ) {
		return convertDistTicks( distance, encoderRadius, ppr, gearRatio );
	}

	public double convertTicksDist( int ticks ) {
		return convertTicksDist( ticks, encoderRadius, ppr, gearRatio );
	}

	/**
	 * convert a distance (in inches) to ticks
	 *
	 * @param distance            the distance you want to convert to ticks
	 * @param wheelRadius         the diameter of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the number of ticks in that distance
	 */
	public static int convertDistTicks( double distance, double wheelRadius, double pulsesPerRevolution, double gearRatio ) {
		double revolutions = distance / Math.PI * 2 * wheelRadius;
		return (int) Math.round( (revolutions * pulsesPerRevolution) / gearRatio );
	}

	/**
	 * convert a number of ticks to distance (in inches)
	 *
	 * @param ticks               the ticks you want to convert to distance
	 * @param wheelRadius         the radius of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the distance (in inches) in that number of ticks
	 */
	public static double convertTicksDist( int ticks, double wheelRadius, double pulsesPerRevolution, double gearRatio ) {
		double circumference = 2 * Math.PI * wheelRadius;
		return (ticks * circumference * gearRatio) / pulsesPerRevolution;
	}

	// setters and getters

	public void setPPR( double ppr ) {
		this.ppr = ppr;
	}

	public double getPPR( ) {
		return ppr;
	}

	public void setGearRatio( double gearRatio ) {
		this.gearRatio = gearRatio;
	}

	public double getGearRatio( ) {
		return gearRatio;
	}

	public void setEncoderRadius( double encoderRadius ) {
		this.encoderRadius = encoderRadius;
	}

	public double getEncoderRadius( ) {
		return encoderRadius;
	}
}
