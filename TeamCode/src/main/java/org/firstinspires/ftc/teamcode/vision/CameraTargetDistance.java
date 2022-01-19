package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CameraTargetDistance {

	// Object Dimensions - width, height
	/**
	 * the dimensions of a Ball in inches (width, height)
	 */
	public static final Vector2d BALL_DIM = new Vector2d( 2.75, 2.75 ); // 14.75" -> 127 pixels,      27" -> 64 pixels

	/**
	 * the dimensions of a Duck in inches (width, height)
	 */
	public static final Vector2d DUCK_DIM = new Vector2d( 2, 2.125 );
	/**
	 * the dimensions of an Element in inches (width, height)
	 */
	public static final Vector2d ELEMENT_DIM = new Vector2d( 3, 5 );
	/**
	 * the dimensions of a Heavy Cube in inches (width, height)
	 */
	public static final Vector2d HEAVY_DIM = new Vector2d( 2, 2 );
	/**
	 * the dimensions of a Light Cube in inches (width, height)
	 */
	public static final Vector2d LIGHT_DIM = new Vector2d( 2, 2 );
	/**
	 * the dimensions of a Medium Cube in inches (width, height)
	 */
	public static final Vector2d MEDIUM_DIM = new Vector2d( 2, 2 );


	public enum ObjectType {
		BALL,
		DUCK,
		ELEMENT,
		HEAVY,
		LIGHT,
		MEDIUM
	}


	/**
	 * Duck Distance Ratio (graph) a, p, h -> scale, x offset, y offset
	 */
	static final double[] DUCK_DIST_RAT = new double[]{ 750, 7.4, 15.7 };

	/*

	use scale to get complete distance
	use camera angle to get distance from robot
	use center offset to get lateral error (turning offset)
	turn the lateral error (turning offset) into an angle and get the x and y components for moving

	Note:
	might have to find a way to calculate the distance with a different camera angle

	think: what happens with the distance
	when the camera is looking straight at it
	vs
	when the camera is constantly looking at one angle

	and you move a block

	 */

	private double cameraWidth;
	private double cameraFOV;
	private double cameraAngle;

//	double camWidth = 720, camFOV = 45;
//	double cameraAngle = 90 - 22.5;

	public CameraTargetDistance( double camWidth, double camFOV, double camAngle ) {

		cameraWidth = camWidth;
		cameraFOV = camFOV;
		cameraAngle = camAngle;
	}

	public Vector2d getDistance( double x, double width ) {
		return componentDistanceFromTarget( parDistFromTargetSize( width ), getLateralError( x + (width / 2), cameraWidth, cameraFOV ), cameraAngle );
	}

	/**
	 * @param distance     the direct distance to the target object
	 * @param headingError the lateral error (turning offset) from the target object
	 * @param camAngle     the vertical angle of camera (degrees) (0° is perpendicular with & facing the ground)
	 * @return the x and y components of how far the target object is (where x is parallel with the camera, y is perpendicular, and z would be the height)
	 */
	public static Vector2d componentDistanceFromTarget( double distance, double headingError, double camAngle ) {

		double flatParallel = distance * Math.sin( Math.toRadians( camAngle ) );

		return new Vector2d( flatParallel, flatParallel * Math.tan( Math.toRadians( headingError ) ) );
	}

	/**
	 * @param width      the width of the target object (pixels)
	 * @param height     the height of the target object (pixels)
	 * @param objectType the type of the target object (ObjectType)
	 * @return how far the robot is from the object (inches)
	 */
	public static double directDistanceFromTarget( double width, double height, ObjectType objectType ) {

		return parDistFromTargetSize( (width + height) / 2 );
	}


	/**
	 * finds the direction a target is from the camera
	 *
	 * @param x        the x position (center) of the target object (pixels) (x = 0, is far left)
	 * @param camWidth the width of the camera (pixels)
	 * @param camFOV   the field of view of the camera (degrees): normally 55°
	 * @return the lateral error (turning offset) from the target object (0 is straight, + is right, - is left)
	 */
	public static double getLateralError( double x, double camWidth, double camFOV ) {

		camWidth /= 2;
		camFOV /= 2;

		// x offset from center of screen
		double xOffset = x - camWidth;

		return (xOffset / camWidth) * camFOV;
	}

	/**
	 * @param targetObjectSize size of the target object (pixels)
	 * @return the direct distance from target object (parallel/straight from the camera)
	 * -1 if the targetObjectSize is less than p (below the horizontal asymptote), -2 if the distance calculated is less than h (below the vertical asymptote)
	 */
	public static double parDistFromTargetSize( double targetObjectSize ) {
		return parDistFromTargetSize( targetObjectSize, DUCK_DIST_RAT );
	}

	/**
	 * @param targetObjectSize size of the target object (pixels)
	 * @param distanceRatios   the distance ratio (graph) a, p, h -> scale, x offset, y offset
	 * @return the direct distance from target object (parallel/straight from the camera)
	 * -1 if the targetObjectSize is less than p (below the horizontal asymptote), -2 if the distance calculated is less than h (below the vertical asymptote)
	 */
	public static double parDistFromTargetSize( double targetObjectSize, double[] distanceRatios ) {
		if( targetObjectSize <= distanceRatios[2] )
			return -1;
		double distance = distanceRatios[0] / (targetObjectSize - distanceRatios[2]) + distanceRatios[1];
		if( distance <= distanceRatios[1] )
			return -2;
		return distance;
	}

	/**
	 * @param targetSize the width of the target object (inches)
	 * @return the distance ratio (graph) dataset
	 */
	public static double[] distanceRatioFromTargetSize( double targetSize ) {

//		return new double[]{ 770, 20, 8 }; // duck og
		return new double[]{ 750, 15.7, 7.4 };
	}

}
