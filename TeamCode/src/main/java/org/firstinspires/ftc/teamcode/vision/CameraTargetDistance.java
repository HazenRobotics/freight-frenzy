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
	static final double[] DUCK_DIST_RAT = new double[]{ 770, 20, 8 };

	public static void main( String[] args ) {

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

		/*
		normal: 223 x 634


		 */

		// duck at distance of 20.5"
		// floor distance of 17"
		// top: 349.5
		// bottom: 441.5
		// left: 235.5
		// right: 318.4
		// height: 88.8
		// width: 82.2
		/*
		Distance (from target): 47.927012791572615
		Lateral Error (from target): 57.895833333333336
		Final Distance (from target): (74.783, 46.919)
		*/

		// duck at distance of 41"
		// floor distance of 39.5"
		// top: 137.9
		// bottom: 177.7
		// left: 275.1
		// right: 312.5
		// height: 39.9
		// width: 38.1
		/*
		Distance (from target): 51.14014296463507
		Lateral Error (from target): 61.74583333333333
		Final Distance (from target): (89.587, 48.145)
		*/

//		double width = 82.2, height = 88.8;
		double width = 38.1, height = 39.9;
		ObjectType objectType = ObjectType.DUCK;
		double distance = straightDistanceFromTarget( width, height, objectType );

		double camWidth = 720, camFOV = 70;
//		double x = 235.5;
		double x = 275.1;
		double error = getLateralError( x, camWidth, camFOV );

		double cameraAngle = 33.97635282; // normal max of 22.5
		Vector2d finalDistance = componentDistanceFromTarget( distance, error, cameraAngle );

		System.out.println( "Distance (from target): " + distance );
		System.out.println( "Lateral Error (from target): " + error );
		System.out.println( "Final Distance (from target): " + finalDistance );
		System.out.println( "--------------------------------------------" );

	}

	/**
	 * @param distance     the straight distance from the target object
	 * @param headingError the lateral error (turning offset) from the target object
	 * @param camAngle     the vertical angle of camera (degrees) (0° is parallel with the ground)
	 * @return the x and y components of how far the target object is (where x is parallel with the camera, y is perpendicular, and z would be the height)
	 */
	public static Vector2d componentDistanceFromTarget( double distance, double headingError, double camAngle ) {

		double flatDistance = distance * Math.cos( Math.toRadians( camAngle ) );

		return new Vector2d( flatDistance / Math.cos( Math.toRadians( headingError ) ), flatDistance / Math.sin( Math.toRadians( headingError ) ) );
	}

	/**
	 * @param width      the width of the target object (pixels)
	 * @param height     the height of the target object (pixels)
	 * @param objectType the type of the target object (ObjectType)
	 * @return how far the robot is from the object (inches)
	 */
	public static double straightDistanceFromTarget( double width, double height, ObjectType objectType ) {

		return distanceFromTargetSize( (width + height) / 2 );
	}


	/**
	 * finds the direction a target is from the camera
	 *
	 * @param x        the x position of the target object (pixels)
	 * @param camWidth the width of the camera (pixels)
	 * @param camFOV   the field of view of the camera (degrees): normally 55°
	 * @return the lateral error (turning offset) from the target object
	 */
	public static double getLateralError( double x, double camWidth, double camFOV ) {

		double xOffset = camWidth / 2 + x;

		return xOffset * camFOV / camWidth;
	}

	/**
	 * @param targetObjectSize size of the target object (pixels)
	 * @return the distance from target object
	 * -1 if the targetObjectSize is less than p (below the horizontal asymptote), -2 if the distance calculated is less than h (below the vertical asymptote)
	 */
	public static double distanceFromTargetSize( double targetObjectSize ) {
		return distanceFromTargetSize( targetObjectSize, DUCK_DIST_RAT );
	}

	/**
	 * @param targetObjectSize size of the target object (pixels)
	 * @param distanceRatios the distance ratio (graph) a, p, h -> scale, x offset, y offset
	 * @return the distance from target object
	 * -1 if the targetObjectSize is less than p (below the horizontal asymptote), -2 if the distance calculated is less than h (below the vertical asymptote)
	 */
	public static double distanceFromTargetSize( double targetObjectSize, double[] distanceRatios ) {
		if( targetObjectSize <= distanceRatios[2] )
			return -1;
		double distance = distanceRatios[0] / (targetObjectSize - distanceRatios[1]) + distanceRatios[2];
		if( distance <= distanceRatios[1] )
			return -2;
		return distance;
	}

	/**
	 *
	 * @param targetSize the width of the target object (inches)
	 * @return the distance ratio (graph) dataset
	 */
	public static double[] distanceRatioFromTargetSize( double targetSize ) {

		return new double[]{ 770, 20, 8 };
	}

}
