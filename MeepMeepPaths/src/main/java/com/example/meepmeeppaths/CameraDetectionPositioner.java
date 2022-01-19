package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CameraDetectionPositioner {

//    public static Vector2d duckMin = new Vector2d( 3.25, 720 ); // fullscreen image
//    public static Vector2d duckMax = new Vector2d( 50, 58 ); // image size at 50 inches
//
//    public static Vector2d blockMin = new Vector2d( 3.75, 720 ); // fullscreen image
//    public static Vector2d blockMax = new Vector2d( 50, 56.5 ); // image size at 50 inches
//
//    public static Vector2d ballMin = new Vector2d( -1, 720 ); // fullscreen image
//    public static Vector2d ballMax = new Vector2d( 50, -1); // image size at 50 inches

	public static final double duckSize = 1.875; // (3.25, 720) (50, 58)
	public static final double blockSize = 1.95; // (3.75, 720) (50, 56.5)
	public static final double ballSize = 2.75; //  (9.08, 720) (50, 40.5) // test to see if these numbers are correct

	// Object Dimensions - width, height
	/**
	 * the dimensions of a Ball in inches (width, height)
	 */
	public static final Vector2d BALL_DIM = new Vector2d( 2.75, 2.75 );
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



	/*

	short distances
	2.00
	2.25
	2.50


	first: https://www.desmos.com/calculator/xjkt5xjnep
	second: https://www.desmos.com/calculator/ftm6v22gvt


	 */


	enum ObjectType {
		BALL,
		DUCK,
		ELEMENT,
		HEAVY,
		LIGHT,
		MEDIUM

	}

	public static void main( String[] args ) {

		/*

		use scale to get complete distance
		use camera angle to get distance from robot
		use center offset to get lateral error (turning offset)
		turn the lateral error (turning offset) into an angle and get the x and y components for moving

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


//		System.out.println( );
//
//		System.out.println( "duckSize min: " + getMinPair( duckSize ) );
//		System.out.println( "blockSize min: " + getMinPair( blockSize ) );
//		System.out.println( "ballSize min: " + getMinPair( ballSize ) );
//
//		System.out.println( );
//
//		System.out.println( "duckSize max: " + getMaxPair( duckSize ) );
//		System.out.println( "blockSize max: " + getMaxPair( blockSize ) );
//		System.out.println( "ballSize max: " + getMaxPair( ballSize ) );


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

		double avgSize = (width + height) / 2;

		double distanceFromCam = distanceFromObjectType( avgSize, objectType );

//		System.out.println( "Distance (from camera): " + distanceFromCam );

		return distanceFromCam;
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

	public static double distanceFromObjectType( double pixelAvg, ObjectType objectType ) {

		switch( objectType ) {
			default: // (DUCK)
				return distanceFromRatios( pixelAvg, getMinPair( DUCK_DIM.getX( ) ), getMaxPair( DUCK_DIM.getX( ) ) );
			case BALL:
				return distanceFromRatios( pixelAvg, getMinPair( BALL_DIM.getX( ) ), getMaxPair( BALL_DIM.getX( ) ) );
			case ELEMENT:
				return distanceFromRatios( pixelAvg, getMinPair( ELEMENT_DIM.getX( ) ), getMaxPair( ELEMENT_DIM.getX( ) ) );
			case HEAVY:
				return distanceFromRatios( pixelAvg, getMinPair( HEAVY_DIM.getX( ) ), getMaxPair( HEAVY_DIM.getX( ) ) );
			case LIGHT:
				return distanceFromRatios( pixelAvg, getMinPair( LIGHT_DIM.getX( ) ), getMaxPair( LIGHT_DIM.getX( ) ) );
			case MEDIUM:
				return distanceFromRatios( pixelAvg, getMinPair( MEDIUM_DIM.getX( ) ), getMaxPair( MEDIUM_DIM.getX( ) ) );
		}
	}

	/**
	 * @param objectSize the size (avg of width & height) of the target object (inches)
	 * @return a Vector2d pair of: the target object's distance away (when it takes up the full camera screen) (inches)
	 * and the camera resolution's height (pixels)
	 */
	public static Vector2d getMinPair( double objectSize ) { // object: (some # of inches away, 720 pixels)
		// static example variables - ( block size, (720 pixels) distance from camera )
		Vector2d duck = new Vector2d( 1.875, 3.25 );
		Vector2d block = new Vector2d( 1.95, 3.75 );

		double m = (block.getY( ) - duck.getY( )) / (block.getX( ) - duck.getX( ));
		double b = block.getY( ) - block.getX( ) * m;
		return new Vector2d( objectSize * m + b /*inches*/, 720 /*pixels*/ );
	}

	/**
	 * @param objectSize the size (avg of width & height) of the target object (inches)
	 * @return a Vector2d pair of: the target object's distance away (50 inches)
	 * and the size (avg of width & height) of the target object (pixels)
	 */
	public static Vector2d getMaxPair( double objectSize ) { // object: (50 inches away, it's on-screen size)
		// static example variables - ( block size, (50 inches) distance from camera )
		Vector2d duck = new Vector2d( 1.875, 58 );
		Vector2d block = new Vector2d( 1.95, 56.5 );

		double m = (block.getY( ) - duck.getY( )) / (block.getX( ) - duck.getX( ));
		double b = block.getY( ) - block.getX( ) * m;
		return new Vector2d( 50 /*inches*/, objectSize * m + b /*pixels*/ );
	}


	public static double distanceFromRatios( double pixelAvg, Vector2d min, Vector2d max ) {

		double m = (max.getY( ) - min.getY( )) / (max.getX( ) - min.getX( ));
		double b = max.getY( ) - m * max.getX( );

		// y = mx + b
		// x = (y - b)/m
		return (pixelAvg - b) / m;
	}


}
