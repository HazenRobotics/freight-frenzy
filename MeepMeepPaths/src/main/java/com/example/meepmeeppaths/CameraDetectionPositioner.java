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

	public static double duckSize = 1.875; // (3.25, 720) (50, 58)
	public static double blockSize = 1.95; // (3.75, 720) (50, 56.5)
	public static double ballSize = 2.75; //  (9.08, 720) (50, 40.5) // test to see if these numbers are correct

	enum ObjectType {
		DUCK,
		BLOCK,
		BALL
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

	public static void main( String[] args ) {

		double distance = distanceFromTarget( 65, 70, ObjectType.DUCK, 22.5 );

		System.out.println( "Distance (from target): " + distance );

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
	 * @param width      the width of the target object (pixels)
	 * @param height     the height of the target object (pixels)
	 * @param objectType the type of the target object (ObjectType)
	 * @param camAngle   the vertical angle of camera (degrees)
	 * @return how far the robot is from the object (inches)
	 */
	public static double distanceFromTarget( double width, double height, ObjectType objectType, double camAngle ) {
		double avgSize = (width + height) / 2;

		double distanceFromCam = distanceFromObjectType( avgSize, objectType );

		System.out.println( "Distance (from camera): " + distanceFromCam );

		return distanceFromCam * Math.cos( Math.toRadians( camAngle ) );
	}

	public static double distanceFromObjectType( double pixelAvg, ObjectType objectType ) {

		switch( objectType ) {
			default: // (DUCK)
				return distanceFromRatios( pixelAvg, getMinPair( duckSize ), getMaxPair( duckSize ) );
//				return distanceFromRatios( pixelAvg, duckMin, duckMax );
			case BLOCK:
				return distanceFromRatios( pixelAvg, getMinPair( blockSize ), getMaxPair( blockSize ) );
//				return distanceFromRatios( pixelAvg, blockMin, blockMax );
			case BALL:
				return distanceFromRatios( pixelAvg, getMinPair( ballSize ), getMaxPair( ballSize ) );
//				return distanceFromRatios( pixelAvg, ballMin, ballMax );
		}
	}

	public static double distanceFromRatios( double pixelAvg, Vector2d min, Vector2d max ) {

		double m = (max.getY( ) - min.getY( )) / (max.getX( ) - min.getX( ));
		double b = max.getY( ) - m * max.getX( );

		// y = mx + b
		// x = (y - b)/m
		return (pixelAvg - b) / m;
	}


	/**
	 * finds the direction a target is from the camera
	 *
	 * @param x        the x position of the target object (pixels)
	 * @param camWidth the width of the camera (pixels)
	 * @param camFOV   the field of view of the camera (degrees): normally 55°
	 * @return the direction the target is from the robot
	 */
	public static double directionFromPosition( double x, double camWidth, double camFOV ) {

		double xOffset = camWidth / 2 + x;

		return xOffset * camFOV / camWidth;
	}


}
