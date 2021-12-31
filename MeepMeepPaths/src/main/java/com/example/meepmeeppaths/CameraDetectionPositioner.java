package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CameraDetectionPositioner {

	public static Vector2d duckMin = new Vector2d( 3.25, 720 ); // fullscreen image
	public static Vector2d duckMax = new Vector2d( 50, 58 ); // image size at 50 inches
	public static double duckSize = 1.875;

	public static Vector2d blockMin = new Vector2d( 3.75, 720 ); // fullscreen image
	public static Vector2d blockMax = new Vector2d( 50, 56.5 ); // image size at 50 inches
	public static double blockSize = 1.95;

	public static Vector2d ballMin = new Vector2d( -1, 720 ); // fullscreen image
	public static Vector2d ballMax = new Vector2d( 50, -1 ); // image size at 50 inches
	public static double ballSize = 2.75;

	/*

	first calcs: https://www.desmos.com/calculator/qps2aikmjz
	second calcs: https://www.desmos.com/calculator/vf8nov8gyr

	 */

	enum ObjectType {
		DUCK,
		BLOCK,
		BALL
	}

	public static void main( String[] args ) {
//65 x 70
		double distance = distanceFromTarget( 65, 70, ObjectType.DUCK, 22.5 );

		System.out.println( "Distance (from target): " + distance );

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
				return distanceFromRatios( pixelAvg, duckMin, duckMax );
			case BLOCK:
				return distanceFromRatios( pixelAvg, blockMin, blockMax );
			case BALL:
				return distanceFromRatios( pixelAvg, ballMin, ballMax );
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
