package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.opencv.core.Point;

import Jama.Matrix;

/**
 * This class contains the variables and methods to be able to calculate a 3D position from a 2D image pixel.
 * Matrices need to be re-calculated if the camera is moved on the robot.
 */
public class HomographyTargetDistance {

	//Value from solvePnP
	private static final Matrix TRANSLATION_VECTOR = new Matrix( new double[][]{
			{ -38.42787084 },
			{ 14.09083823 },
			{ 653.22310629 }
	} );
	//Value after applying Rodriguez rotation formula to rotation vector from solvePnP
	private static final Matrix ROTATION_MATRIX = new Matrix( new double[][]{
			{ -0.99915473, -0.03914632, -0.01254565 },
			{ 0.0015009, -0.33972784, 0.94052259 },
			{ -0.04108011, 0.93970876, 0.33949943 }
	} );
	//Value from camera calibration
	private static final Matrix CAMERA_MATRIX = new Matrix( new double[][]{
			{ 1.42116404e+03, 0.00000000e+00, 7.17082278e+02 },
			{ 0.00000000e+00, 1.44515847e+03, 3.86650346e+02 },
			{ 0.00000000e+00, 0.00000000e+00, 1.00000000e+00 }
	} );
	//Z axis is always 1 since all the objects are on the ground
	final static double Z_CONST = 1;


	/**
	 * Calculates the position of the corresponding point in 3D space from a camera point
	 *
	 * @param point camera point at which to find the corresponding 3D point
	 * @return point's position relative to the camera
	 */
	public static Vector2d positionFromPoint( Point point ) {

		//Change point into a matrix
		Matrix pointMatrix = new Matrix( new double[][]{
				{ point.x },
				{ point.y },
				{ 1 }
		} );

		//Calculating scalar value
		Matrix leftSideMatrix = ROTATION_MATRIX.inverse( ).times( CAMERA_MATRIX.inverse( ) ).times( pointMatrix );
		Matrix rightSideMatrix = ROTATION_MATRIX.inverse( ).times( TRANSLATION_VECTOR );
		double scalar = (Z_CONST + rightSideMatrix.get( 2, 0 )) / leftSideMatrix.get( 2, 0 );

		//(x,y) position in mm
		Matrix calculatedPosition = ROTATION_MATRIX.inverse( ).times( CAMERA_MATRIX.inverse( ).times( pointMatrix ).times( scalar ).minus( TRANSLATION_VECTOR ) );

		//Divide by 25.4 to get inches from mm
		calculatedPosition = calculatedPosition.times( 1 / 25.4 );

		//Swap X and Y, and add 24 to X to account for camera coordinate system
		double x = calculatedPosition.get( 0, 0 );
		double y = calculatedPosition.get( 1, 0 );
		return new Vector2d( y /*+ 24*/, x );
	}

}
