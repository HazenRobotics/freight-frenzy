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
	private static final Matrix TRANSLATION_VECTOR = new Matrix( new double[][] {
			{-20.97044681},
			{59.12198531},
			{531.10257189}
	} );
	//Value after applying Rodriguez rotation formula to rotation vector from solvePnP
	private static final Matrix ROTATION_MATRIX = new Matrix( new double[][] {
			{-0.99927905, -0.03767002,  0.00472714},
			{0.0174136,  -0.34413014,  0.93876047},
			{-0.03373638,  0.93816599,  0.34453801}
	} );
	//Value from camera calibration
	private static final Matrix CAMERA_MATRIX = new Matrix( new double[][] {
			{813.87098059,       0,      360.18166264},
			{0,            815.10883372, 228.27266263},
			{0,               0,               1}
	} );
	//Z axis is always 1 since all the objects are on the ground
	final static double Z_CONST = 1;


	/**
	 * Calculates the position of the corresponding point in 3D space from a camera point
	 * @param point camera point at which to find the corresponding 3D point
	 * @return point's position relative to the camera
	 */
	public static Vector2d positionFromPoint( Point point ) {

		//Change point into a matrix
		Matrix pointMatrix = new Matrix( new double[][] {
				{point.x},
				{point.y},
				{1}
		} );

		//Calculating scalar value
		Matrix leftSideMatrix = ROTATION_MATRIX.inverse().times( CAMERA_MATRIX.inverse() ).times(pointMatrix);
		Matrix rightSideMatrix = ROTATION_MATRIX.inverse().times( TRANSLATION_VECTOR );
		double scalar = (Z_CONST + rightSideMatrix.get( 2, 0 )) / leftSideMatrix.get( 2, 0 );

		//(x,y) position in mm
		Matrix calculatedPosition = ROTATION_MATRIX.inverse().times( CAMERA_MATRIX.inverse().times( pointMatrix ).times( scalar ).minus( TRANSLATION_VECTOR ) );

		//Divide by 25.4 to get inches from mm
		calculatedPosition = calculatedPosition.times( 1 / 25.4 );

		//Swap X and Y, and add 24 to X to account for camera coordinate system
		double x = calculatedPosition.get( 0, 0 );
		double y = calculatedPosition.get( 1, 0 );
		return new Vector2d( y + 18.5, x );
	}

}
