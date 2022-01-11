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
			{-40.06944102},
			{113.57229532},
			{655.72896487}
	} );
	//Value after applying Rodriguez rotation formula to rotation vector from solvePnP
	private static final Matrix ROTATION_MATRIX = new Matrix( new double[][] {
			{-0.99979639, -0.0197295,   0.00423448},
			{0.00823028, -0.20710507,  0.97828409},
			{-0.01842407,  0.97811975,  0.20722528}
	} );
	//Value from camera calibration
	private static final Matrix CAMERA_MATRIX = new Matrix( new double[][] {
			{1.42712092e+03, 0.00000000e+00, 6.92836355e+02},
			{0.00000000e+00, 1.43303450e+03, 3.28652617e+02},
			{0.00000000e+00, 0.00000000e+00, 1.00000000e+00}
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
		return new Vector2d( y + 24, x );
	}

}
