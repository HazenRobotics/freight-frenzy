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
			{78.08870033},
			{110.767108},
			{397.27562884}
	} );
	//Value after applying Rodriguez rotation formula to rotation vector from solvePnP
	private static final Matrix ROTATION_MATRIX = new Matrix( new double[][] {
			{-9.99849474e-01, -1.73249749e-02,  9.34937086e-04},
			{6.96330302e-03, -3.51339381e-01,  9.36222277e-01},
			{-1.58915472e-02,  9.36087862e-01,  3.51407134e-0}
	} );
	//Value from camera calibration
	private static final Matrix CAMERA_MATRIX = new Matrix( new double[][] {
			{823.41290477,   0.0,         359.87721122},
			{0.0,         825.66798289, 210.74873307},
			{0.0, 0.0, 1.0}
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
		return new Vector2d( y + 12.9, x - 4.1);
	}

}
