package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.HomographyTargetDistance;
import org.opencv.core.Point;

public class TargetPositionCalculator {

	private final Vector2d CAMERA_OFFSET;

	public TargetPositionCalculator( Vector2d cameraOffset ) {
		CAMERA_OFFSET = cameraOffset;
	}

	/**
	 * Gets the position of a tensorflow recognition on the field
	 *
	 * @param target     tensorflow recognition to find position of
	 * @param robotAngle angle of the robot relative to 0, in Radians
	 * @return field position of the target
	 */
	public Vector2d getTargetPosition( Recognition target, double robotAngle ) {

		//Calculate point at the bottom center of the bounding box
		double x = (target.getLeft( ) + (target.getWidth( ) / 2));
		double y = target.getBottom( );
		Point targetBasePoint = new Point( x, y );

		//Calculate 3D position from point
		Vector2d positionFromCamera = HomographyTargetDistance.positionFromPoint( targetBasePoint );

		//Apply camera offset to get position from center of robot
		Vector2d positionFromRobot = positionFromCamera.minus( CAMERA_OFFSET );

		//Calculate field coordinates (no idea if this actually works)
		double targetX = positionFromRobot.getX( ) * Math.cos( robotAngle ) + positionFromRobot.getY( ) * Math.sin( robotAngle );
		double targetY = positionFromRobot.getY( ) * Math.cos( robotAngle ) + positionFromRobot.getX( ) * Math.sin( robotAngle );

		return new Vector2d( -targetX, -targetY );

	}


}
