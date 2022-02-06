package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;
import org.firstinspires.ftc.teamcode.vision.HomographyTargetDistance;
import org.opencv.core.Point;

public class TargetPositionCalculator {

	private final Pose2d CAMERA_OFFSET;

	public TargetPositionCalculator( Pose2d cameraOffset ) {
		CAMERA_OFFSET = cameraOffset;
	}

	/**
	 * Gets the position of a tensorflow recognition on the field
	 *
	 * @param target     tensorflow recognition to find position of
	 * @param robotAngle angle of the robot relative to 0, in Radians
	 * @return field position of the target
	 */
	public Vector2d getTargetPosition( @NonNull Classifier.Recognition target, double robotAngle ) {

		//Calculate point at the bottom center of the bounding box
		double x = target.getLocation().centerX();
		double y = target.getLocation().bottom;
		Point targetBasePoint = new Point( x, y );

		//Calculate 3D position from point
		Vector2d positionFromCamera = HomographyTargetDistance.positionFromPoint( targetBasePoint );

		//Apply camera offset to get position from center of robot
		Vector2d positionFromRobot = positionFromCamera.minus( CAMERA_OFFSET.vec() );

		//Calculate field coordinates (no idea if this actually works)
		double targetX = positionFromRobot.getX( ) * Math.cos( robotAngle + CAMERA_OFFSET.getHeading() ) - positionFromRobot.getY( ) * Math.sin( robotAngle + CAMERA_OFFSET.getHeading() );
		double targetY = positionFromRobot.getY( ) * Math.cos( robotAngle + CAMERA_OFFSET.getHeading() ) + positionFromRobot.getX( ) * Math.sin( robotAngle + CAMERA_OFFSET.getHeading() );

		return new Vector2d( targetX, targetY );

	}

}
