package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.ejml.data.MatrixType;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;

public class TensorFlowPositionDetection {

	Mat homographyMatrix;

	public TensorFlowPositionDetection() {

		//homographyMatrix = Calib3d.findHomography();

		//new Mat( 1, 3, MatrixType.CDRM.getBits() );
	}





	public Vector2d getObjectPosition( Recognition object, double robotAngle ) {
		float bottom = object.getBottom();

		//map bottom to robot coordinates
		Vector2d objectFromRobot = new Vector2d(  );

		//change robot coordinates to field coordinates
		double objX = objectFromRobot.getX( ) * Math.cos( -robotAngle ) + objectFromRobot.getY() * Math.sin( -robotAngle );
		double objY = objectFromRobot.getY() * Math.cos( -robotAngle ) + objectFromRobot.getX() * Math.sin( -robotAngle );

		return new Vector2d( objX, objY );
	}
}
