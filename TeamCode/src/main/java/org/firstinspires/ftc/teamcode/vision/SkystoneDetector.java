package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
	Telemetry telemetry;
	Mat mat = new Mat();
	public enum Location {
		LEFT,
		MIDDLE,
		RIGHT,
		NOT_FOUND
	}
	private Location location;

	static final Rect LEFT_ROI = new Rect(
			new Point(0, 0),
			new Point(106, 240));
	static final Rect MIDDLE_ROI = new Rect(
			new Point( 106,0 ),
			new Point( 224, 240 ));
	static final Rect RIGHT_ROI = new Rect(
			new Point(224, 0),
			new Point(320, 240));


	static double PERCENT_COLOR_THRESHOLD = 0.01;

	public SkystoneDetector(Telemetry t) { telemetry = t; }

	public Mat processFrame(Mat input,String type) {
		Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
		Scalar lowHSV;
		Scalar highHSV;

		if(type.equalsIgnoreCase( "duck" )) {
			lowHSV = new Scalar(25, 25, 35);
			highHSV = new Scalar(40, 255, 255);
		}
		else {
			lowHSV = new Scalar(40, 50, 70);
			highHSV = new Scalar(65, 255, 255);
		}
		Core.inRange(mat, lowHSV, highHSV, mat);

		Mat left = mat.submat(LEFT_ROI);
		Mat middle = mat.submat(MIDDLE_ROI);
		Mat right = mat.submat(RIGHT_ROI);

		double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
		double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
		double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

		left.release();
		middle.release();
		right.release();

		/*
		telemetry.addData(type + " Left raw value", (int) Core.sumElems(left).val[0]);
		telemetry.addData(type + " Middle raw value", (int) Core.sumElems(middle).val[0]);
		telemetry.addData(type + " Right raw value", (int) Core.sumElems(right).val[0]);
		telemetry.addData(type + " Left percentage", Math.round(leftValue * 100) + "%");
		telemetry.addData(type + " Middle percentage", Math.round(middleValue * 100) + "%");
		telemetry.addData(type + " Right percentage", Math.round(rightValue * 100) + "%"); */

		boolean leftBool = leftValue > PERCENT_COLOR_THRESHOLD;
		boolean middleBool = middleValue > PERCENT_COLOR_THRESHOLD;
		boolean rightBool = rightValue > PERCENT_COLOR_THRESHOLD;

		if (rightBool) {
			location = Location.RIGHT;
			telemetry.addData("Location", type + " right");
		}
		else if (leftBool) {
			location = Location.LEFT;
			telemetry.addData("Location", type + " left");
		}
		else if (middleBool) {
			location = Location.MIDDLE;
			telemetry.addData( "Location", type + " middle" );
		}
		else {
			location = Location.NOT_FOUND;
			telemetry.addData("Location", type + " not found");
		}
		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

		Scalar elementColor = new Scalar(255, 0, 0);
		Scalar notElement = new Scalar(0, 255, 0);

		Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? notElement:elementColor);
		Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? notElement:elementColor);
		Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? notElement:elementColor);
		return mat;
	}

	@Override
	public Mat processFrame(Mat input) {
		processFrame( input, "element" );
		Mat duck = processFrame( input,"duck" );
		telemetry.update();
		return duck;
	}

	public Location getLocation() {
		return location;
	}
}