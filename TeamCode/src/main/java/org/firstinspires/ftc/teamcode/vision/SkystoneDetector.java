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
			new Point(80, 320));
	static final Rect MIDDLE_ROI = new Rect(
			new Point( 80,0 ),
			new Point( 160, 320 ));
	static final Rect RIGHT_ROI = new Rect(
			new Point(160, 0),
			new Point(240, 320));


	static double PERCENT_COLOR_THRESHOLD = 0.005;

	public SkystoneDetector(Telemetry t) { telemetry = t; }

	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
		//Scalar lowHSV = new Scalar(40, 50, 70);
		//Scalar highHSV = new Scalar(65, 255, 255);
		Scalar lowHSV = new Scalar(25, 25, 35);
		Scalar highHSV = new Scalar(40, 255, 255);

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

		telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
		telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
		telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
		telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
		telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
		telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

		boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;
		boolean elementMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
		boolean elementRight = rightValue > PERCENT_COLOR_THRESHOLD;

		if (!elementLeft && !elementMiddle && !elementRight) {
			location = Location.NOT_FOUND;
			telemetry.addData("Element Location", "not found");
		}
		else if (elementLeft) {
			location = Location.LEFT;
			telemetry.addData("Element Location", "left");
		}
		else if (elementMiddle) {
			location = Location.MIDDLE;
			telemetry.addData("Element Location", "middle");
		}
		else {
			location = Location.RIGHT;
			telemetry.addData("Element Location", "right");
		}
		telemetry.update();

		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

		Scalar elementColor = new Scalar(255, 0, 0);
		Scalar notElement = new Scalar(0, 255, 0);

		Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? notElement:elementColor);
		Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? notElement:elementColor);
		Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? notElement:elementColor);

		return mat;
	}

	public Location getLocation() {
		return location;
	}
}