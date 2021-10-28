package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class BarcodeDetection {

	Telemetry telemetry;
	private OpenCvWebcam webcam;
	private BarcodePipeline pipeline;

	public enum BarcodePosition {
		LEFT,
		MIDDLE,
		RIGHT,
		NOT_FOUND
	}

	public BarcodeDetection( HardwareMap hw, String webcamName, Telemetry t ) {
		setup( hw, webcamName );
		telemetry = t;
	}

	public void setup( HardwareMap hw, String webcamName ) {

		int cameraMonitorViewId = hw.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hw.appContext.getPackageName( ) );
		webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hw.get( WebcamName.class, webcamName ), cameraMonitorViewId );
		pipeline = new BarcodePipeline( telemetry );
		setPipeline( pipeline );
	}

	public void processFrameCapture() {
		//pipeline.processFrame(  )
	}

	public void init( ) {
		openCameraDevice( );
	}

	public void setPipeline( OpenCvPipeline pipeline ) {
		webcam.setPipeline( pipeline );
	}

	public void setTimeoutTime( int milliseconds ) {
		// Timeout for obtaining permission is configurable. Set before opening.
		webcam.setMillisecondsPermissionTimeout( milliseconds );
	}

	public void openCameraDevice( ) {

		webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
			@Override
			public void onOpened( ) {
				webcam.startStreaming( 320, 240, OpenCvCameraRotation.UPRIGHT );
			}

			@Override
			public void onError( int errorCode ) {
				//This will be called if the camera could not be opened
				Log.e( "CAMERA_DEVICE", "Camera could not be opened. Error code: " + errorCode );
			}
		} );
	}

	public BarcodePosition getAnalysis( ) {
		return pipeline.getAnalysis( );
	}


	public static class BarcodePipeline extends OpenCvPipeline {

		double PERCENT_COLOR_THRESHOLD = 0.005;
		Mat mat = new Mat();
		BarcodePosition position;
		Telemetry telemetry;

		public BarcodePipeline(Telemetry t) {
			telemetry = t;
		}

		/*
		 * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
		 * highly recommended to declare them here as instance variables and re-use them for
		 * each invocation of processFrame(), rather than declaring them as new local variables
		 * each time through processFrame(). This removes the danger of causing a memory leak
		 * by forgetting to call mat.release(), and it also reduces memory pressure by not
		 * constantly allocating and freeing large chunks of memory.
		 */

		@Override
		public Mat processFrame( Mat input ) {
			/*
			 * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
			 * will only dereference to the same image for the duration of this particular
			 * invocation of this method. That is, if for some reason you'd like to save a copy
			 * of this particular frame for later use, you will need to either clone it or copy
			 * it to another Mat.
			 */

			/*
			 * Draw a simple box around the middle 1/2 of the entire frame
			 */
			final Rect LEFT_ROI = new Rect(
					new Point(0, 0),
					new Point(80, 240));
			final Rect MIDDLE_ROI = new Rect(
					new Point( 80,0 ),
					new Point( 160, 240 ));
			final Rect RIGHT_ROI = new Rect(
					new Point(160, 0),
					new Point(240, 240));
			// TODO: figure out where duck is here

			/**
			 * NOTE: to see how to get data from your pipeline to your OpMode as well as how
			 * to change which stage of the pipeline is rendered to the viewport when it is
			 * tapped, please see {@link PipelineStageSwitchingExample}
			 */
			Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
			//duck colors
			//Scalar lowHSV = new Scalar(40, 50, 70);
			//Scalar highHSV = new Scalar(65, 255, 255);

			//shipping hub colors
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


			boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;
			boolean elementMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
			boolean elementRight = rightValue > PERCENT_COLOR_THRESHOLD;

			if (!elementLeft && !elementMiddle && !elementRight) {
				position = BarcodePosition.NOT_FOUND;
			}
			else if (elementLeft) {
				position = BarcodePosition.LEFT;
			}
			else if (elementMiddle) {
				position = BarcodePosition.MIDDLE;
			}
			else {
				position = BarcodePosition.RIGHT;
			}

			telemetry.addLine( "" + position );

			Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

			Scalar elementColor = new Scalar(255, 0, 0);
			Scalar notElement = new Scalar(0, 255, 0);

			Imgproc.rectangle(mat, LEFT_ROI, position == BarcodePosition.LEFT? notElement:elementColor);
			Imgproc.rectangle(mat, RIGHT_ROI, position == BarcodePosition.RIGHT? notElement:elementColor);
			Imgproc.rectangle(mat, MIDDLE_ROI, position == BarcodePosition.MIDDLE? notElement:elementColor);


			return input;

		}


		public BarcodePosition getAnalysis( ) {
			return position;
		}
	}
}
