package org.firstinspires.ftc.teamcode.vision.unused;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Webcam {

	private OpenCvWebcam webcam;
	private BarcodePipeline pipeline;

	public enum BarcodePosition {
		LEFT,
		MIDDLE,
		RIGHT
	}

	public Webcam( HardwareMap hw ) {
		this( hw, "webcam" );
	}

	public Webcam( HardwareMap hw, String webcamName ) {
		setup( hw, webcamName );
	}

	public void setup( HardwareMap hw, String webcamName ) {

		int cameraMonitorViewId = hw.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hw.appContext.getPackageName( ) );
		webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hw.get( WebcamName.class, webcamName ), cameraMonitorViewId );
		pipeline = new BarcodePipeline( );
		setPipeline( pipeline );
	}

	public void init( ) {
		setTimeoutTime( 2500 );
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

		BarcodePosition position;

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
			Imgproc.rectangle(
					input,
					new Point(
							input.cols( ) / 4,
							input.rows( ) / 4 ),
					new Point(
							input.cols( ) * (3f / 4f),
							input.rows( ) * (3f / 4f) ),
					new Scalar( 0, 255, 0 ), 4 );

			// TODO: figure out where duck is here

			/**
			 * NOTE: to see how to get data from your pipeline to your OpMode as well as how
			 * to change which stage of the pipeline is rendered to the viewport when it is
			 * tapped, please see {@link PipelineStageSwitchingExample}
			 */

			return input;
		}


		public BarcodePosition getAnalysis( ) {
			return position;
		}
	}
}
