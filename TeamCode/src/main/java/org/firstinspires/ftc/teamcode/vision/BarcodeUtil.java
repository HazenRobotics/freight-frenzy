package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class BarcodeUtil {

	Telemetry telemetry;
	private OpenCvWebcam webcam;
	private BarcodePositionDetector pipeline;

	public BarcodeUtil( HardwareMap hw, String webcamName, Telemetry t ) {
		telemetry = t;
		setup( hw, webcamName );
	}

	public void setup( HardwareMap hw, String webcamName ) {

		int cameraMonitorViewId = hw.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hw.appContext.getPackageName( ) );
		webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hw.get( WebcamName.class, webcamName ), cameraMonitorViewId );
		pipeline = new BarcodePositionDetector( telemetry );
		webcam.setPipeline( pipeline );
	}

	public void init( ) {
		openCameraDevice( );
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

	public BarcodePositionDetector.BarcodePosition getBarcodePosition( ) {
		return pipeline.getBarcodePosition( );
	}
}
