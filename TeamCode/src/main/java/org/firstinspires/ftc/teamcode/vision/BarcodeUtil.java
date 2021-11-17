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

	public BarcodeUtil( HardwareMap hardwareMap, String webcamName, Telemetry telemetry ) {
		this.telemetry = telemetry;
		setup( hardwareMap, webcamName );
	}

	public void setup( HardwareMap hardwareMap, String webcamName ) {

		int cameraMonitorViewId = hardwareMap.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName( ) );
		webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hardwareMap.get( WebcamName.class, webcamName ), cameraMonitorViewId );
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
				webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
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

	public void stopCamera( ) {
		webcam.stopStreaming( );
	}
}
