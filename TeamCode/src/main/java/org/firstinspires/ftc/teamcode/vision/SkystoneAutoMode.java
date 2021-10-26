package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Skystone Detector", group="Auto")
public class SkystoneAutoMode extends LinearOpMode {
	OpenCvCamera phoneCam;
	@Override
	public void runOpMode() throws InterruptedException {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam 1", "id", hardwareMap.appContext.getPackageName());
		SkystoneDetector detector = new SkystoneDetector(telemetry);
		phoneCam.setPipeline(detector);
		phoneCam.openCameraDeviceAsync(
				new OpenCvCamera.AsyncCameraOpenListener( ) {
					@Override
					public void onOpened( ) {
						phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
					}

					@Override
					public void onError( int errorCode ) {
					}
				}
		);

		waitForStart();
		switch (detector.getLocation()) {
			case LEFT:
				// ...
				break;
			case MIDDLE:
				//...
				break;
			case RIGHT:
				// ...
				break;
			case NOT_FOUND:
				// ...
		}
		phoneCam.stopStreaming();
	}
}