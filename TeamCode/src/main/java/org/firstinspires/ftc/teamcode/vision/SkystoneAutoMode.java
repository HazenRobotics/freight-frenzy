package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Skystone Detector", group="Auto")
public class SkystoneAutoMode extends LinearOpMode {
	OpenCvCamera webcam;

	@Override
	public void runOpMode() throws InterruptedException {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hardwareMap.get( WebcamName.class, "webcam" ), cameraMonitorViewId );
		SkystoneDetector detector = new SkystoneDetector(telemetry);
		webcam.setPipeline(detector);
		webcam.openCameraDeviceAsync(
				new OpenCvCamera.AsyncCameraOpenListener( ) {
					@Override
					public void onOpened( ) {
						webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
		webcam.stopStreaming();
	}
}