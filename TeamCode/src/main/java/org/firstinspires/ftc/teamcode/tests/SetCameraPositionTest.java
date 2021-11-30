package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;

@Autonomous
public class SetCameraPositionTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		TrackingCameraLocalizer trackingCameraLocalizer;
		trackingCameraLocalizer = new TrackingCameraLocalizer( hardwareMap, new Pose2d() );
		trackingCameraLocalizer.update();
		while( trackingCameraLocalizer.getPoseConfidence().ordinal() < T265Camera.PoseConfidence.Medium.ordinal() ) {
			telemetry.addLine("Getting Camera Position");
			telemetry.update();
			trackingCameraLocalizer.update();
		}
		while( !isStarted() ) {
			telemetry.addData( "Camera Position" ,trackingCameraLocalizer.getPoseEstimate());
			telemetry.update();
			trackingCameraLocalizer.update();

		}
		waitForStart();
		trackingCameraLocalizer.setPoseEstimate( new Pose2d( 10, 10, Math.PI ) );

		while( !isStopRequested() ) {
			trackingCameraLocalizer.update();
			telemetry.addData( "New Camera Position", trackingCameraLocalizer.getPoseEstimate() );
			telemetry.update();
		}
	}
}
