package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHexWood42;
import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;

@TeleOp(name = "Drive Test", group = "Test")
public class RRDriveTest extends OpMode {

	RRMecanumDriveHexWood42 drive;

	@Override
	public void init( ) {

		drive = new RRMecanumDriveHexWood42( hardwareMap );
	}

	@Override
	public void init_loop() {
		if(drive.getPoseConfidence() == null || drive.getPoseConfidence().compareTo( T265Camera.PoseConfidence.Low ) < 0 ) {
			telemetry.addLine( "Getting pose from camera. Please wait....." );
		} else {
			telemetry.addLine("Ready!" );
		}
		telemetry.update();
	}

	@Override
	public void loop( ) {

		drive.setDrivePower( new Pose2d( -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x ) );
		drive.update();

		telemetry.addData( "Pose Estimate", drive.getPoseEstimate( ) );
		telemetry.addData( "Pose Confidence",  drive.getPoseConfidence());
		telemetry.update( );
	}

	@Override
	public void stop() {
		drive.stopCamera();
	}
}
