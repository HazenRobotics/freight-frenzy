package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHexWood42;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp(name = "Drive Test", group = "Test")
public class RRDriveTest extends OpMode {

	RRMecanumDriveHexWood42 drive;

	@Override
	public void init( ) {
		drive = new RRMecanumDriveHexWood42( hardwareMap );


	}

	@Override
	public void loop( ) {
		drive.setDrivePower( new Pose2d( -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x ) );

		telemetry.addData( "Pose Estimate", drive.getPoseEstimate());
		telemetry.update();
	}

	@Override
	public void stop() {
		drive.freeCamera();
	}
}
