package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;

@Autonomous
public class ResetCamera extends LinearOpMode {
	RRMecanumDriveTippy42 drive;

	@Override
	public void runOpMode( ) throws InterruptedException {
		drive = new RRMecanumDriveTippy42( hardwareMap );
		telemetry.addLine( "This will reset the robot controller app. Continue?" );
		waitForStart();
		new TrackingCameraLocalizer(hardwareMap, new Pose2d(  ), drive ).resetCamera();
		System.exit( 0 );
	}
}
