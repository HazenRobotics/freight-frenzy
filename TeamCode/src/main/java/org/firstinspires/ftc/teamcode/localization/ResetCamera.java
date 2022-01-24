package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.RRHexBot;

@Autonomous
public class ResetCamera extends LinearOpMode {
	RRHexBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new RRHexBot( this );
		telemetry.addLine( "This will reset the robot controller app. Continue?" );
		waitForStart();
		new TrackingCameraLocalizer(hardwareMap, new Pose2d(  ) ).resetCamera();
		System.exit( 0 );
	}
}
