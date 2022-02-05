package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.TwoWheelTrackingLocalizerTippy;
import org.firstinspires.ftc.teamcode.localization.DistanceSensorLocalizer;
import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp
public class DistanceSensorLogging extends LinearOpMode {
	RRTippyBot robot;
	DistanceSensorLocalizer distanceSensorLocalizer;
	TrackingCameraLocalizer trackingCameraLocalizer;
	TwoWheelTrackingLocalizerTippy deadWheelLocalizer;
	MecanumDrive.MecanumLocalizer driveEncoderLocalizer;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new RRTippyBot( this, true );
		distanceSensorLocalizer = robot.distanceSensorLocalizer;
		trackingCameraLocalizer = new TrackingCameraLocalizer( hardwareMap, new Pose2d(  ) );
		deadWheelLocalizer = new TwoWheelTrackingLocalizerTippy( hardwareMap, robot.drive );
		driveEncoderLocalizer = (MecanumDrive.MecanumLocalizer) robot.drive.getLocalizer();
		Logger.writeAFile( "DistanceSensors.csv", "x, y", true, false);
		Logger.writeAFile( "TrackingCamera.csv", "x, y", true, false);
		Logger.writeAFile( "DeadWheels.csv", "x, y", true, false);
		Logger.writeAFile( "DriveEncoder.csv", "x, y", true, false);

		waitForStart();

		distanceSensorLocalizer.setPoseEstimate( new Pose2d( 48, 48, Math.toRadians( 180 ) ) );
		trackingCameraLocalizer.setPoseEstimate( new Pose2d( 48, 48, Math.toRadians( 180 ) ) );
		deadWheelLocalizer.setPoseEstimate( new Pose2d( 48, 48, Math.toRadians( 180 ) ) );
		driveEncoderLocalizer.setPoseEstimate( new Pose2d( 48, 48, Math.toRadians( 180 ) ) );

		new Thread( () -> {
			while( opModeIsActive() && !isStopRequested() ) {
				robot.mecanumDrive.drive( gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2 );
			}
		} ).start();

		for(int i = 0; i < 50; i++) {
			distanceSensorLocalizer.update();
			trackingCameraLocalizer.update();
			deadWheelLocalizer.update();
			driveEncoderLocalizer.update();
			Logger.writeAFile( "DistanceSensors.csv", distanceSensorLocalizer.getPoseEstimate().getX() + ", " + distanceSensorLocalizer.getPoseEstimate().getY(), true, false);
			Logger.writeAFile( "TrackingCamera.csv", trackingCameraLocalizer.getPoseEstimate().getX() + ", " + trackingCameraLocalizer.getPoseEstimate().getY(), true, false);
			Logger.writeAFile( "DeadWheels.csv", deadWheelLocalizer.getPoseEstimate().getX() + ", " + deadWheelLocalizer.getPoseEstimate().getY(), true, false);
			Logger.writeAFile( "DriveEncoder.csv", driveEncoderLocalizer.getPoseEstimate().getX() + ", " + driveEncoderLocalizer.getPoseEstimate().getY(), true, false);
			robot.sleepRobot( 0.2 );
		}
	}
}
