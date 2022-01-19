package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.teleops.TippyBotTeleOp;
import org.firstinspires.ftc.teamcode.utils.GameTimer;
import org.firstinspires.ftc.teamcode.utils.RGBLights;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

@Autonomous
public class TechnovaKillerAuto extends LinearOpMode {

	RRTippyBot robot;

	double wallPos = 64.625;

	@Override
	public void runOpMode( ) throws InterruptedException {
		TippyBotTeleOp.isBlueSide = true;

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new RRTippyBot( this, true );

		robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_MOVING );

		robot.drive.setCameraFrameOfReference( TrackingCameraLocalizer.CardinalDirection.SOUTH );

		robot.barcodeUtil.init( );

		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			robot.drive.update( );
		} while( !isStopRequested( ) && !isStarted( ) && robot.drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		robot.drive.setPoseEstimate( new Pose2d( 17, 64.125, Math.toRadians( 270 ) ) );
		robot.lights.showStatus( RGBLights.StatusLights.WAITING );


		while( opModeIsActive( ) && !isStarted( ) ) {
			robot.drive.update( );
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition( ) );
			telemetry.update( );
		}

		waitForStart( );

		//start timer
		GameTimer.start();
		robot.lights.setPattern( RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE );

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence trajectorySequence = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( RRTippyBot.getHubPosition( -22.5, 270, 10, true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )
				.waitSeconds( 0.8 )
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 38, wallPos ) )
				.lineToConstantHeading( new Vector2d( 38, 38 ) )
				.lineToLinearHeading( new Pose2d( wallPos + 2 , 38, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( wallPos + 2, -36 ) )
				.lineToConstantHeading( new Vector2d( 38, -36 ) )
				.lineToConstantHeading( new Vector2d( 38, -64.125 ) )
				.waitSeconds( 4 )
				.lineToConstantHeading( new Vector2d( 39, -64.125 ) )
				.build();

		robot.drive.followTrajectorySequence( trajectorySequence );

	}
}
