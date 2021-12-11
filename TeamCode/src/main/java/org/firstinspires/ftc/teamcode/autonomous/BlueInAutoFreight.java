package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

//@Autonomous
public class BlueInAutoFreight extends LinearOpMode {
	RRHexBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new RRHexBot( this );

		robot.barcodeUtil.init( );

		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			robot.drive.update( );
		} while( !isStopRequested( ) && !isStarted( ) && robot.drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

		while( !isStopRequested( ) && !isStarted( ) ) {
			robot.drive.update( );
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition() );
			telemetry.addLine(  );
			telemetry.addData( "Current pose estimate", robot.drive.getPoseEstimate() );
			telemetry.addLine(  );
			telemetry.addLine( "Pose estimate should be somewhere close to (0, 0, 0). If not, run the \"ResetCamera\" Autonomous." );
			telemetry.update();
		}

		waitForStart( );

		Localizer localizer = robot.drive.getLocalizer();
		robot.drive.setLocalizer( new MecanumDrive.MecanumLocalizer( robot.drive ) );


		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence cameraFix = robot.getTrajectorySequenceBuilder()
				.forward( 6 )
				.back( 6 )
				.build();
		robot.drive.followTrajectorySequence( cameraFix );
		robot.drive.setLocalizer( localizer );
		robot.drive.setPoseEstimate( new Pose2d( 5.625, 62.1875, Math.toRadians( -90 ) ) );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder()
				//move element out of the way
				.lineToLinearHeading( new Pose2d( barcodePosition == BarcodePositionDetector.BarcodePosition.LEFT ? 10 : barcodePosition == BarcodePositionDetector.BarcodePosition.RIGHT ? 8 : 10 , 48, Math.toRadians( 180 ) ) )
				.addTemporalMarker( () -> {
					robot.liftToShippingHubHeight( height );
				} )
				.strafeLeft( 30 )
				//Drop block in shipping hub
				.lineToConstantHeading( new Vector2d( 10, 24 ) )
				.lineToConstantHeading( new Vector2d( -12 + robot.distanceFromShippingHub( height ), 24 ))
				.addTemporalMarker( () -> {
					robot.dumpBucket();
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.lineToConstantHeading( new Vector2d( 10, 24 ) )

				//Drive over bumps
				.lineToConstantHeading( new Vector2d( 10, 40 ) )
				.lineToConstantHeading( new Vector2d( 40, 40 ) )
				//Pick up freight
				.addTemporalMarker( () -> {
					robot.intake.setPower( -0.6 );
				} )
				.lineToLinearHeading( new Pose2d( 48, 48, Math.toRadians( 225 ) ) )
				.waitSeconds( 2 )
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0 );
				} )
				//Drive over bumps
				.lineToLinearHeading( new Pose2d( 40, 40, Math.toRadians( 180 ) ) )
				.lineToConstantHeading( new Vector2d( 10, 40 ) )
				//Drop freight in shipping hub
				.addTemporalMarker( () -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
				} )
				.lineToConstantHeading( new Vector2d( 10, 24 ) )
				.lineToConstantHeading( new Vector2d( -12 + robot.distanceFromShippingHub( RRHexBot.ShippingHubHeight.LOW ), 24 ))
				.addTemporalMarker( () -> {
					robot.dumpBucket();
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.lineToConstantHeading( new Vector2d( 10, 24 ) )

				//Drive over bumps
				.lineToConstantHeading( new Vector2d( 10, 40 ) )
				.lineToConstantHeading( new Vector2d( 40, 40 ) )
				//Park
				.splineTo( new Vector2d( 60, -44 ), 0 )

				.build();

		robot.drive.followTrajectorySequence( mainTrajectory );
	}
}
