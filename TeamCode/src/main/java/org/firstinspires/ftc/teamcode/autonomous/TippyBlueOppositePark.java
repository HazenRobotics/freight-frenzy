package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.localization.DistanceSensorLocalizer;
import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.teleops.TippyBotTeleOp;
import org.firstinspires.ftc.teamcode.utils.GameTimer;
import org.firstinspires.ftc.teamcode.utils.RGBLights;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

import java.util.function.Supplier;

public class TippyBlueOppositePark extends LinearOpMode {

	RRTippyBot robot;
	double wallPos = 64.625;
	Supplier<Boolean> deadwheelCheckFunc;

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

		robot.drive.setPoseEstimate( new Pose2d( -30.5, 64.125, Math.toRadians( 270 ) ) );
		robot.lights.showStatus( RGBLights.StatusLights.WAITING );


		while( opModeIsActive( ) && !isStarted( ) ) {
			robot.drive.update( );
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition( ) );
			telemetry.update( );
		}

		waitForStart( );
		//start timer
		GameTimer.start( );
		robot.lights.setPattern( RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE );

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		new Thread( ( ) -> robot.initTF( ) ).start( );

		TrajectorySequence trajectorySequence = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				.addTemporalMarker( ( ) -> robot.liftToShippingHubHeight( height ) )
				//drop preload in alliance hub
				.splineToLinearHeading( RRTippyBot.getHubPosition( 45, 270, robot.shippingHubDistance( height ), true ), Math.toRadians( 270 ) )
				.waitSeconds( 0.5 )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 1.2 )

				//duck spin
				.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( -61, 57, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> robot.spinner.setVelocity( 250 ) )
				.waitSeconds( 0.9 )
				.addTemporalMarker( ( ) -> robot.spinner.setVelocity( 1500 ) )
				.waitSeconds( 0.3 )
				.addTemporalMarker( ( ) -> robot.spinner.setPower( 0 ) )

				//pickup duck sweep
				.addTemporalMarker( ( ) -> robot.intake.setPower( 0.4 ) )
				.setTangent( Math.toRadians( 270 ) )
				.splineToLinearHeading( new Pose2d( -48, 63, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( -24, 63 ) )
				.addTemporalMarker( ( ) -> robot.intake.setPower( 0 ) )

				//drop duck in alliance hub while turning
				.splineToSplineHeading( new Pose2d( -36, 24, 0 ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH ) )
				.splineToSplineHeading( RRTippyBot.getHubPosition( 180, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.splineToSplineHeading( new Pose2d( 12, 24, Math.toRadians( 180 ) ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> {
					deadwheelCheckFunc = robot.drive.getDeadwheelsDisabledCheck( );
					robot.drive.setDeadwheelsDisabledCheck( ( ) -> false );
					robot.odometryLift.raise( );
				} )

				//travel over barrier
				.lineToConstantHeading( new Vector2d( 12, 38 ) )
				.lineToConstantHeading( new Vector2d( 48, 38 ) )
				.addTemporalMarker( ( ) -> {
					robot.distanceSensorLocalizer.update();
					robot.drive.setPoseEstimate( robot.distanceSensorLocalizer.getPoseEstimate() );
					robot.odometryLift.lower( );
					robot.drive.setDeadwheelsDisabledCheck( deadwheelCheckFunc );
				} )

				//drive to other side to prepare for tele-op
				.lineToSplineHeading( new Pose2d( wallPos, 38, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( wallPos, -20 ) )

				.build( );

		robot.drive.followTrajectorySequence( trajectorySequence );
	}
}
