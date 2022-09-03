package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
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
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;



@Autonomous
public class TippyBlueOutAuto extends LinearOpMode {

	RRTippyBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		TippyBotTeleOp.isBlueSide = true;

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new RRTippyBot( this, true );

		robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );

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
		GameTimer.start();
		robot.lights.setPattern( RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE );

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence beforeDuckPickup = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				// Duck spin
				.setTangent( Math.toRadians( 225 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( new Pose2d( -61, 57, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setVelocity( 150 );
				} )
				.waitSeconds( 0.94 )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setPower( 1 );
				} )
				.waitSeconds( 0.4 )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setPower( 0 );
				} )

				// start duck scanning, move lift up, and move to drop off block
				.addTemporalMarker( ( ) -> {
//					robot.startDuckScanning( 250 );
					robot.liftToShippingHubHeight( height );
				} )

				.setTangent( Math.toRadians( 300 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( RRTippyBot.getHubPosition( 45, 270, robot.shippingHubDistance( height ), true ), Math.toRadians( 290 ) )
				.waitSeconds( 0.5 )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
				} )
				.waitSeconds( 1.2 )
				.build( );
		robot.drive.followTrajectorySequence( beforeDuckPickup );
		robot.lift.setDefaultHeightVel( 1000 );

//		robot.waitForDuck( );

		if( robot.getDuckPosition( ) == null ) {
			SoundLibrary.playAudio( "nooo" );
			robot.drive.followTrajectorySequence( robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )
					.turn( Math.toRadians( -30 ) )
					.build( ) );
		} else
			SoundLibrary.playAudio( "fine_addition" );

		TrajectorySequence afterPickupDuck = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )
				// pickup the duck
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0.4 );
					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
				} )
				.setTangent( Math.toRadians( 100 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( robot.getDuckPosition( Math.toRadians( 270 ) ) == null ? new Pose2d( -40, 60, Math.toRadians( 270 ) ) : robot.getDuckPosition( Math.toRadians( 270 ) ).plus( new Pose2d( 0, -7 ) ), Math.toRadians( 90 ) )
				.strafeLeft( 2 )
				.strafeRight( 4 )

				//.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
					robot.sleepRobot( 1.5 );
					robot.intake.setPower( 0 );
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
					robot.stopDuckScanning( );
					robot.stopFreightDetection();
				} )
				//.waitSeconds( 1.0 )

				// drop duck in top
				.setTangent( Math.toRadians( 320 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( RRTippyBot.getHubPosition( 45, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
				.waitSeconds( 0.5 )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.sleepRobot( 1 );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.5 )
				/*
				//Line up for parking
				.lineToLinearHeading( new Pose2d( -36, 44, Math.toRadians( 0 ) ) )
				*/
				/*


				// move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( 11.5, 44, 0 ), Math.toRadians( -45 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, 44, 0 ) )
				*/
				/*.setVelConstraint( new MecanumVelocityConstraint( 45, 11.5 ) )
				.setTangent( Math.toRadians( 90 ) )
				.splineToConstantHeading( new Vector2d( -55, 36 ), Math.toRadians( 270 ) )
				.splineToSplineHeading( new Pose2d( -12, 0, Math.toRadians( 0 ) ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 12, 44 ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> {

				} )
				.waitSeconds( 1.2 )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, 44, 0 ) )*/
				.build( );
		robot.drive.followTrajectorySequence( afterPickupDuck );

		robot.drive.setDeadwheelsDisabledCheck( () -> true );
		robot.odometryLift.raise( );
		//Park in last 3 seconds
		while(opModeIsActive() && GameTimer.remainingTimeAutonomous() > 3);
		robot.drive.followTrajectorySequence( robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate() )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToConstantHeading(new Vector2d( 48, 44 ))
				.build());

		while( !isStopRequested( ) && !opModeIsActive( ) );

	}
}
