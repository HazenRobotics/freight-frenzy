package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

@Autonomous
public class TippyBlueInAuto extends LinearOpMode {

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


		while( opModeIsActive( ) && !isStarted( ) ) {
			robot.drive.update( );
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition( ) );
			telemetry.update( );
		}

		waitForStart( );

		//start timer
		GameTimer.start();

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence trajectorySequence = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				// move to dump initial block in designated layer
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( height );
				} )
				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( RRTippyBot.getHubPosition( -22.5, 270, robot.shippingHubDistance( height ), true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )

				.waitSeconds( 0.8 )

				// move to grab block 1
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
					robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.lineToConstantHeading( new Vector2d( 50, wallPos ) )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )

				// move to dump block 1 in the top layer

				.splineToSplineHeading( RRTippyBot.getHubPosition( -22.5, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 2
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
					robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.lineToConstantHeading( new Vector2d( 52, wallPos ) )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )

				// move to dump block 2 in the top layer
				.splineToSplineHeading( RRTippyBot.getHubPosition( -22.5, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )
				.addTemporalMarker( ( ) -> {
					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
					robot.odometryLift.raise( );
				} )
				.waitSeconds( 0.8 )

				/*// move to grab block 3
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
					robot.intake.intakeBlocks( 0.6, 2, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.lineToConstantHeading( new Vector2d( 54, wallPos ) )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )

				// move to dump block 3 in the top layer
				.splineToSplineHeading( RRTippyBot.getHubPosition( -22.5, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200 );
				} )*/

				//.waitSeconds( 0.8 )
				//park

				.addTemporalMarker( ( ) -> {
					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
					robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 48, wallPos ) )

				// turn towards the
				//.turn( Math.toRadians( 110 ) )
				/*// move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 11.5, 44, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 62, 44 ) )*/
				.build( );

		robot.drive.followTrajectorySequence( trajectorySequence );
	}
}
