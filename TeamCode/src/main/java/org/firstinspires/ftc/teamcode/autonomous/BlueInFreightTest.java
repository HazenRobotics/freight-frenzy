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
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

//@Autonomous
public class BlueInFreightTest extends LinearOpMode {

	RRTippyBot robot;

	double wallPos = 64.625;

	@Override
	public void runOpMode( ) throws InterruptedException {

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

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		new Thread( ( ) -> robot.initTF( ) ).start( );

		TrajectorySequence beforeFerrying = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				// move to dump initial block in designated layer
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( height );
				} )
				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( RRTippyBot.getHubPosition( -22.5, 270, robot.shippingHubDistance( height ), true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
				} )

				.waitSeconds( 0.8 )
				.build();
		robot.drive.followTrajectorySequence( beforeFerrying );
		robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );

		//run 3 times
		for(int i = 0; i <= 3; i++) {
			robot.drive.followTrajectorySequence( robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate() )
					// move to grab block 1
					.setTangent( Math.toRadians( 90 ) )
					.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
					.build());

			//scan for blocks and select block closest to the robot
			Vector2d freightPosition;
			do {
				freightPosition = robot.getClosestFreightPosition(  );
			} while( freightPosition == null );


			robot.drive.followTrajectorySequence( robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate() )

					.addTemporalMarker( ( ) -> {
						robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
					} )
					.setTangent( 0 )
					.splineToConstantHeading( freightPosition, Math.toRadians( 180 ) )
					.setTangent( 180 )
					.splineToConstantHeading( new Vector2d( 18, wallPos ), Math.toRadians( 0 ) )
					.addTemporalMarker( ( ) -> {
						robot.intake.setPower( 0 );
					} )
					.addTemporalMarker( ( ) -> {
						robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
					} )
					.splineToSplineHeading( RRTippyBot.getHubPosition( -22.5, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
					.addTemporalMarker( ( ) -> {
						robot.dumpBucket( );
					} )
					.waitSeconds( 0.8 )
					.build( ));
			robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );

		}
	}

}
