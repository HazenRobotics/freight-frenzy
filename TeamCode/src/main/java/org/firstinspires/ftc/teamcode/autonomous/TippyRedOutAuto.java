package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

@Autonomous
public class TippyRedOutAuto extends LinearOpMode {

	RRTippyBot robot;

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

		robot.drive.setPoseEstimate( new Pose2d( -30.5, -64.125, Math.toRadians( 90 ) ) );


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

		TrajectorySequence beforeDuckPickup = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				// Duck spin
				.setTangent( Math.toRadians( 225 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( new Pose2d( -61, -57, Math.toRadians( 90 ) ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setVelocity( 325 );
				} )
				.waitSeconds( 3.0 )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setPower( 0 );
				} )

				// start duck scanning, move lift up, and move to drop off block
				.addTemporalMarker( ( ) -> {
					robot.startDuckScanning( 250 );
					robot.liftToShippingHubHeight( height );
				} )

				.setTangent( Math.toRadians( 240 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( RRTippyBot.getHubPosition( 45, 90, robot.shippingHubDistance( height ), false ), Math.toRadians( 270 + 22.5 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
				} )
				.waitSeconds( 1.2 )
				.build( );
		robot.drive.followTrajectorySequence( beforeDuckPickup );
		robot.lift.setDefaultHeightVel( 1000 );

		robot.waitForDuck();

		if(robot.getDuckPosition() == null) {
			robot.drive.followTrajectorySequence( robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate() )
					.turn( Math.toRadians( -10 ) )
					.build());
		}

		TrajectorySequence afterPickupDuck = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate() )
				// pickup the duck
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0.4 );
					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
				} )
				.setTangent( Math.toRadians( 100 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( robot.getDuckPosition( Math.toRadians( 90 ) ) == null ? new Pose2d( -40, 60, Math.toRadians( 90 ) ) : robot.getDuckPosition( Math.toRadians( 90 ) ).plus( new Pose2d( 0, 4 ) ), Math.toRadians( 270 ) )
				.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
					robot.stopDuckScanning( );
					robot.stopTF();
				} )

				// drop duck in top
				.setTangent( Math.toRadians( 320 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( RRTippyBot.getHubPosition( 22, 90, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), false ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				/*.addTemporalMarker( ( ) -> {
					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
					robot.odometryLift.raise( );
				} )*/
				.waitSeconds( 1.2 )

				// move to barrier to park
				/*.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( 11.5, 44, 0 ), Math.toRadians( -45 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, 44, 0 ) )*/
				.build();
		robot.drive.followTrajectorySequence( afterPickupDuck );

	}
}
