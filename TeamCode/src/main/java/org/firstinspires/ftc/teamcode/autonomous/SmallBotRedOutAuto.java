package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class SmallBotRedOutAuto extends LinearOpMode {

	RRTippyBot robot;

	static final double tileSize = 23;
	static final double tileConnector = 0.75;
	static final double hubRadius = 9;
	static double robotLength = 13.25;

	@Override
	public void runOpMode( ) throws InterruptedException {

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new RRTippyBot( this );

		robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_MOVING );

		robot.drive.setCameraFrameOfReference( TrackingCameraLocalizer.CardinalDirection.NORTH );

		robot.barcodeUtil.init( );

		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			robot.drive.update( );
		} while( !isStopRequested( ) && !isStarted( ) && robot.drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		robot.drive.setPoseEstimate( new Pose2d( -40.75, -64.25, Math.toRadians( 90 ) ) );


		while( opModeIsActive( ) && !isStarted( ) ) {
			robot.drive.update( );
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition( ) );
			telemetry.update( );
		}

		waitForStart( );

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence trajectorySequence = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( height );

				} )

				.splineToLinearHeading( getHubPosition( -22.5, 90, robot.shippingHubDistance( height ), false ), Math.toRadians( 90 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				// Duck spin
				.lineToLinearHeading( new Pose2d( -62, -58, Math.toRadians( 90 ) ) )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setPower( -0.6 );
				} )
				.waitSeconds( 3.2 )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setPower( 0 );
				} )

				// pickup the duck
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0.5 );
				} )
				.setTangent( 0 )
				.splineToLinearHeading( new Pose2d( -50, -62.1875, Math.toRadians( 90 ) ), Math.toRadians( 270 ) )
				.lineToConstantHeading( new Vector2d( -13 - 5, -62.1875 ) )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_MOVING );
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )

				// drop duck in top
				.splineToLinearHeading( getHubPosition( 0, 90, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), false ), Math.toRadians( 90 - 22.5 ) )
				.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.addTemporalMarker( ( ) -> {
					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
					robot.odometryLift.liftOdometry( );
				} )
				.waitSeconds( 1.2 )

				// move to barrier to park

				.setTangent( Math.toRadians( 270 ) )
				.splineToLinearHeading( new Pose2d( 11.5, -44, 0 ), Math.toRadians( 45 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, -44, 0 ) )

				.waitSeconds( 6 )

				.build( );

		robot.drive.followTrajectorySequence( trajectorySequence );
	}

	public Pose2d getHubPosition( double angle, double angleOffset, double indent, boolean blueSide ) {
		double negate = Math.toRadians( angle * (blueSide ? 1 : -1) );
		double x = tileConnector / 2 + tileSize / 2 + Math.sin( negate ) * (hubRadius + indent + robotLength / 2);
		double y = tileConnector + tileSize + Math.cos( negate ) * (hubRadius + indent + robotLength / 2);
		return new Pose2d( -x, y * (blueSide ? 1 : -1), Math.toRadians( angleOffset + angle ) );
		// new Pose2d( -23.631, 35.506, toRadians( 270 + 45 ) )
	}
}
