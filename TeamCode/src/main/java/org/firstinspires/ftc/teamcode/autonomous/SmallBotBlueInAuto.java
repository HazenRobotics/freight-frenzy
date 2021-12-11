package org.firstinspires.ftc.teamcode.autonomous;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
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
public class SmallBotBlueInAuto extends LinearOpMode {
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

		robot.drive.setCameraFrameOfReference( TrackingCameraLocalizer.CardinalDirection.SOUTH );

		robot.barcodeUtil.init( );

		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			robot.drive.update( );
		} while( !isStopRequested( ) && !isStarted( ) && robot.drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		robot.drive.setPoseEstimate( new Pose2d( 17, 64.25, Math.toRadians( 270 ) ) );


		while( opModeIsActive( ) && !isStarted( ) ) {
			robot.drive.update( );
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition( ) );
			telemetry.update( );
		}

		waitForStart();

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence trajectorySequence = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate() )
				.addTemporalMarker( () -> {
					robot.liftToShippingHubHeight( height );
				} )
				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( getHubPosition( -22.5, 270, robot.shippingHubDistance( height ), true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( () -> {
					robot.dumpBucket();
					robot.lift.setDefaultHeightVel( 1200 );
				} )

				.waitSeconds( 0.8 )

				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, 64, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 48, 64 ) )
				.lineToConstantHeading( new Vector2d( 18, 64 ) )
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0 );
				} )
				.addTemporalMarker( () -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.splineToSplineHeading( getHubPosition( -22.5, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
				.addTemporalMarker( () -> {
					robot.dumpBucket();
					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 49, 64, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 53, 64 ) )
				.lineToConstantHeading( new Vector2d( 18, 64 ) )
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0 );
				} )
				.addTemporalMarker( () -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.splineToSplineHeading( getHubPosition( -22.5, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
				.addTemporalMarker( () -> {
					robot.dumpBucket();
					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, 64, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 50, 64 ) )
				.lineToConstantHeading( new Vector2d( 18, 64 ) )
				.addTemporalMarker( () -> {
					robot.intake.setPower( 0 );
				} )
				.addTemporalMarker( () -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.splineToSplineHeading( getHubPosition( -22.5, 270, robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH ), true ), Math.toRadians( 270 ) )
				.addTemporalMarker( () -> {
					robot.dumpBucket();
					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.addTemporalMarker( ( ) -> {
					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
					robot.odometryLift.liftOdometry( );
				} )
				.waitSeconds( 0.8 )
				.turn( Math.toRadians( -110 ) )
				/*// move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 11.5, 44, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 62, 44 ) )*/
				.build();

		robot.drive.followTrajectorySequence( trajectorySequence );
	}

	public Pose2d getHubPosition( double angle, double angleOffset, double indent, boolean blueSide ) {
		double x = tileConnector / 2 + tileSize / 2 + Math.sin( Math.toRadians( angle ) ) * (hubRadius + indent + robotLength / 2);
		double y = tileConnector + tileSize + Math.cos( Math.toRadians( angle ) ) * (hubRadius + indent + robotLength / 2);
		return new Pose2d( -x, y * (blueSide ? 1 : -1), Math.toRadians( angleOffset + angle ) );
		// new Pose2d( -23.631, 35.506, toRadians( 270 + 45 ) )
	}
}
