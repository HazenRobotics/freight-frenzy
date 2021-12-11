package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

//@Autonomous
public class RedInAutoDuck extends LinearOpMode {

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
		robot.drive.setPoseEstimate( new Pose2d( 2.125, -62.1875, Math.toRadians( 90 ) ) );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )

				//Drop block in Shipping Hub
				.addTemporalMarker( () -> {
					robot.liftToShippingHubHeight( height );
				} )
				.splineToConstantHeading( new Vector2d( -12, -54 ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( -12 , -24 - robot.distanceFromShippingHub( height ) ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				//Duck spin
				.lineToConstantHeading( new Vector2d( -12, -54 ) )
				.lineToLinearHeading( new Pose2d( -56, -58.5, 0 ) )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setPower( 0.75 );
				} )
				.waitSeconds( 3.2 )
				.addTemporalMarker( ( ) -> {
					robot.spinner.setPower( 0 );
				} )

				//Pickup duck from ground
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( -0.6 );
				} )
				.lineToConstantHeading( new Vector2d( -60, -51 ) )
				.strafeRight( 2 )
				.strafeLeft( 12 )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
				} )

				//Drop duck in Shipping Hub
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
				} )
				.splineTo(new Vector2d( -36, -50 ), Math.toRadians( 0 ))
				.splineToLinearHeading( new Pose2d( -12, -54, Math.toRadians( 90 ) ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( -12 , -24 - robot.distanceFromShippingHub( RRHexBot.ShippingHubHeight.LOW )) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				//Park in Warehouse
				.lineToLinearHeading( new Pose2d( -12, -54, 0 ) )
				.splineToConstantHeading( new Vector2d( -10, -50 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 6, -50 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 10, -40 ), Math.toRadians( 90 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 17 ) )
				.lineTo( new Vector2d( 36, -42 ) )
				.resetVelConstraint( )
				.splineTo( new Vector2d( 60, -44 ), 0 )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
