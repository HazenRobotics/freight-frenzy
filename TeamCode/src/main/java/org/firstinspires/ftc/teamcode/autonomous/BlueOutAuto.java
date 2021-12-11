package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

//@Autonomous
public class BlueOutAuto extends LinearOpMode {

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
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition( ) );
			telemetry.addLine( );
			telemetry.addData( "Current pose estimate", robot.drive.getPoseEstimate( ) );
			telemetry.addLine( );
			telemetry.addLine( "Pose estimate should be somewhere close to (0, 0, 0). If not, run the \"ResetCamera\" Autonomous." );
			telemetry.update( );
		}

		waitForStart( );

		//set it to mecanum wheels in very beginning

		Localizer localizer = robot.drive.getLocalizer( );
		robot.drive.setLocalizer( new MecanumDrive.MecanumLocalizer( robot.drive ) );


		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

//		TrajectorySequence cameraFix = robot.getTrajectorySequenceBuilder( )
//				.setAccelConstraint( new ProfileAccelerationConstraint( 50 ) )
//				.setVelConstraint( new MecanumVelocityConstraint( 50, 17 ) )
//				.forward( 8 )
//				.strafeRight( 8 )
//				.strafeLeft( 8 )
//				.back( 8 )
//				.build( );
//		robot.drive.followTrajectorySequence( cameraFix );
		robot.drive.setLocalizer( localizer );
		robot.drive.setPoseEstimate( new Pose2d( -42.375, 62.1875, Math.toRadians( -90 ) ) );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )

				.splineToLinearHeading( robot.getHubPosition( 22.5, 270, robot.shippingHubHeightToInches( height ), true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
//											robot.dumpBucket( );
//											robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				// Duck spin
				.lineToLinearHeading( new Pose2d( -58.5, 56, Math.toRadians( -90 ) ) )
				.addTemporalMarker( ( ) -> robot.spinner.setPower( 0.5 ) )
				.waitSeconds( 3.2 )
				.addTemporalMarker( ( ) -> robot.spinner.setPower( 0 ) )

				// pickup the duck
				.addTemporalMarker( ( ) -> robot.intake.setPower( 0.6 ) )
				.setTangent( 0 )
				.splineToLinearHeading( new Pose2d( -50, 62.1875, Math.toRadians( -90 ) ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( -13 - 5, 62.1875 ) )
				.addTemporalMarker( ( ) -> robot.intake.setPower( 0 ) )

				// drop duck in bottom (always)
				.splineToLinearHeading( robot.getHubPosition( 0, 270, robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.LOW ), true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				// move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( 11.5, 40.25, 0 ), Math.toRadians( -45 ) )
				.lineToLinearHeading( new Pose2d( 62, 40.25, 0 ) )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
