package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;

@Autonomous
public class RedOutAuto extends LinearOpMode {

	RRHexBot robot;
	final int PARKING_ZONE = 3;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new RRHexBot( this );
		robot.barcodeUtil.init( );
		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			robot.drive.update( );
		} while( !isStopRequested( ) && !isStarted( ) && robot.drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		telemetry.addLine( "Ready!" );
		telemetry.update( );
		robot.drive.setPoseEstimate( new Pose2d( -42.375, -62.1875, Math.toRadians( 90 ) ) );
		while( !isStopRequested( ) && !isStarted( ) ) {
			robot.drive.update( );
		}

		waitForStart( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( robot.barcodeUtil.getBarcodePosition( ) );
		robot.barcodeUtil.stopCamera( );

		robot.liftToShippingHubHeight( height );
		TrajectorySequence dropFreight = robot.getTrajectorySequenceBuilder( )
				.splineToConstantHeading( new Vector2d( -12, -52 ), 90 )
				.lineToConstantHeading( new Vector2d( -12, -24 - 10 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( height ) ) ) )
				.build( );

		robot.drive.followTrajectorySequence( dropFreight );
		robot.dumpBucket();
		robot.lift.setDefaultHeightVel( 1000 );

		TrajectorySequence duckSpinner = robot.getTrajectorySequenceBuilder( )
				.lineToConstantHeading( new Vector2d( -12, -52 ) )
				.lineToLinearHeading( new Pose2d( -57, -57, 0 ) )
				.build( );
		robot.drive.followTrajectorySequence( duckSpinner );
		robot.spinnerRight.setPower( 0.75 );
		robot.sleepRobot( 3.2 );
		robot.spinnerRight.setPower( 0 );

		TrajectorySequence pickupDuck = robot.getTrajectorySequenceBuilder( )
				.lineToConstantHeading( new Vector2d( -60, -51 ))
				.build( );
		robot.drive.followTrajectorySequence( pickupDuck );
		robot.intake.setPower( -1 );
		robot.sleepRobot( 2 );
		robot.intake.setPower( 0 );

		robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
		TrajectorySequence dropDuck = robot.getTrajectorySequenceBuilder( )
				.lineToLinearHeading( new Pose2d( -12, -52, Math.toRadians( 90 ) ))
				.lineTo( new Vector2d( -12, -41 ) )
				.build( );

		robot.drive.followTrajectorySequence( dropDuck );
		robot.dumpBucket();
		robot.lift.setDefaultHeightVel( 1000 );

		TrajectorySequence driveToBumps = robot.getTrajectorySequenceBuilder( )
				.lineToConstantHeading( new Vector2d( -12, -55 ) )
				.splineToLinearHeading( new Pose2d( 0, -42, 0 ), 0 )
				.build( );
		robot.drive.followTrajectorySequence( driveToBumps );

		TrajectorySequence park = robot.getTrajectorySequenceBuilder( )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 17 ) )
				.lineTo( new Vector2d( 36, -42 ) )
				.resetVelConstraint( )
				.splineTo( new Vector2d( 60, -38 ), 0 )
				.build( );

		robot.drive.followTrajectorySequence( park );
	}
}
