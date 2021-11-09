package org.firstinspires.ftc.teamcode.autonomous;

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

		robot.barcodeUtil.stopCamera( );
		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( robot.barcodeUtil.getBarcodePosition( ) );

		robot.lift.setLiftHeightVel( 750, 16 );

		while( robot.lift.isBusy( ) );
		robot.sleepRobot( 3 );

		robot.liftToShippingHubHeight( height );

		while( !isStopRequested() );
		/*
		TrajectorySequence dropFreight = robot.getTrajectorySequenceBuilder( )
				.splineToConstantHeading( new Vector2d( -12, -50 ), 90 )
				.lineToConstantHeading( new Vector2d( -12, -24 - 12 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( height ) ) ) )
				.build( );
		robot.drive.followTrajectorySequence( dropFreight );
		telemetry.addLine( "lift (in) before tall: " + robot.lift.getLiftPositionInch( ) );
		robot.liftToShippingHubHeight( height );
		telemetry.addLine( "lift (in) after tall: " + robot.lift.getLiftPositionInch( ) );
		//robot.dumpBucket();
		robot.lift.setDefaultHeightVel( 750 );
		telemetry.addLine( "lift (in) after short: " + robot.lift.getLiftPositionInch( ) );
		telemetry.update( );

		TrajectorySequence duckSpinner = robot.getTrajectorySequenceBuilder( )
				.lineToConstantHeading( new Vector2d( -12, -50 ) )
				.splineToLinearHeading( new Pose2d( -57, -57, 0 ), 0 )
				.build( );
		robot.drive.followTrajectorySequence( duckSpinner );
		robot.spinnerRight.setPower( 0.75 );
		robot.sleepRobot( 3 );
		robot.spinnerRight.setPower( 0 );

		TrajectorySequence pickupDuck = robot.getTrajectorySequenceBuilder( )
				.splineToConstantHeading( new Vector2d( -62, -50 ), 90 )
				.build( );
		robot.drive.followTrajectorySequence( pickupDuck );
		robot.intake.setPower( -1 );
		robot.sleepRobot( 2 );
		robot.intake.setPower( 0 );

		TrajectorySequence dropDuck = robot.getTrajectorySequenceBuilder( )
				.splineToLinearHeading( new Pose2d( -12, -24 - 12 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.LOW ) ), Math.toRadians( 90 ) ), 90 )
				.build( );
		robot.drive.followTrajectorySequence( dropDuck );
		robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
		//robot.dumpBucket();
		robot.lift.setDefaultHeightPow( 0.8 );

		TrajectorySequence driveToBumps = robot.getTrajectorySequenceBuilder( )
				.splineToLinearHeading( new Pose2d( 0, -42, 0 ), 0 )
				.build( );
		robot.drive.followTrajectorySequence( driveToBumps );

		TrajectorySequence park = robot.getTrajectorySequenceBuilder( )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 17 ) )
				.lineTo( new Vector2d( 36, -42 ) )
				.resetVelConstraint( )
				.splineTo( new Vector2d( 60, -36 ), 0 )
				.build( );

		robot.drive.followTrajectorySequence( park );
*/
	}
}
