package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;

@Autonomous
public class RedOutAuto extends LinearOpMode {

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

		robot.drive.setPoseEstimate( new Pose2d( /*-29.375*/-42.375, -62.1875, Math.toRadians( 90 ) ) );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

		while( !isStopRequested( ) && !isStarted( ) )
			robot.drive.update( );

		waitForStart( );

		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )

				//Drop block in Shipping Hub
				.lineToLinearHeading( new Pose2d( barcodePosition == BarcodePositionDetector.BarcodePosition.LEFT ? -40 : barcodePosition == BarcodePositionDetector.BarcodePosition.RIGHT ? -32 : -36 , -48, 0 ) )
				.strafeLeft( 30 )
				//.splineToLinearHeading( new Pose2d( -58, -36, Math.toRadians( -45 ) ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> robot.liftToShippingHubHeight( height ) )
				.lineToConstantHeading( new Vector2d( -36, -24 ))
				.lineToConstantHeading( new Vector2d( -12 - 10.5 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( height )), -24 ))
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 0.7 )

				//Duck spin
				.lineToConstantHeading( new Vector2d( -36, -24 ) )
				.lineToConstantHeading( new Vector2d( -56.5, -57.5 ) )
				.addTemporalMarker( ( ) -> {
					robot.spinnerRight.setPower( 0.75 );
				} )
				.waitSeconds( 3.2 )
				.addTemporalMarker( ( ) -> robot.spinnerRight.setPower( 0 ) )

				//Pickup duck from ground
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( -0.6 );
				} )
				.lineToConstantHeading( new Vector2d( -60, -51 ) )
				.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
				} )

				//Drop duck in Shipping Hub
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
				} )
				.splineToConstantHeading( new Vector2d( -36, -24 ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( -12 - 10.5 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.LOW ) ), -24 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 0.7 )

				//Park in Warehouse
				.lineToConstantHeading( new Vector2d( -36, -24 ) )
				.splineToConstantHeading( new Vector2d( -36, -50 ), Math.toRadians( 270 ) )
				//.setVelConstraint( new MecanumVelocityConstraint( 35, 17 ) )
				.splineToConstantHeading( new Vector2d( -10, -50 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 0, -50 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 10, -40 ), Math.toRadians( 90 ) )


				.setVelConstraint( new MecanumVelocityConstraint( 50, 17 ) )
				.lineTo( new Vector2d( 36, -42 ) )
				.resetVelConstraint( )
				.splineTo( new Vector2d( 60, -44 ), 0 )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

		/*for( int i = 0; i < 2; i++ ) {
			robot.liftToShippingHubHeight( height );
		}
		TrajectorySequence dropFreight = robot.getTrajectorySequenceBuilder( )
				.splineToConstantHeading( new Vector2d( -12, -52 ), 90 )
				.lineToConstantHeading( new Vector2d( -12, -24 - 10 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( height ) ) ) )
				.addTemporalMarker( ( ) -> {

					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.build( );
		robot.drive.followTrajectorySequence( dropFreight );


		TrajectorySequence duckSpinner = robot.getTrajectorySequenceBuilder( )
				.lineToConstantHeading( new Vector2d( -12, -52 ) )
				.lineToLinearHeading( new Pose2d( -58, -58, 0 ) )
				.build( );
		robot.drive.followTrajectorySequence( duckSpinner );
		robot.spinnerRight.setPower( 0.75 );
		robot.sleepRobot( 3.2 );
		robot.spinnerRight.setPower( 0 );

		TrajectorySequence pickupDuck = robot.getTrajectorySequenceBuilder( )
				.lineToConstantHeading( new Vector2d( -60, -51 ) )
				.build( );
		robot.drive.followTrajectorySequence( pickupDuck );
		robot.intake.setPower( -1 );
		robot.sleepRobot( 2 );
		robot.intake.setPower( 0 );

		for( int i = 0; i < 2; i++ ) {
			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
		}
		TrajectorySequence dropDuck = robot.getTrajectorySequenceBuilder( )
				.lineToLinearHeading( new Pose2d( -12, -52, Math.toRadians( 90 ) ) )
				.lineTo( new Vector2d( -12, -41 ) )
				.build( );

		robot.drive.followTrajectorySequence( dropDuck );
		robot.dumpBucket( );
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

		robot.drive.followTrajectorySequence( park );*/
	}
}
