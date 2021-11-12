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
public class BlueInAuto extends LinearOpMode {

	RRHexBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new RRHexBot( this );

		robot.barcodeUtil.init( );

		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			robot.drive.update( );
		} while( !isStopRequested( ) && !isStarted( ) && robot.drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		robot.drive.setPoseEstimate( new Pose2d( 2.125, 62.1875, Math.toRadians( -90 ) ) );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

		while( !isStopRequested( ) && !isStarted( ) )
			robot.drive.update( );

		waitForStart( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( robot.barcodeUtil.getBarcodePosition( ) );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )

				//Drop block in Shipping Hub
				//.lineToConstantHeading( new Vector2d( -52, -36 ) )
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( height );
					robot.liftToShippingHubHeight( height );
				} )
				.splineToConstantHeading( new Vector2d( -12, 54 ), Math.toRadians( 270 )  )
				.lineToConstantHeading( new Vector2d( -12 , 24 + 10.5 + robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( height ) )  ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 0.7 )

				//Duck spin
				.lineToConstantHeading( new Vector2d( -12, 54 ) )
				.lineToLinearHeading( new Pose2d( -56.5, 57.5, Math.toRadians( 0 ) ) )
				.addTemporalMarker( ( ) -> {
					robot.spinnerLeft.setPower( 0.75 );
				} )
				.waitSeconds( 3.2 )
				.addTemporalMarker( ( ) -> {
					robot.spinnerLeft.setPower( 0 );
				} )

				//Pickup duck from ground
				.lineToConstantHeading( new Vector2d( -60, 51 ) )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( -0.6 );
				} )
				.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
				} )

				//Drop duck in Shipping Hub
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
				} )
				.lineToLinearHeading( new Pose2d( -12, 54, Math.toRadians( -90 ) ) )
				.lineToConstantHeading( new Vector2d( -12 , 24 + 10.5 + robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.LOW ) ) ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 0.7 )

				//Park in Warehouse
				.lineToLinearHeading( new Pose2d( -12, 54, 0 ) )
				.splineToConstantHeading( new Vector2d( 10, 40 ), Math.toRadians( 270 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 17 ) )
				.lineTo( new Vector2d( 36, 42 ) )
				.resetVelConstraint( )
				.splineTo( new Vector2d( 60, 44 ), 0 )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
