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

@Autonomous
public class RedOutAuto extends LinearOpMode {

	RRHexBot robot;
	final int PARKING_ZONE = 3;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new RRHexBot( this );
		robot.barcodeUtil.init();
		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update();
			robot.drive.update();
		} while( !isStopRequested() && !isStarted() && robot.drive.getPoseConfidence().compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		telemetry.addLine( "Ready!" );
		telemetry.update();
		robot.drive.setPoseEstimate( new Pose2d( -29.75, -62.1875, Math.toRadians( 90 ) ) );
		while( !isStopRequested() && !isStarted() ) {
			robot.drive.update();
		}

		waitForStart();

		robot.barcodeUtil.stopCamera();
		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( robot.barcodeUtil.getBarcodePosition() );

		TrajectorySequence dropFreight = robot.getTrajectorySequenceBuilder()
				.splineToConstantHeading( new Vector2d( -24 + 3, -24 - 1.5), 0) //-12, -24 - 2 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( height ) ) ) )
				.build();
		robot.drive.followTrajectorySequence( dropFreight );
		robot.liftToShippingHubHeight( height );
		robot.dumpBucket();
		robot.lift.setDefaultHeightPow(0.8);

		TrajectorySequence duckSpinner = robot.getTrajectorySequenceBuilder()
				.splineToLinearHeading( new Pose2d( -52, -52,0 ), 0 )
				.build();
		robot.drive.followTrajectorySequence( duckSpinner );
		robot.spinnerRight.setPower( 0.5 );
		robot.sleepRobot( 1500 );
		robot.spinnerRight.setPower( 0 );

		TrajectorySequence pickupDuck = robot.getTrajectorySequenceBuilder()
				.splineTo(new Vector2d( -62, -50 ), 0)
				.build();
		robot.drive.followTrajectorySequence( pickupDuck );
		robot.intake.setPower( 1 );
		robot.sleepRobot( 800 );
		robot.intake.setPower( 0 );

		TrajectorySequence dropDuck = robot.getTrajectorySequenceBuilder()
				.splineToLinearHeading( new Pose2d( -12, -24 - 2 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.LOW ) ), 90 ), 0 )
				.build();
		robot.drive.followTrajectorySequence( dropDuck );
		robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
		robot.dumpBucket();
		robot.lift.setDefaultHeightPow( 0.8 );

		TrajectorySequence driveToBumps = robot.getTrajectorySequenceBuilder()
				.splineToLinearHeading( new Pose2d( 0, -42, 0 ), 0 )
				.build();
		robot.drive.followTrajectorySequence( driveToBumps );

		TrajectorySequence park = robot.getTrajectorySequenceBuilder()
				.setVelConstraint(new MecanumVelocityConstraint(50, 17))
				.lineTo( new Vector2d( 36, 42 ) )
				.resetVelConstraint()
				.splineTo( new Vector2d( 60, 36 ), 0 )
				.build();

		robot.drive.followTrajectorySequence( park );

	}
}
