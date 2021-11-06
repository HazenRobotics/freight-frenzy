package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.robots.RRHexBot;

@Autonomous
public class RedInAuto extends LinearOpMode {

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


		waitForStart();
		robot.barcodeUtil.stopCamera();
		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( robot.barcodeUtil.getBarcodePosition() );

		//set start position
		robot.drive.setPoseEstimate( new Pose2d( 5.625, -62.1875, Math.toRadians( 90 ) ) );
		robot.drive.followTrajectorySequence( robot.getTrajectorySequenceBuilder()
				.splineToConstantHeading( new Vector2d( -12, -24 - 2 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( height ) ) ), 0 )
				.addDisplacementMarker( () -> {
					robot.liftToShippingHubHeight( height );
					robot.dumpBucket();
					robot.lift.setDefaultHeightPow(0.8);
				} )
				.splineToLinearHeading( new Pose2d( -52, -52,0 ), 0 )
				.addDisplacementMarker( () -> {
					robot.spinnerRight.setPower( 0.5 );
					robot.sleepRobot( 1500 );
					robot.spinnerRight.setPower( 0 );
				} )
				.splineTo(new Vector2d( -62, -50 ), 0)
				.addDisplacementMarker( () -> {
					robot.intake.setPower( 1 );
					robot.sleepRobot( 800 );
					robot.intake.setPower( 0 );
				} )
				.splineToLinearHeading( new Pose2d( -12, -24 - 2 - robot.lift.calcBucketDistanceFromHeight( robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.LOW ) ), 90 ), 0 )
				.addDisplacementMarker( () -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
					robot.dumpBucket();
					robot.lift.setDefaultHeightPow(0.8);
				} )
				.splineToLinearHeading( new Pose2d( 0, -42, 0 ), 0 )
				.setVelConstraint(new MecanumVelocityConstraint(50, 17))
				.lineTo( new Vector2d( 36, 42 ) )
				.resetVelConstraint()
				.splineTo( new Vector2d( 60, 36 ), 0 )
				.build() );

	}
}
