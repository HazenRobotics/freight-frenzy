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

@Autonomous
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
			telemetry.addData( "Element position", robot.barcodeUtil.getBarcodePosition() );
			telemetry.addLine(  );
			telemetry.addData( "Current pose estimate", robot.drive.getPoseEstimate() );
			telemetry.addLine(  );
			telemetry.addLine( "Pose estimate should be somewhere close to (0, 0, 0). If not, run the \"ResetCamera\" Autonomous." );
			telemetry.update();
		}


		waitForStart( );

		//set it to mecanum wheels in very beginning

		Localizer localizer = robot.drive.getLocalizer();
		robot.drive.setLocalizer( new MecanumDrive.MecanumLocalizer( robot.drive ) );


		BarcodePositionDetector.BarcodePosition barcodePosition = robot.barcodeUtil.getBarcodePosition( );

		RRHexBot.ShippingHubHeight height = robot.barcodePosToShippingHubHeight( barcodePosition );
		robot.barcodeUtil.stopCamera( );

		TrajectorySequence cameraFix = robot.getTrajectorySequenceBuilder()
				.setVelConstraint( new MecanumVelocityConstraint( 55, 17 ) )
				.setAccelConstraint( new ProfileAccelerationConstraint( 90 ))
				.forward( 12 )
				.back( 12.5 )
				.build();
		robot.drive.followTrajectorySequence( cameraFix );
		robot.drive.setLocalizer( localizer );
		robot.drive.setPoseEstimate( new Pose2d( /*-29.375*/-42.375, 62.1875, Math.toRadians( -90 ) ) );
		robot.sleepRobot( 2 );


		TrajectorySequence mainTrajectory = robot.drive.trajectorySequenceBuilder( new Pose2d(  ) )
				.addTemporalMarker( () -> {
			telemetry.addData( "PoseConfidence", robot.drive.getPoseConfidence() );
			telemetry.update();
		} )
				.waitSeconds( 5 )

				//Drop block in Shipping Hub
				.lineToLinearHeading( new Pose2d( barcodePosition == BarcodePositionDetector.BarcodePosition.LEFT ? -32 : barcodePosition == BarcodePositionDetector.BarcodePosition.RIGHT ?  -40 : -36 , 48, 0 ) )

				.addTemporalMarker( ( ) -> robot.liftToShippingHubHeight( height ) )
				.strafeRight( 30 )
				//.splineToLinearHeading( new Pose2d( -58, -36, Math.toRadians( -45 ) ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( -36, 24 ))
				.lineToConstantHeading( new Vector2d( -12 - robot.distanceFromShippingHub( height ), 24 ))
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				//Duck spin
				.lineToConstantHeading( new Vector2d( -36, 24 ) )
				.lineToConstantHeading( new Vector2d( -58.5, 56  ) )
				.addTemporalMarker( ( ) -> {
					robot.spinnerLeft.setPower( 0.5 );
				} )
				.waitSeconds( 3.2 )
				.addTemporalMarker( ( ) -> robot.spinnerLeft.setPower( 0 ) )

				//Pickup duck from ground
				/*.addTemporalMarker( ( ) -> {
					robot.intake.setPower( -0.6 );
				} )
				.splineToConstantHeading( new Vector2d( -51, -53 ), Math.toRadians( 270 ) )
				.lineToConstantHeading( new Vector2d( -51,-61 ) )
				.strafeLeft( 2 )
				.strafeRight( 12 )

				.addTemporalMarker( ( ) -> {
					robot.intake.setPower( 0 );
				} )

				//Drop duck in Shipping Hub
				.addTemporalMarker( ( ) -> {
					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
				} )
				.splineToLinearHeading( new Pose2d( -36, -24, Math.toRadians( 0 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( -12 - robot.distanceFromShippingHub( RRHexBot.ShippingHubHeight.LOW ), -24 ) )
				.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )*/

				//Park in Warehouse
				//.lineToConstantHeading( new Vector2d( -36, -24 ) )
				/*.splineToConstantHeading( new Vector2d( -36, -32 ), Math.toRadians( 270 ) )
				.splineToConstantHeading( new Vector2d( -10, -50 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 6, -50 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 10, -40 ), Math.toRadians( 90 ) )*/

				.splineToConstantHeading( new Vector2d( -30, 24 ), Math.toRadians( 270 ) )
				.splineToConstantHeading( new Vector2d( -26, 2 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( -10, 2 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 6, 2 ), Math.toRadians( 90 ) )
				.splineToConstantHeading( new Vector2d( 8, 42 ), Math.toRadians( 90 ) )


				/*.setVelConstraint( new MecanumVelocityConstraint( 50, 17 ) )
				.lineTo( new Vector2d( 36, 42 ) )
				.resetVelConstraint( )
				.splineTo( new Vector2d( 60, 44 ), 0 )*/
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
