package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous(name = "PositionGrapherTest", group = "Test")
@Disabled
public class PositionGrapherTest extends LinearOpMode {

	RRTippyBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new RRTippyBot( this, true );

		robot.drive.setCameraFrameOfReference( TrackingCameraLocalizer.CardinalDirection.SOUTH );

		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			robot.drive.update( );
		} while( !isStopRequested( ) && !isStarted( ) && robot.drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );

		new Thread(() -> {
			while( !isStopRequested( ) ) {
				robot.drive.update( );
			}
		} ).start( );

		telemetry.addLine( "Finished init" );
		telemetry.update( );

		waitForStart( );

		Logger.writeAFile( "PosGraph.csv", "Time, Camera, Wheel, Fusion", false, false );

		robot.drive.setPoseEstimate( new Pose2d( 0, 64, Math.toRadians( 270 ) ) );

		TrajectorySequence beforeDuckPickup = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				.lineTo( new Vector2d( 0, -16 ) )
				.build( );

		robot.drive.followTrajectorySequence( beforeDuckPickup );

	}
}
