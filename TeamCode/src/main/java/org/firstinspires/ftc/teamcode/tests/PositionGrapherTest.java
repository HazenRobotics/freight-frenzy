package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous(name = "PositionGrapherTest", group = "Test")
//@Disabled
public class PositionGrapherTest extends LinearOpMode {

	RRTippyBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new RRTippyBot( this, true );

		telemetry.addLine( "Finished init" );
		telemetry.update( );

		waitForStart( );

		Logger.writeAFile( "PosGraph.csv", "A, B, C, D", false, false );

		robot.drive.setPoseEstimate( new Pose2d( 0, 64, Math.toRadians( 270 ) ) );

		TrajectorySequence beforeDuckPickup = robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )

				.lineTo( new Vector2d( 0, -16 ) )
				.build( );

		robot.drive.followTrajectorySequence( beforeDuckPickup );

	}
}
