package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;

@Autonomous(name="Fetch", group="outreach")
public class FetchGame extends LinearOpMode {

	private RRTippyBot robot;
	private RobotState state = RobotState.SEARCH;
	private RobotState previousState = null;
	private Vector2d objectPosition = null;

	private enum RobotState {
		SEARCH,
		DRIVE,
		PICKUP,
		RETURN
	}

	private TrajectorySequence searchTrajectory;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new RRTippyBot( this, true );
		robot.drive.setPoseEstimate( new Pose2d( ) );

		searchTrajectory = robot.drive.trajectorySequenceBuilder( new Pose2d(  ) )
				.turn( Math.toRadians( 45 ) )
				.turn( Math.toRadians( -90 ) )
				.turn( Math.toRadians( 45 ) )
				.build();

		robot.startDuckScanning( 500 );

		waitForStart();

		while( opModeIsActive() ) {
			//state machine!!!!
			switch( state ) {
				case SEARCH:
					//if robot is idle or was forced into searching mode
					if(!robot.drive.isBusy() || previousState != state) {
						robot.drive.cancelTrajectorySequence();
						robot.drive.followTrajectorySequenceAsync( searchTrajectory );
						previousState = state;
					}
					//if the robot finds an object
					if(robot.getLastIdentifiedPosition() != null) {
						objectPosition = robot.getLastIdentifiedPosition();
						state = RobotState.DRIVE;
					}
					break;

				case DRIVE:
					if(previousState != state) {
						robot.drive.cancelTrajectorySequence();
						robot.drive.followTrajectorySequenceAsync(
								robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate( ) )
										.addTemporalMarker( ( ) -> {
											robot.intake.setPower( 0.5 );
										} )
										.splineToLinearHeading( new Pose2d( objectPosition, Math.toRadians( 0 ) ), Math.toRadians( 180 ) )
										.build( )
						);
						previousState = state;
					}
					if(!robot.drive.isBusy()) {
						state = RobotState.PICKUP;
					}
					break;
				case PICKUP:
					if(previousState != state) {
						robot.intake.intakeBlocks( 1, 1000 );
					}
					break;
				case RETURN:
					if(previousState != state) {
						robot.drive.followTrajectorySequenceAsync(
								robot.drive.trajectorySequenceBuilder( robot.drive.getPoseEstimate() )
										.splineToLinearHeading( new Pose2d(  ), Math.toRadians( 0 ) )
										.addTemporalMarker( () -> {
											robot.lift.setHeightVelocityLinear( 1800, 22 );
											robot.dumpBucket();
											robot.lift.setDefaultHeightVel( 1800 );
										} )
										.build()
						);
						previousState = state;
					}
					break;
				default:
					state = RobotState.SEARCH;
			}

			robot.drive.update();

			telemetry.addData( "Current State", state );
			telemetry.update();
		}

		//cleanup
		robot.stopDuckScanning();
	}
}
