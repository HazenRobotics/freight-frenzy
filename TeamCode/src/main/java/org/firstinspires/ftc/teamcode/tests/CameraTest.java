package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;

//@Autonomous
public class CameraTest extends LinearOpMode {

	RRMecanumDriveHex42 drive;
	T265Camera slamra;

	@Override
	public void runOpMode( ) throws InterruptedException {
		drive = new RRMecanumDriveHex42( hardwareMap );
		slamra = new T265Camera( new Transform2d(  ), 0, hardwareMap.appContext );
		slamra.start();

		new Thread( () -> {
			while(true) {
				telemetry.addData( "PoseConfidence", slamra.getLastReceivedCameraUpdate().confidence );
				telemetry.update( );
			}
		} ).start();

		waitForStart();

		drive.followTrajectorySequence( drive.trajectorySequenceBuilder( new Pose2d(  ) )
				.setVelConstraint( new MecanumVelocityConstraint( 55, 17 ) )
				.setAccelConstraint( new ProfileAccelerationConstraint( 90 ))
				.forward( 12 )
				.back( 12.5 )
				.build());
		while( !isStopRequested() );
	}
}
