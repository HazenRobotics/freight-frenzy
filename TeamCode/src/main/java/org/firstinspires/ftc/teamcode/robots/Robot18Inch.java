package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDrive18Inch;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class Robot18Inch {

	private RRMecanumDrive18Inch drive;

	private Pose2d currentPose;

	public Robot18Inch( HardwareMap hardwareMap, Pose2d startPose ) {
		drive = new RRMecanumDrive18Inch( hardwareMap );
		currentPose = startPose;
		drive.setPoseEstimate( currentPose );
	}

	public TrajectorySequenceBuilder getTrajectorySequenceBuilder( Pose2d startPose ) {
		return drive.trajectorySequenceBuilder( startPose );
	}

	public TrajectorySequenceBuilder getTrajectorySequenceBuilder( ) {
		updatePose();
		return drive.trajectorySequenceBuilder( currentPose );
	}

	public void followTrajectorySequence( TrajectorySequence trajectory ) {
		drive.followTrajectorySequence( trajectory );
	}

	public void updatePose() {
		currentPose = drive.getPoseEstimate();
	}

}
