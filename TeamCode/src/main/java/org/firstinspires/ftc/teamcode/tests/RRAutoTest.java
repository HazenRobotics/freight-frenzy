package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.Robot18Inch;

@Autonomous(name = "Road Runner Test", group = "Test")
public class RRAutoTest extends LinearOpMode {

	Robot18Inch robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new Robot18Inch( hardwareMap, new Pose2d( -36, -63 ) );

		waitForStart();

		//TrajectorySequence trajectorySequence = robot.getTrajectorySequenceBuilder();

	}
}
