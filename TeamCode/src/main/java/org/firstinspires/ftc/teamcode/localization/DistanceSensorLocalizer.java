package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;

public class DistanceSensorLocalizer implements Localizer {

	private Rev2mDistanceSensor left;
	private double leftDistanceFromRobotCenter;
	private Rev2mDistanceSensor right;
	double rightDistanceFromRobotCenter;
	private Rev2mDistanceSensor back;
	double backDistanceFromRobotCenter;

	private RRMecanumDriveTippy42 drive;

	private Pose2d poseEstimate = new Pose2d(  );
	private long poseEstimateTime = System.currentTimeMillis();
	private double headingOffset = 0;

	private Pose2d previousPoseEstimate = new Pose2d(  );
	private long previousPoseEstimateTime = System.currentTimeMillis();

	public DistanceSensorLocalizer( HardwareMap hardwareMap, String leftSensorName, double leftDistanceFromRobotCenter, String rightSensorName, double rightDistanceFromRobotCenter, String backSensorName, double backDistanceFromRobotCenter, RRMecanumDriveTippy42 drive ) {
		this.drive = drive;
		left = hardwareMap.get( Rev2mDistanceSensor.class, leftSensorName );
		this.leftDistanceFromRobotCenter = leftDistanceFromRobotCenter;
		right = hardwareMap.get( Rev2mDistanceSensor.class, rightSensorName );
		this.rightDistanceFromRobotCenter = rightDistanceFromRobotCenter;
		back = hardwareMap.get( Rev2mDistanceSensor.class, backSensorName );
		this.backDistanceFromRobotCenter = backDistanceFromRobotCenter;
	}

	public DistanceSensorLocalizer( HardwareMap hardwareMap, double leftDistanceFromRobotCenter, double rightDistanceFromRobotCenter, double backDistanceFromRobotCenter, RRMecanumDriveTippy42 drive ) {
		this(hardwareMap, "leftDistance", leftDistanceFromRobotCenter, "rightDistance", rightDistanceFromRobotCenter, "backDistance", backDistanceFromRobotCenter, drive);
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return poseEstimate.minus( new Pose2d( 0, 0, headingOffset ) );
	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		//set pose heading offset
		headingOffset = drive.getRawExternalHeading( ) - pose2d.getHeading();

	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return new Pose2d( (poseEstimate.getX() - previousPoseEstimate.getX()) / (poseEstimateTime - previousPoseEstimateTime), (poseEstimate.getY() - previousPoseEstimate.getY()) / (poseEstimateTime - previousPoseEstimateTime), drive.getExternalHeadingVelocity() );
	}

	@Override
	public void update( ) {
		previousPoseEstimate = poseEstimate;
		previousPoseEstimateTime = poseEstimateTime;
		double yFromLeftSensor = 72 - left.getDistance( DistanceUnit.INCH ) - leftDistanceFromRobotCenter;
		double yFromRightSensor = -72 + right.getDistance( DistanceUnit.INCH ) + rightDistanceFromRobotCenter;
		double robotX = -72 + back.getDistance( DistanceUnit.INCH ) + backDistanceFromRobotCenter;

		//figure out which sensor is reading
		double robotY = left.getDistance( DistanceUnit.INCH ) < right.getDistance( DistanceUnit.INCH ) ? yFromLeftSensor : yFromRightSensor;
		double robotHeading = drive.getRawExternalHeading();

		double absoluteX = robotX * Math.cos( robotHeading ) - robotY * Math.sin( robotHeading );
		double absoluteY = robotY * Math.cos( robotHeading ) + robotX * Math.sin( robotHeading );

		poseEstimate = new Pose2d( absoluteX, absoluteY, robotHeading );
		poseEstimateTime = System.currentTimeMillis() / 1000;

	}
}
