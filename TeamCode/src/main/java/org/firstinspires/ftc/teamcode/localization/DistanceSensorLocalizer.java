package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;

public class DistanceSensorLocalizer implements Localizer {

	private Rev2mDistanceSensor left;
	Vector2d leftPosition;
	private Rev2mDistanceSensor right;
	Vector2d rightPosition;
	private Rev2mDistanceSensor back;
	Vector2d backPosition;

	private RRMecanumDriveTippy42 drive;

	private Pose2d poseEstimate = new Pose2d(  );
	private long poseEstimateTime = System.currentTimeMillis();
	private double headingOffset = 0;

	private Pose2d previousPoseEstimate = new Pose2d(  );
	private long previousPoseEstimateTime = System.currentTimeMillis();

	public DistanceSensorLocalizer( HardwareMap hardwareMap, String leftSensorName, Vector2d leftPosition, String rightSensorName, Vector2d rightPosition, String backSensorName, Vector2d backPosition, RRMecanumDriveTippy42 drive ) {
		this.drive = drive;
		left = hardwareMap.get( Rev2mDistanceSensor.class, leftSensorName );
		this.leftPosition = leftPosition;
		right = hardwareMap.get( Rev2mDistanceSensor.class, rightSensorName );
		this.rightPosition = rightPosition;
		back = hardwareMap.get( Rev2mDistanceSensor.class, backSensorName );
		this.backPosition = backPosition;
	}

	public DistanceSensorLocalizer( HardwareMap hardwareMap, Vector2d leftPosition, Vector2d rightPosition, Vector2d backPosition, RRMecanumDriveTippy42 drive ) {
		this(hardwareMap, "leftDistance", leftPosition, "rightDistance", rightPosition, "backDistance", backPosition, drive);
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
		double robotHeading = drive.getRawExternalHeading();
		double yFromLeftSensor = 72 - left.getDistance( DistanceUnit.INCH ) - leftPosition.getY();
		double yFromRightSensor = -72 + right.getDistance( DistanceUnit.INCH ) + rightPosition.getY();
		double robotX = -72 + back.getDistance( DistanceUnit.INCH ) + backPosition.getX() + (backPosition.getY() * Math.tan(robotHeading));

		//figure out which sensor is reading
		double robotY = left.getDistance( DistanceUnit.INCH ) < right.getDistance( DistanceUnit.INCH ) ? yFromLeftSensor + (leftPosition.getX() * Math.tan(robotHeading)) : yFromRightSensor + (rightPosition.getX() * Math.tan(robotHeading));


		double absoluteX = robotX * Math.cos( robotHeading ) - robotY * Math.sin( robotHeading );
		double absoluteY = robotY * Math.cos( robotHeading ) + robotX * Math.sin( robotHeading );

		poseEstimate = new Pose2d( absoluteX, absoluteY, robotHeading );
		poseEstimateTime = System.currentTimeMillis() / 1000;

	}
}
