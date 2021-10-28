package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class TrackingCameraLocalizer implements Localizer {

	private Pose2d poseEstimate;
	private Pose2d poseVelocity;
	private T265Camera slamra;

	/**
	 * Constructs and starts a new VSLAM Camera instance
	 * @param hardwareMap Robot's hardware map
	 * @param cameraFromRobot The position of the camera in relation to the center of the robot
	 * @param encoderMeasurementCovariance Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
	 */
	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot, double encoderMeasurementCovariance ) {
		slamra = new T265Camera(transformFromRobot( cameraFromRobot ), encoderMeasurementCovariance, hardwareMap.appContext);
		slamra.start();
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return poseEstimate;
	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		slamra.setPose( rrPose2dToFtclib( pose2d ) );

	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return poseVelocity;
	}

	@Override
	public void update( ) {
		T265Camera.CameraUpdate cameraUpdate = slamra.getLastReceivedCameraUpdate();
		poseEstimate = ftclibPose2dToRR( cameraUpdate.pose );
		poseVelocity = ftclibChassisSpeedsToRR( cameraUpdate.velocity );
	}

	public void freeCamera() {
		if(slamra.isStarted()) {
			slamra.stop();
		}
		slamra.free();
	}

	private com.arcrobotics.ftclib.geometry.Pose2d rrPose2dToFtclib(Pose2d rrPose) {
		return new com.arcrobotics.ftclib.geometry.Pose2d( rrPose.getY() / 39.37, -rrPose.getX() / 39.37, new Rotation2d( rrPose.getHeading() ) );
	}
	private Pose2d ftclibPose2dToRR( com.arcrobotics.ftclib.geometry.Pose2d ftclibPose ) {
		return new Pose2d(ftclibPose.getY() * 39.37, -ftclibPose.getX() * 39.37, ftclibPose.getHeading());
	}
	private Pose2d ftclibChassisSpeedsToRR( ChassisSpeeds ftclibChassisSpeeds ) {
		return new Pose2d(ftclibChassisSpeeds.vyMetersPerSecond * 39.37
				, -ftclibChassisSpeeds.vxMetersPerSecond * 39.37
				, ftclibChassisSpeeds.omegaRadiansPerSecond);
	}
	private Transform2d transformFromRobot(Pose2d item) {
		return new Transform2d( rrPose2dToFtclib( item ), new com.arcrobotics.ftclib.geometry.Pose2d(  ) );
	}
}
