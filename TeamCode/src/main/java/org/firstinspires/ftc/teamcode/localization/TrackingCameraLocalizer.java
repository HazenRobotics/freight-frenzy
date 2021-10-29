package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class TrackingCameraLocalizer implements Localizer {

	private Pose2d poseEstimate;
	private Pose2d poseVelocity;
	private T265Camera.PoseConfidence confidence;
	private static T265Camera slamra;

	private com.arcrobotics.ftclib.geometry.Pose2d previousPose;
	private com.arcrobotics.ftclib.geometry.Pose2d previousAbsolutePose;

	/**
	 * Constructs and starts a new VSLAM Camera instance
	 * @param hardwareMap Robot's hardware map
	 * @param cameraFromRobot The position of the camera in relation to the center of the robot
	 */
	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot ) {
		if(slamra == null) {
			slamra = new T265Camera(transformFromRobot( cameraFromRobot ), 0, hardwareMap.appContext);
		}
		try {
			if(!slamra.isStarted()) slamra.start();
		} catch( Exception e ) {
			setPoseEstimate( new Pose2d(  ) );
		}
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return poseEstimate;
	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		previousAbsolutePose = rrPose2dToFtclib( pose2d );
		poseEstimate = ftclibPose2dToRR( previousAbsolutePose );
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return poseVelocity;
	}

	@Override
	public void update( ) {
		T265Camera.CameraUpdate cameraUpdate = slamra.getLastReceivedCameraUpdate();
		confidence = cameraUpdate.confidence;

		poseEstimate = ftclibPose2dToRR( relativePoseToAbsolutePose( cameraUpdate.pose ) );

	}

	public void stopCamera() {
		slamra.stop();
	}

	private com.arcrobotics.ftclib.geometry.Pose2d rrPose2dToFtclib(Pose2d rrPose) {
		return new com.arcrobotics.ftclib.geometry.Pose2d( rrPose.getY() / 39.37, -rrPose.getX() / 39.37,  new Rotation2d( rrPose.getHeading() ) );
	}
	private Pose2d ftclibPose2dToRR( com.arcrobotics.ftclib.geometry.Pose2d ftclibPose ) {
		return new Pose2d(ftclibPose.getY() * 39.37, -ftclibPose.getX() * 39.37,  ftclibPose.getHeading());
	}
	private Pose2d ftclibChassisSpeedsToRR( ChassisSpeeds ftclibChassisSpeeds ) {
		return new Pose2d( ftclibChassisSpeeds.vyMetersPerSecond * 39.37
				,-ftclibChassisSpeeds.vxMetersPerSecond * 39.37
				, ftclibChassisSpeeds.omegaRadiansPerSecond);
	}
	private Transform2d transformFromRobot(Pose2d item) {
		return new Transform2d( new com.arcrobotics.ftclib.geometry.Pose2d(  ),  rrPose2dToFtclib( item ) );
	}

	public T265Camera.PoseConfidence getPoseConfidence( ) {
		return confidence;
	}

	private com.arcrobotics.ftclib.geometry.Pose2d relativePoseToAbsolutePose( com.arcrobotics.ftclib.geometry.Pose2d pose ) {
		double dx = pose.getX() - previousPose.getX();
		double dy = pose.getY() - previousPose.getY();
		double dtheta = pose.getHeading() - previousPose.getHeading();
		double theta = Math.atan( dtheta );

		previousPose = pose;

		double absoluteDY = Math.sin( theta ) * dy;
		double absoluteDX = Math.cos( theta ) * dx;

		double absoluteX = previousAbsolutePose.getX() + absoluteDX;
		double absoluteY = previousAbsolutePose.getY() + absoluteDY;
		com.arcrobotics.ftclib.geometry.Pose2d absolutePose = new com.arcrobotics.ftclib.geometry.Pose2d( absoluteX, absoluteY, new Rotation2d( pose.getHeading() ) );

		previousAbsolutePose = absolutePose;

		return absolutePose;

	}


}
