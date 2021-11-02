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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

public class TrackingCameraLocalizer implements Localizer {

	private Pose2d _poseEstimate;
	private Pose2d _poseVelocity;
	private T265Camera.PoseConfidence confidence;
	private static T265Camera slamra;

	/**
	 * Constructs and starts a new VSLAM Camera instance
	 * @param hardwareMap Robot's hardware map
	 * @param cameraFromRobot The position of the camera in relation to the center of the robot
	 */
	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot ) {
		if(slamra == null) {
			File map = new File("/localization/maps/map");
			if(map.exists()) {
				slamra = new T265Camera(transformFromRobot( cameraFromRobot ), 0, "/localization/maps/map",  hardwareMap.appContext);
			}
			else {
				slamra = new T265Camera(transformFromRobot( cameraFromRobot ), 0,  hardwareMap.appContext);
			}
		}
		try {
			if(!slamra.isStarted()) slamra.start();
		} catch( Exception e ) {

		}
		setPoseEstimate( new Pose2d(  ) );
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return _poseEstimate;
	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		_poseEstimate = pose2d;
		slamra.setPose(rrPose2dToFtclib(pose2d));
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return _poseVelocity;
	}

	@Override
	public void update( ) {
		T265Camera.CameraUpdate cameraUpdate = slamra.getLastReceivedCameraUpdate();
		confidence = cameraUpdate.confidence;

		_poseEstimate = ftclibPose2dToRR( cameraUpdate.pose );
		_poseVelocity = ftclibChassisSpeedsToRR( cameraUpdate.velocity );

	}

	public void stopCamera() {
		slamra.exportRelocalizationMap( "/localization/maps/map" );
		slamra.stop();
		slamra.free();
	}

	private com.arcrobotics.ftclib.geometry.Pose2d rrPose2dToFtclib(Pose2d rrPose) {
		return new com.arcrobotics.ftclib.geometry.Pose2d( -rrPose.getY() * DistanceUnit.mPerInch, rrPose.getX() * DistanceUnit.mPerInch,  new Rotation2d( rrPose.getHeading() ) );
	}
	private Pose2d ftclibPose2dToRR( com.arcrobotics.ftclib.geometry.Pose2d ftclibPose ) {
		return new Pose2d(-ftclibPose.getY() /DistanceUnit.mPerInch,ftclibPose.getX() / DistanceUnit.mPerInch,   ftclibPose.getHeading());
	}
	private Pose2d ftclibChassisSpeedsToRR( ChassisSpeeds ftclibChassisSpeeds ) {
		return new Pose2d( -ftclibChassisSpeeds.vyMetersPerSecond / DistanceUnit.mPerInch
				,ftclibChassisSpeeds.vxMetersPerSecond / DistanceUnit.mPerInch
				, ftclibChassisSpeeds.omegaRadiansPerSecond);
	}
	private Transform2d transformFromRobot(Pose2d item) {
		return new Transform2d( new com.arcrobotics.ftclib.geometry.Pose2d(  ),  rrPose2dToFtclib( item ) );
	}

	public T265Camera.PoseConfidence getPoseConfidence( ) {
		return confidence;
	}


}
