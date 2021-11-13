package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

public class TrackingCameraLocalizer implements Localizer {

	private Pose2d _poseEstimate;
	private Pose2d _poseVelocity;
	private T265Camera.PoseConfidence _confidence;
	private static T265Camera slamra;
	private Pose2d _dumbMathOffset = new Pose2d(  ); //Was in the shop way to late and now hate math, this is the answer to the secrets of the universe.

	/**
	 * Constructs and starts a new VSLAM Camera instance
	 * @param hardwareMap Robot's hardware map
	 * @param cameraFromRobot The position of the camera in relation to the center of the robot
	 * @param loadMap Whether the camera should load an already saved map or not
	 * @param mapName File name of the relocalization map
	 */
	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot, boolean loadMap, String mapName ) {
		if(slamra == null) {
			if(loadMap && new File(String.format("/sdcard/FIRST/localization/maps/%s", mapName)).exists()) {
				slamra = new T265Camera(transformFromRobot( cameraFromRobot ), 0, String.format("/localization/maps/%s", mapName),  hardwareMap.appContext);
			}
			else {
				slamra = new T265Camera(transformFromRobot( cameraFromRobot ), 0,  hardwareMap.appContext);
			}
		}
		try {
			if(!slamra.isStarted()) slamra.start();
		} catch( Exception e ) {

		}
	}

	public TrackingCameraLocalizer(HardwareMap hardwareMap, Pose2d cameraFromRobot, boolean loadMap) {
		this(hardwareMap, cameraFromRobot, loadMap, "map");
	}

	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot) {
		this(hardwareMap, cameraFromRobot, false);
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return _poseEstimate.minus( _dumbMathOffset );

	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		_poseEstimate = pose2d;
		com.arcrobotics.ftclib.geometry.Pose2d newPose = rrPose2dToFtclib(new Pose2d( -pose2d.getX(), -pose2d.getY(), pose2d.getHeading() ));
		Translation2d wanted = newPose.getTranslation().minus(slamra.getLastReceivedCameraUpdate().pose.getTranslation());
		Translation2d given = newPose.getTranslation().minus(slamra.getLastReceivedCameraUpdate().pose.getTranslation()).rotateBy( slamra.getLastReceivedCameraUpdate().pose.getRotation().unaryMinus() );
		_dumbMathOffset = ftclibPose2dToRR( new com.arcrobotics.ftclib.geometry.Pose2d( given.minus( wanted ), new Rotation2d(  ) ) );
		slamra.setPose(newPose);
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return _poseVelocity;
	}

	@Override
	public void update( ) {
		T265Camera.CameraUpdate cameraUpdate = slamra.getLastReceivedCameraUpdate();
		_confidence = cameraUpdate.confidence;

		_poseEstimate = ftclibPose2dToRR( cameraUpdate.pose );
		_poseVelocity = ftclibChassisSpeedsToRR( cameraUpdate.velocity );

	}

	public void exportMap(String fileName) {
		stopCamera();
		File mapFilePath = new File( "/sdcard/FIRST/localization/maps/");
		if (!mapFilePath.exists()) {
			mapFilePath.mkdirs();
		}
		slamra.exportRelocalizationMap( new File(mapFilePath, fileName).getAbsolutePath());
		slamra.free();
		slamra = null;
	}

	public void stopCamera() {
		slamra.stop();
	}

	public void resetCamera() {
		slamra.free();
		slamra = null;
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
		return _confidence;
	}

	public void sendOdometryData(double vx, double vy) {
		slamra.sendOdometry( vy * DistanceUnit.mPerInch, vx * DistanceUnit.mPerInch );
	}


}
