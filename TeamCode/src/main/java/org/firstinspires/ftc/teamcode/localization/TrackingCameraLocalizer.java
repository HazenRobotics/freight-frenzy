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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

public class TrackingCameraLocalizer implements Localizer {

	private Pose2d _poseEstimate;
	private Pose2d _poseVelocity;
	private T265Camera.PoseConfidence _confidence;
	private static T265Camera slamra;
	private Pose2d _offset = new Pose2d(  );
	private CardinalDirection _frameOfReference;
	public static double ODOMETRY_COVARIANCE = 0.3; // 0 = odometry, 1 = camera

	public enum CardinalDirection {
		NORTH,
		SOUTH,
		EAST,
		WEST
	}

	/**
	 * Constructs and starts a new VSLAM Camera instance
	 * @param hardwareMap Robot's hardware map
	 * @param cameraFromRobot The position of the camera in relation to the center of the robot
	 * @param loadMap Whether the camera should load an already saved map or not
	 * @param mapName File name of the relocalization map
	 * @param frameOfReference Direction of the frame of reference for the robot
	 */
	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot, boolean loadMap, String mapName, CardinalDirection frameOfReference ) {
		_frameOfReference = frameOfReference;
		if(slamra == null) {
			if(loadMap && new File(String.format("/sdcard/FIRST/localization/maps/%s", mapName)).exists())
				slamra = new T265Camera(transformFromRobot( cameraFromRobot ), ODOMETRY_COVARIANCE, String.format("/localization/maps/%s", mapName),  hardwareMap.appContext);
			else
				slamra = new T265Camera(transformFromRobot( cameraFromRobot ), ODOMETRY_COVARIANCE,  hardwareMap.appContext);

		}
		try {
			if(!slamra.isStarted()) slamra.start();
		} catch( Exception e ) {

		}
	}

	public TrackingCameraLocalizer(HardwareMap hardwareMap, Pose2d cameraFromRobot, boolean loadMap, CardinalDirection frameOfReference) {
		this(hardwareMap, cameraFromRobot, loadMap, "map", frameOfReference);
	}

	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot, CardinalDirection frameOfReference) {
		this(hardwareMap, cameraFromRobot, false, frameOfReference);
	}

	public TrackingCameraLocalizer( HardwareMap hardwareMap, Pose2d cameraFromRobot) {
		this(hardwareMap, cameraFromRobot, false, CardinalDirection.EAST);
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return correctFrameOfReference(_poseEstimate).minus( _offset );
	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		_offset = correctFrameOfReference( getCameraUpdate().pose ).minus( pose2d );
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return correctFrameOfReference( _poseVelocity );
	}

	@Override
	public void update( ) {
		CameraUpdateRR cameraUpdate = getCameraUpdate();
		_confidence = cameraUpdate.confidence;

		_poseEstimate = cameraUpdate.pose;
		_poseVelocity = cameraUpdate.velocity;

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


	private Transform2d transformFromRobot(Pose2d item) {
		return new Transform2d( new com.arcrobotics.ftclib.geometry.Pose2d(  ),  rrPose2dToFtclib( item ) );
	}

	public T265Camera.PoseConfidence getPoseConfidence( ) {
		return _confidence;
	}

	/**
	 * Sends odometry data to the sensor fusion algorithm
	 * @param velocity velocity in the x and y directions
	 */
	public void sendOdometryData(Pose2d velocity) {
		velocity = reverseFrameOfReference( velocity );
		com.arcrobotics.ftclib.geometry.Pose2d ftcLibPose = rrPose2dToFtclib( velocity );
		slamra.sendOdometry( ftcLibPose.getX(), ftcLibPose.getY() );
	}

	private com.arcrobotics.ftclib.geometry.Pose2d rrPose2dToFtclib(Pose2d rrPose) {
		return new com.arcrobotics.ftclib.geometry.Pose2d( rrPose.getY() * DistanceUnit.mPerInch, -rrPose.getX() * DistanceUnit.mPerInch,  new Rotation2d( rrPose.getHeading() ) );
	}

	/**
	 * Sets the frame of reference for the camera
	 * @param pose pose to correct
	 */
	public Pose2d correctFrameOfReference( Pose2d pose ) {
		if(_frameOfReference == CardinalDirection.NORTH) {
			//swap (x,y) to (-y,x)
			return new Pose2d( -pose.getY(), pose.getX(), pose.getHeading() );
		} else if(_frameOfReference == CardinalDirection.WEST) {
			//swap (x,y) to (-x,-y)
			return new Pose2d( -pose.getX(), -pose.getY(), pose.getHeading() );
		} else if(_frameOfReference == CardinalDirection.SOUTH) {
			//swap (x,y) to (y,-x)
			return new Pose2d( pose.getY(), -pose.getX(), pose.getHeading() );
		}
		//If EAST do nothing because it is the same FOR as the camera
		return pose;
	}

	private Pose2d reverseFrameOfReference( Pose2d pose) {
		if(_frameOfReference == CardinalDirection.NORTH) {
			//swap (x,y) to (-y,x)
			return new Pose2d( pose.getY(), -pose.getX(), pose.getHeading() );
		} else if(_frameOfReference == CardinalDirection.WEST) {
			//swap (x,y) to (-x,-y)
			return new Pose2d( -pose.getX(), -pose.getY(), pose.getHeading() );
		} else if(_frameOfReference == CardinalDirection.SOUTH) {
			//swap (x,y) to (y,-x)
			return new Pose2d( -pose.getY(), pose.getX(), pose.getHeading() );
		}
		//If EAST do nothing because it is the same FOR as the camera
		return pose;
	}

	private CameraUpdateRR getCameraUpdate() {
		return new CameraUpdateRR( slamra.getLastReceivedCameraUpdate() );
	}

	public void setFrameOfReference(CardinalDirection frameOfReference) {
		this._frameOfReference = frameOfReference;
	}



	class CameraUpdateRR {

		public Pose2d pose;
		public Pose2d velocity;
		public T265Camera.PoseConfidence confidence;

		public CameraUpdateRR( T265Camera.CameraUpdate cameraUpdate ) {
			pose = ftclibPose2dToRR( cameraUpdate.pose );
			velocity = ftclibChassisSpeedsToRR( cameraUpdate.velocity );
			confidence = cameraUpdate.confidence;
		}


		private Pose2d ftclibPose2dToRR( com.arcrobotics.ftclib.geometry.Pose2d ftclibPose ) {
			return new Pose2d(ftclibPose.getY() /DistanceUnit.mPerInch,-ftclibPose.getX() / DistanceUnit.mPerInch,   ftclibPose.getHeading());
		}
		private Pose2d ftclibChassisSpeedsToRR( ChassisSpeeds ftclibChassisSpeeds ) {
			return new Pose2d( -ftclibChassisSpeeds.vyMetersPerSecond / DistanceUnit.mPerInch
					,ftclibChassisSpeeds.vxMetersPerSecond / DistanceUnit.mPerInch
					, ftclibChassisSpeeds.omegaRadiansPerSecond);
		}
	}


}
