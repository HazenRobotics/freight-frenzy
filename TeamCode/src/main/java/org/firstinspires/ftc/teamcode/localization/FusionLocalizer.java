package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.drives.TwoWheelTrackingLocalizerTippy;

import java.util.function.Supplier;

public class FusionLocalizer implements Localizer {

	TrackingCameraLocalizer cameraLocalizer;
	TwoWheelTrackingLocalizerTippy wheelLocalizer;
	MecanumDrive.MecanumLocalizer driveLocalizer;

	boolean cameraReady = false;
	boolean deadwheelsDisabled = false;

	Thread checkDeadwheelsThread;

	public FusionLocalizer( HardwareMap hardwareMap, RRMecanumDriveTippy42 drive, Pose2d cameraFromRobot) {
		driveLocalizer = new MecanumDrive.MecanumLocalizer( drive );
		wheelLocalizer = new TwoWheelTrackingLocalizerTippy( hardwareMap, drive );
		cameraLocalizer = new TrackingCameraLocalizer( hardwareMap, cameraFromRobot );
	}
	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return cameraReady ? cameraLocalizer.getPoseEstimate() : !deadwheelsDisabled ? wheelLocalizer.getPoseEstimate() : driveLocalizer.getPoseEstimate();
	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		cameraLocalizer.setPoseEstimate( pose2d );
		wheelLocalizer.setPoseEstimate( pose2d );
		driveLocalizer.setPoseEstimate( pose2d );
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return cameraReady ? cameraLocalizer.getPoseVelocity() : !deadwheelsDisabled ? wheelLocalizer.getPoseVelocity() : driveLocalizer.getPoseVelocity();
	}

	@Override
	public void update( ) {
		driveLocalizer.update();
		wheelLocalizer.update();
		cameraLocalizer.update();
		if(cameraReady && !deadwheelsDisabled) {
			cameraLocalizer.sendOdometryData( wheelLocalizer.getPoseVelocity() );
		}
		else {
			if( cameraLocalizer.getPoseConfidence() == T265Camera.PoseConfidence.High ) {
				cameraReady = true;
				cameraLocalizer.setPoseEstimate( !deadwheelsDisabled ? wheelLocalizer.getPoseEstimate() : driveLocalizer.getPoseEstimate() );
			}
		}
	}

	/**
	 * Sets the function that should run to check whether the dead wheels should be disabled or not, and starts a thread executing that function
	 * @param checkFunc Function that returns true if the dead wheels should be disabled, and false if they should not be disabled
	 */
	public void setDeadwheelsDisabledCheck( Supplier<Boolean> checkFunc ) {
		if(checkDeadwheelsThread != null && checkDeadwheelsThread.isAlive()) {
			checkDeadwheelsThread.interrupt();
		}
		checkDeadwheelsThread = new Thread( () -> {
			while(true) {
				while( !deadwheelsDisabled ) {
					deadwheelsDisabled = checkFunc.get();
				}

				driveLocalizer.setPoseEstimate( wheelLocalizer.getPoseEstimate() );

				while( deadwheelsDisabled ) {
					deadwheelsDisabled = checkFunc.get();
				}

				wheelLocalizer.setPoseEstimate( cameraReady ? cameraLocalizer.getPoseEstimate() : driveLocalizer.getPoseEstimate() );
			}
		});
		checkDeadwheelsThread.start();
	}

	public void stopDeadwheelsDisabledCheck() {
		checkDeadwheelsThread.interrupt();
	}

	public void stopCamera() {
		cameraLocalizer.stopCamera();
	}
}
