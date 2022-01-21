package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.drives.TwoWheelTrackingLocalizerTippy;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.Logger;

import java.util.function.Supplier;

public class FusionLocalizer implements Localizer {

	TrackingCameraLocalizer cameraLocalizer;
	TwoWheelTrackingLocalizerTippy wheelLocalizer;
	MecanumDrive.MecanumLocalizer driveLocalizer;

	boolean cameraReady = false;
	boolean deadwheelsDisabled = false;
	long cameraReadyTime = 0;

	Thread checkDeadwheelsThread;

	public FusionLocalizer( HardwareMap hardwareMap, RRMecanumDriveTippy42 drive, Pose2d cameraFromRobot ) {
		driveLocalizer = new MecanumDrive.MecanumLocalizer( drive );
		wheelLocalizer = new TwoWheelTrackingLocalizerTippy( hardwareMap, drive );
		cameraLocalizer = new TrackingCameraLocalizer( hardwareMap, cameraFromRobot, drive );
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return cameraReady ? cameraLocalizer.getPoseEstimate( ) : !deadwheelsDisabled ? wheelLocalizer.getPoseEstimate( ) : driveLocalizer.getPoseEstimate( );
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
		return cameraReady ? cameraLocalizer.getPoseVelocity( ) : !deadwheelsDisabled ? wheelLocalizer.getPoseVelocity( ) : driveLocalizer.getPoseVelocity( );
	}

	@Override
	public void update( ) {
//		Logger.writeAFile( "PosGraph.csv", "updating", true, false );
		driveLocalizer.update( );
		wheelLocalizer.update( );
		cameraLocalizer.update( );
		if( cameraReady && !deadwheelsDisabled ) {
//			String writeText = "" + (System.currentTimeMillis() / 1000.0 );
//			writeText += ", " + cameraLocalizer.getPoseEstimate( ).getY( );
			cameraLocalizer.sendOdometryData( wheelLocalizer.getPoseVelocity( ) );
//			writeText += ", " + wheelLocalizer.getPoseEstimate( ).getY( );
//			writeText += ", " + cameraLocalizer.getPoseEstimate( ).getY( );
//			Logger.writeAFile( "PosGraph.csv", writeText, true, false );
		} else {
			if( !cameraReady && cameraLocalizer.getPoseConfidence( ) == T265Camera.PoseConfidence.High && cameraReadyTime == 0 ) {
				cameraReadyTime = System.currentTimeMillis();
			}
			else if( !cameraReady && cameraLocalizer.getPoseConfidence( ) == T265Camera.PoseConfidence.High && System.currentTimeMillis() > cameraReadyTime + 1000) {
				cameraLocalizer.setPoseEstimate( !deadwheelsDisabled ? wheelLocalizer.getPoseEstimate( ) : driveLocalizer.getPoseEstimate( ) );
				cameraReady = true;
			}
		}
	}

	/**
	 * Sets the function that should run to check whether the dead wheels should be disabled or not, and starts a thread executing that function
	 *
	 * @param checkFunc Function that returns true if the dead wheels should be disabled, and false if they should not be disabled
	 */
	public void setDeadwheelsDisabledCheck( Supplier<Boolean> checkFunc ) {
		if( checkDeadwheelsThread != null && checkDeadwheelsThread.isAlive( ) ) {
			checkDeadwheelsThread.interrupt( );
		}
		checkDeadwheelsThread = new Thread( ( ) -> {
			while( true ) {
				if( checkFunc.get( ) && !deadwheelsDisabled ) {
					deadwheelsDisabled = true;
					driveLocalizer.setPoseEstimate( wheelLocalizer.getPoseEstimate( ) );
				} else if( !checkFunc.get( ) && deadwheelsDisabled) {
					wheelLocalizer.setPoseEstimate( cameraReady ? cameraLocalizer.getPoseEstimate( ) : driveLocalizer.getPoseEstimate( ) );
					deadwheelsDisabled = false;
				}
				try {
					Thread.sleep( 50 );
				} catch( InterruptedException e ) {
					e.printStackTrace( );
				}
			}
		} );
		checkDeadwheelsThread.start( );
	}

	public void stopDeadwheelsDisabledCheck( ) {
		checkDeadwheelsThread.interrupt( );
	}

	public void stopCamera( ) {
		cameraLocalizer.stopCamera( );
	}

	public void setCameraFrameOfReference( TrackingCameraLocalizer.CardinalDirection frameOfReference ) {
		cameraLocalizer.setFrameOfReference( frameOfReference );
	}

	public T265Camera.PoseConfidence getCameraPoseConfidence() {
		return cameraLocalizer.getPoseConfidence();
	}

}
