package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.drives.TwoWheelTrackingLocalizer;

public class FusionLocalizer implements Localizer {

	TrackingCameraLocalizer cameraLocalizer;
	TwoWheelTrackingLocalizer wheelLocalizer;

	@NonNull
	@Override
	public Pose2d getPoseEstimate( ) {
		return cameraLocalizer.getPoseEstimate();
	}

	@Override
	public void setPoseEstimate( @NonNull Pose2d pose2d ) {
		cameraLocalizer.setPoseEstimate( pose2d );
		wheelLocalizer.setPoseEstimate( pose2d );
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity( ) {
		return cameraLocalizer.getPoseVelocity();
	}

	@Override
	public void update( ) {
		wheelLocalizer.update();
		cameraLocalizer.sendOdometryData( wheelLocalizer.getPoseVelocity().getX(), wheelLocalizer.getPoseVelocity().getY() );
		cameraLocalizer.update();
	}
}
