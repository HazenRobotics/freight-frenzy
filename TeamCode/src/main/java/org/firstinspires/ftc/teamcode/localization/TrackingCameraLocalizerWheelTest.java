package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class TrackingCameraLocalizerWheelTest extends TwoTrackingWheelLocalizer {

	public TrackingCameraLocalizerWheelTest( @NonNull List<Pose2d> wheelPoses ) {
		super( wheelPoses );
	}

	@Override
	public double getHeading( ) {
		return 0;
	}
	@Override
	public Double getHeadingVelocity() {
		return 0d;
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions( ) {
		return null;
	}

	@Override
	public List<Double> getWheelVelocities() {
		return null;
	}

	public static double metersToInches(double meters) {
		return meters * DistanceUnit.mPerInch;
	}
}
