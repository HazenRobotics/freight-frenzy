package org.firstinspires.ftc.teamcode.drives;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
	public static double TICKS_PER_REV = 250;
	public static double WHEEL_RADIUS = 38 / 2.0 / 25.4; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	public static double PARALLEL_X = 7.5; // X is the up and down direction
	public static double PARALLEL_Y = 4; // Y is the strafe direction

	public static double PERPENDICULAR_X = 7;
	public static double PERPENDICULAR_Y = 1;

	public static double X_MULTIPLIER = 1; // Multiplier in the X direction
	public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

	// Parallel/Perpendicular to the forward axis
	// Parallel wheel is parallel to the forward axis
	// Perpendicular is perpendicular to the forward axis
	private Encoder parallelEncoder, perpendicularEncoder;

	private RRMecanumDriveHex42 drive;

	public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, RRMecanumDriveHex42 drive) {
		super(Arrays.asList(
				new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
				new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
		));

		this.drive = drive;

		parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
		perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicular"));

		// TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
		perpendicularEncoder.setDirection( Encoder.Direction.REVERSE );
	}

	public static double encoderTicksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@Override
	public double getHeading() {
		return drive.getRawExternalHeading();
	}

	@Override
	public Double getHeadingVelocity() {
		return drive.getExternalHeadingVelocity();
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions() {
		return Arrays.asList(
				encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
				encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
		);
	}

	@NonNull
	@Override
	public List<Double> getWheelVelocities() {
		// TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
		//  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
		//  compensation method

		return Arrays.asList(
				encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
				encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
		);
	}
}