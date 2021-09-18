package org.firstinspires.ftc.teamcode.drives;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.road_runner.util.Encoder;

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
@Config
public class RRTwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

	public static double TICKS_PER_REV = 4000; // old = 250
	public static double WHEEL_RADIUS = 1.4960629921 / 2; // odometry wheel 38mm // in // old = 0.748
	public static double GEAR_RATIO = 4; // output (wheel) speed / input (encoder) speed

	public static double PARALLEL_X = 3.25; // X is the up and down direction // old 3
	public static double PARALLEL_Y = -5.5; // Y is the strafe direction

	public static double PERPENDICULAR_X = -3.5;
	public static double PERPENDICULAR_Y = -5.25; // old 6

	// (90*3)/(89.69190072548207 + 89.83577948877036 + 89.19368620066722)
	public static double X_MULTIPLIER = 0.994337245535;// 1.00475821; // Multiplier in the X direction
	// (90*3)/(89.36874508971657 + 89.43101734156207 + 89.70519900055706)
	public static double Y_MULTIPLIER = 1;//1.00556801; // Multiplier in the Y direction

	//    // Parallel/Perpendicular to the forward axis
	//    // Parallel wheel is parallel to the forward axis
	//    // Perpendicular is perpendicular to the forward axisF
	private final Encoder parallelEncoder;
	private final Encoder perpendicularEncoder;

	private final RRMecanumDriveTechnicolor drive;

	public RRTwoWheelTrackingLocalizer( HardwareMap hardwareMap, RRMecanumDriveTechnicolor drive ) {
		super( Arrays.asList(
				new Pose2d( PARALLEL_X, PARALLEL_Y, 0 ),
				new Pose2d( PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians( 90 ) )
		) );

		parallelEncoder = new Encoder( hardwareMap.get( DcMotorEx.class, "goalLift" ) );
		perpendicularEncoder = new Encoder( hardwareMap.get( DcMotorEx.class, "intake" ) );

		// TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
		parallelEncoder.setDirection( Encoder.Direction.REVERSE );
		perpendicularEncoder.setDirection( Encoder.Direction.REVERSE );

		this.drive = drive;
	}

	public static double encoderTicksToInches( double ticks ) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@NonNull
	@Override
	public double getHeading( ) {
		return drive.getRawExternalHeading( );
	}

	@NonNull
	@Override
	public Double getHeadingVelocity( ) {
		return drive.getExternalHeadingVelocity( );
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions( ) {
		return Arrays.asList(
				encoderTicksToInches( parallelEncoder.getCurrentPosition( ) * X_MULTIPLIER ),
				encoderTicksToInches( perpendicularEncoder.getCurrentPosition( ) * Y_MULTIPLIER )
		);
	}

	@NonNull
	@Override
	public List<Double> getWheelVelocities( ) {
		// TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
		//  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
		//  compensation method

		return Arrays.asList(
				encoderTicksToInches( parallelEncoder.getRawVelocity( ) * X_MULTIPLIER ),
				encoderTicksToInches( perpendicularEncoder.getRawVelocity( ) * Y_MULTIPLIER )
		);
	}
}