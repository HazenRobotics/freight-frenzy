package org.firstinspires.ftc.teamcode.drives;

import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.kA;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.kStatic;
import static org.firstinspires.ftc.teamcode.drives.RRDriveConstantsHex42.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class RRMecanumDriveHex42 extends MecanumDrive {

	public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients( 6, 0, 0 );
	public static PIDCoefficients HEADING_PID = new PIDCoefficients( 4, 0, 0 );

	public static double LATERAL_MULTIPLIER = 1.148;

	public static double VX_WEIGHT = 1;
	public static double VY_WEIGHT = 1;
	public static double OMEGA_WEIGHT = 1;

	private final TrajectorySequenceRunner trajectorySequenceRunner;

	private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint( MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH );
	private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint( MAX_ACCEL );

	private final TrajectoryFollower follower;

	private final DcMotorEx leftFront;
	private final DcMotorEx leftRear;
	private final DcMotorEx rightRear;
	private final DcMotorEx rightFront;
	private final List<DcMotorEx> motors;

	private final BNO055IMU imu;
	private final VoltageSensor batteryVoltageSensor;

	public static final double CAMERA_X = 0;
	public static final double CAMERA_Y = 6.625;

	public RRMecanumDriveHex42( HardwareMap hardwareMap, boolean loadMap, String mapName ) {
		super( kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER );

		follower = new HolonomicPIDVAFollower( TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
				new Pose2d( 0.5, 0.5, Math.toRadians( 5.0 ) ), 0.5 );

		LynxModuleUtil.ensureMinimumFirmwareVersion( hardwareMap );

		batteryVoltageSensor = hardwareMap.voltageSensor.iterator( ).next( );

		for( LynxModule module : hardwareMap.getAll( LynxModule.class ) ) {
			module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );
		}

		// TODO: adjust the names of the following hardware devices to match your configuration
		imu = hardwareMap.get( BNO055IMU.class, "imu" );
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters( );
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize( parameters );

		// TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
		// upward (normal to the floor) using a command like the following:
		// BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

		leftFront = hardwareMap.get( DcMotorEx.class, "frontLeft" );
		leftRear = hardwareMap.get( DcMotorEx.class, "backLeft" );
		rightRear = hardwareMap.get( DcMotorEx.class, "backRight" );
		rightFront = hardwareMap.get( DcMotorEx.class, "frontRight" );

		motors = Arrays.asList( leftFront, leftRear, rightRear, rightFront );

		for( DcMotorEx motor : motors ) {
			MotorConfigurationType motorConfigurationType = motor.getMotorType( ).clone( );
			motorConfigurationType.setAchieveableMaxRPMFraction( 1.0 );
			motor.setMotorType( motorConfigurationType );
		}

		if( RUN_USING_ENCODER ) {
			setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		}

		setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

		if( RUN_USING_ENCODER && MOTOR_VELO_PID != null ) {
			setPIDFCoefficients( DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID );
		}

		// TODO: reverse any motors using DcMotor.setDirection()
		rightFront.setDirection( DcMotorSimple.Direction.REVERSE );
		rightRear.setDirection( DcMotorSimple.Direction.REVERSE );

		// TODO: if desired, use setLocalizer() to change the localization method
		// for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
		//setLocalizer( new TwoWheelTrackingLocalizer( hardwareMap, this ) );
		if(!(mapName == null)) {
			mapName = mapName + ".bin";
		}
		setLocalizer( new TrackingCameraLocalizer(hardwareMap, new Pose2d( CAMERA_X, CAMERA_Y, 0 ), loadMap, mapName) );

		trajectorySequenceRunner = new TrajectorySequenceRunner( follower, HEADING_PID );
	}

	public RRMecanumDriveHex42(HardwareMap hardwareMap) {
		this(hardwareMap, false, null);
	}

	public TrajectoryBuilder trajectoryBuilder( Pose2d startPose ) {
		return new TrajectoryBuilder( startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT );
	}

	public TrajectoryBuilder trajectoryBuilder( Pose2d startPose, boolean reversed ) {
		return new TrajectoryBuilder( startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT );
	}

	public TrajectoryBuilder trajectoryBuilder( Pose2d startPose, double startHeading ) {
		return new TrajectoryBuilder( startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT );
	}

	public TrajectorySequenceBuilder trajectorySequenceBuilder( Pose2d startPose ) {
		return new TrajectorySequenceBuilder(
				startPose,
				VEL_CONSTRAINT, ACCEL_CONSTRAINT,
				MAX_ANG_VEL, MAX_ANG_ACCEL
		);
	}

	public void turnAsync( double angle ) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(
				trajectorySequenceBuilder( getPoseEstimate( ) )
						.turn( angle )
						.build( )
		);
	}

	public void turn( double angle ) {
		turnAsync( angle );
		waitForIdle( );
	}

	public void followTrajectoryAsync( Trajectory trajectory ) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(
				trajectorySequenceBuilder( trajectory.start( ) )
						.addTrajectory( trajectory )
						.build( )
		);
	}

	public void followTrajectory( Trajectory trajectory ) {
		followTrajectoryAsync( trajectory );
		waitForIdle( );
	}

	public void followTrajectorySequenceAsync( TrajectorySequence trajectorySequence ) {
		trajectorySequenceRunner.followTrajectorySequenceAsync( trajectorySequence );
	}

	public void followTrajectorySequence( TrajectorySequence trajectorySequence ) {
		followTrajectorySequenceAsync( trajectorySequence );
		waitForIdle( );
	}

	public Pose2d getLastError( ) {
		return trajectorySequenceRunner.getLastPoseError( );
	}

	public void update( ) {
		updatePoseEstimate( );
		DriveSignal signal = trajectorySequenceRunner.update( getPoseEstimate( ), getPoseVelocity( ) );
		if( signal != null ) setDriveSignal( signal );
	}

	public void waitForIdle( ) {
		while( !Thread.currentThread( ).isInterrupted( ) && isBusy( ) )
			update( );
	}

	public boolean isBusy( ) {
		return trajectorySequenceRunner.isBusy( );
	}

	public void setMode( DcMotor.RunMode runMode ) {
		for( DcMotorEx motor : motors ) {
			motor.setMode( runMode );
		}
	}

	public void setZeroPowerBehavior( DcMotor.ZeroPowerBehavior zeroPowerBehavior ) {
		for( DcMotorEx motor : motors ) {
			motor.setZeroPowerBehavior( zeroPowerBehavior );
		}
	}

	public void setPIDFCoefficients( DcMotor.RunMode runMode, PIDFCoefficients coefficients ) {
		PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
				coefficients.p, coefficients.i, coefficients.d,
				coefficients.f * 12 / batteryVoltageSensor.getVoltage( )
		);

		for( DcMotorEx motor : motors ) {
			motor.setPIDFCoefficients( runMode, compensatedCoefficients );
		}
	}

	public void setWeightedDrivePower( Pose2d drivePower ) {
		Pose2d vel = drivePower;

		if( Math.abs( drivePower.getX( ) ) + Math.abs( drivePower.getY( ) )
				+ Math.abs( drivePower.getHeading( ) ) > 1 ) {
			// re-normalize the powers according to the weights
			double denom = VX_WEIGHT * Math.abs( drivePower.getX( ) )
					+ VY_WEIGHT * Math.abs( drivePower.getY( ) )
					+ OMEGA_WEIGHT * Math.abs( drivePower.getHeading( ) );

			vel = new Pose2d(
					VX_WEIGHT * drivePower.getX( ),
					VY_WEIGHT * drivePower.getY( ),
					OMEGA_WEIGHT * drivePower.getHeading( )
			).div( denom );
		}

		setDrivePower( vel );
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions( ) {
		List<Double> wheelPositions = new ArrayList<>( );
		for( DcMotorEx motor : motors ) {
			wheelPositions.add( encoderTicksToInches( motor.getCurrentPosition( ) ) );
		}
		return wheelPositions;
	}

	@Override
	public List<Double> getWheelVelocities( ) {
		List<Double> wheelVelocities = new ArrayList<>( );
		for( DcMotorEx motor : motors ) {
			wheelVelocities.add( encoderTicksToInches( motor.getVelocity( ) ) );
		}
		return wheelVelocities;
	}

	@Override
	public void setMotorPowers( double v, double v1, double v2, double v3 ) {
		leftFront.setPower( v );
		leftRear.setPower( v1 );
		rightRear.setPower( v2 );
		rightFront.setPower( v3 );
	}

	@Override
	public double getRawExternalHeading( ) {
		return imu.getAngularOrientation( ).firstAngle;
	}

	@Override
	public Double getExternalHeadingVelocity( ) {
		// TODO: This must be changed to match your configuration
		//                           | Z axis
		//                           |
		//     (Motor Port Side)     |   / X axis
		//                       ____|__/____
		//          Y axis     / *   | /    /|   (IO Side)
		//          _________ /______|/    //      I2C
		//                   /___________ //     Digital
		//                  |____________|/      Analog
		//
		//                 (Servo Port Side)
		//
		// The positive x axis points toward the USB port(s)
		//
		// Adjust the axis rotation rate as necessary
		// Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
		// flat on a surface

		return (double) imu.getAngularVelocity( ).zRotationRate;
	}

	public static TrajectoryVelocityConstraint getVelocityConstraint( double maxVel, double maxAngularVel, double trackWidth ) {
		return new MinVelocityConstraint( Arrays.asList(
				new AngularVelocityConstraint( maxAngularVel ),
				new MecanumVelocityConstraint( maxVel, trackWidth )
		) );
	}

	public static TrajectoryAccelerationConstraint getAccelerationConstraint( double maxAccel ) {
		return new ProfileAccelerationConstraint( maxAccel );
	}

	public void stopCamera() {
		((TrackingCameraLocalizer) getLocalizer()).stopCamera();
	}

	public void resetCamera() {
		((TrackingCameraLocalizer) getLocalizer()).resetCamera();
	}

	public T265Camera.PoseConfidence getPoseConfidence() {
		return ((TrackingCameraLocalizer) getLocalizer()).getPoseConfidence();
	}

	public void exportLocalizationMap(String mapName) {
		((TrackingCameraLocalizer) getLocalizer()).exportMap(mapName + ".bin");
	}
	public void exportLocalizationMap() {
		exportLocalizationMap( "map" );
	}
}
