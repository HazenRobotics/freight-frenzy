package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.SwerveKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import java.util.ArrayList;
import java.util.List;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public abstract class SwerveDriveJava extends Drive {

	@NotNull
	private Localizer localizer;
	private final double kV;
	private final double kA;
	private final double kStatic;
	private final double trackWidth;
	private final double wheelBase;

	public SwerveDriveJava( double kV, double kA, double kStatic, double trackWidth, double wheelBase ) {
		this.kV = kV;
		this.kA = kA;
		this.kStatic = kStatic;
		this.trackWidth = trackWidth;
		this.wheelBase = wheelBase;
		localizer = new SwerveDriveJava.SwerveLocalizer( this );
	}

	public SwerveDriveJava( double kV, double kA, double kStatic, double trackWidth ) {
		this( kV, kA, kStatic, trackWidth, trackWidth );
	}

	@NotNull
	public Localizer getLocalizer( ) {
		return localizer;
	}

	public void setLocalizer( @NotNull Localizer localizer ) {
		this.localizer = localizer;
	}

	public void setDriveSignal( @NotNull DriveSignal driveSignal ) {
		List<Double> velocities = SwerveKinematics.robotToWheelVelocities( driveSignal.getVel( ), trackWidth, wheelBase );
		List<Double> accelerations = SwerveKinematics.robotToWheelAccelerations( driveSignal.getVel( ), driveSignal.getAccel( ), trackWidth, wheelBase );
		List<Double> powers = Kinematics.calculateMotorFeedforward( velocities, accelerations, kV, kA, kStatic );
		List<Double> orientations = SwerveKinematics.robotToModuleOrientations( driveSignal.getVel( ), trackWidth, wheelBase );

		setMotorPowers( powers.get( 0 ), powers.get( 1 ), powers.get( 2 ), powers.get( 3 ) );
		setModuleOrientations( orientations.get( 0 ), orientations.get( 1 ), orientations.get( 2 ), orientations.get( 3 ) );
	}

	public void setDrivePower( @NotNull Pose2d drivePower ) {
		double avg = (trackWidth + wheelBase) / 2.0D;
		List<Double> powers = SwerveKinematics.robotToWheelVelocities( drivePower, trackWidth / avg, wheelBase / avg );
		List<Double> orientations = SwerveKinematics.robotToModuleOrientations( drivePower, trackWidth / avg, wheelBase / avg );
		setMotorPowers( powers.get( 0 ), powers.get( 1 ), powers.get( 2 ), powers.get( 3 ) );
		setModuleOrientations( orientations.get( 0 ), orientations.get( 1 ), orientations.get( 2 ), orientations.get( 3 ) );
	}

	public abstract void setMotorPowers( double frontLeft, double rearLeft, double rearRight, double frontRight );

	public abstract void setModuleOrientations( double frontLeft, double rearLeft, double rearRight, double frontRight );

	@NotNull
	public abstract List<Double> getWheelPositions( );

	@Nullable
	public List<Double> getWheelVelocities( ) {
		return null;
	}

	@NotNull
	public abstract List<Double> getModuleOrientations( );

	public static final class SwerveLocalizer implements Localizer {

		private Pose2d _poseEstimate;
		@Nullable
		private Pose2d poseVelocity;
		private List<Double> lastWheelPositions;
		private double lastExtHeading;
		private SwerveDriveJava drive;
		private boolean useExternalHeading;

		public SwerveLocalizer( @NotNull SwerveDriveJava drive, boolean useExternalHeading ) {

			this.drive = drive;
			this.useExternalHeading = useExternalHeading;
			_poseEstimate = new Pose2d( 0.0D, 0.0D, 0.0D/*, 7, (DefaultConstructorMarker) null*/ );
			lastWheelPositions = new ArrayList<>( 4 ); // CollectionsKt.emptyList( )
			lastExtHeading = Double.NaN;
		}

		public SwerveLocalizer( @NotNull SwerveDriveJava drive ) {
			this( drive, true );
		}

		@NotNull
		public Pose2d getPoseEstimate( ) {
			return _poseEstimate;
		}

		public void setPoseEstimate( @NotNull Pose2d value ) {
			lastWheelPositions = new ArrayList<>( 4 ); // CollectionsKt.emptyList( )
			lastExtHeading = Double.NaN;
			if( useExternalHeading )
				drive.setExternalHeading( value.getHeading( ) );
			_poseEstimate = value;
		}

		@Nullable
		public Pose2d getPoseVelocity( ) {
			return poseVelocity;
		}

		public void update( ) {

			List<Double> wheelPositions = drive.getWheelPositions( );
			List<Double> moduleOrientations = drive.getModuleOrientations( );
			double extHeading = useExternalHeading ? drive.getExternalHeading( ) : Double.NaN;

			if( !lastWheelPositions.isEmpty( ) ) {
				List<Double> wheelDeltas = new ArrayList<>( );
				for( int i = 0; i < wheelPositions.size( ); i++ )
					wheelDeltas.add( wheelPositions.get( i ) - lastWheelPositions.get( i ) );

				Pose2d robotPoseDelta = SwerveKinematics.wheelToRobotVelocities(
						wheelDeltas,
						moduleOrientations,
						drive.wheelBase,
						drive.trackWidth
				);

				double finalHeadingDelta = useExternalHeading ? Angle.normDelta( extHeading - lastExtHeading ) : robotPoseDelta.getHeading( );

				_poseEstimate = Kinematics.relativeOdometryUpdate(
						_poseEstimate,
						new Pose2d( robotPoseDelta.vec( ), finalHeadingDelta )
				);
			}

			List<Double> wheelVelocities = drive.getWheelVelocities( );
			Double extHeadingVel = this.drive.getExternalHeadingVelocity( );
			if( wheelVelocities != null ) {
				poseVelocity = SwerveKinematics.wheelToRobotVelocities(
						wheelVelocities,
						moduleOrientations,
						drive.wheelBase,
						drive.trackWidth
				);

				if( useExternalHeading && extHeadingVel != null )
					poseVelocity = new Pose2d( getPoseVelocity( ).vec( ), extHeadingVel );
			}

			lastWheelPositions = wheelPositions;
			lastExtHeading = extHeading;
		}
	}
}
