package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Capper;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinnerMotor;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

public class RRHexBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public BarcodeUtil barcodeUtil;

	public RRMecanumDriveHex42 drive;
	public MecanumDrive mecanumDrive;

	public CarouselSpinnerMotor spinner;

	public Lift lift;
	public Bucket bucket;
	public Capper capper;

	public Intake intake;



	// 13" robot (l, w): 13.25, 13.5
	// 18" robot (l, w): 17.375, 17.5
	static double ROBOT_LENGTH = 17.375;
	static double ROBOT_WIDTH = 17.5; // with belts

	static final double TILE_SIZE = 23;
	static final double TILE_CONNECTOR = 0.75;

	static final double HUB_RADIUS = 9;

	public static final double LIFT_ANGLE = 55;

	public static final double BUCKET_ANGLE_INTAKE = 85; // theoretically should be exactly 90 but 0.0 - 0.4 doesn't set position correctly
	public static final double BUCKET_ANGLE_MOVING = LIFT_ANGLE;
	public static final double BUCKET_ANGLE_DUMP = -35;

	public static final double CAPPER_PICKUP = 1.0;
	public static final double CAPPER_HOLD = 0.8;

	public enum ShippingHubHeight {
		LOW,
		MIDDLE,
		HIGH
	}

	public RRHexBot( OpMode op ) {

		super( op );

		Robot.createDefaultMatchLogFile( );
		Robot.writeToDefaultFile( "Creating " + getClass( ).getSimpleName( ), false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes
		barcodeUtil = new BarcodeUtil( hardwareMap, "webcam1", opMode.telemetry );

//		new SoundLibrary( hardwareMap );

		drive = new RRMecanumDriveHex42( hardwareMap );
		mecanumDrive = new MecanumDrive( hardwareMap );

		spinner = new CarouselSpinnerMotor( hardwareMap, "spinner" );

		lift = new Lift( hardwareMap, "lift", false, 2.375, (38.2 / 25.4) / 2, LIFT_ANGLE, AngleUnit.DEGREES );
		// LIFT_ANGLE - 90 :: because the servo's one position is below and perpendicular to the lift
		bucket = new Bucket( hardwareMap, "bucket", LIFT_ANGLE - 80, 170 ); // was , LIFT_ANGLE - 90, 180
		capper = new Capper( hardwareMap, "capper" );

		intake = new Intake( hardwareMap );

		capper.setPosition( 0 );

	}

	/**
	 * @param time wait time in seconds
	 */
	public void sleepRobot( double time ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + time > opMode.getRuntime( ) ) ;
	}

	/**
	 * @param angle           the number of degrees to turn to reach the side of the shipping hub
	 * @param angleOffset     the starting angle of the robot
	 * @param distanceFromHub the distance away from the shipping hub base to be
	 * @param blueSide        whether or not the robot is on the blue side
	 * @return the position (Pose2D) of where to go
	 */
	public Pose2d getHubPosition( double angle, double angleOffset, double distanceFromHub, boolean blueSide ) {
		double x = TILE_CONNECTOR / 2 + TILE_SIZE / 2 + Math.sin( Math.toRadians( angle ) ) * (HUB_RADIUS + distanceFromHub + ROBOT_LENGTH / 2);
		double y = TILE_CONNECTOR + TILE_SIZE + Math.cos( Math.toRadians( angle ) ) * (HUB_RADIUS + distanceFromHub + ROBOT_LENGTH / 2);
		return new Pose2d( -x, y * (blueSide ? 1 : -1), Math.toRadians( angleOffset + angle ) );
	}

	public TrajectorySequenceBuilder getTrajectorySequenceBuilder( ) {
		return drive.trajectorySequenceBuilder( drive.getPoseEstimate( ) );
	}

	public TrajectoryBuilder getTrajectoryBuilder( ) {
		return drive.trajectoryBuilder( drive.getPoseEstimate( ) );
	}

	public double shippingHubHeightToInches( ShippingHubHeight height ) {

		telemetry.addLine( "shippingHubHeightToInches: " + height );
		telemetry.update( );
		switch( height ) {
			case LOW:
				return 9;
			case MIDDLE:
				return 16;
			case HIGH:
				return 18;
			default:
				return 8;
		}
	}

	public void liftToShippingHubHeight( ShippingHubHeight height ) {
		lift.setHeightVelocity( 800, shippingHubHeightToInches( height ) );
		bucket.setAngle( BUCKET_ANGLE_MOVING );
	}

	public ShippingHubHeight barcodePosToShippingHubHeight( BarcodePositionDetector.BarcodePosition position ) {
		ShippingHubHeight height;
		switch( position ) {
			case LEFT:
				height = ShippingHubHeight.LOW;
				break;
			case MIDDLE:
				height = ShippingHubHeight.MIDDLE;
				break;
			case RIGHT:
				height = ShippingHubHeight.HIGH;
				break;
			default:
				height = ShippingHubHeight.LOW;
				break;
		}
		return height;
	}

	public void dumpBucket( ) {
		bucket.setAngle( BUCKET_ANGLE_DUMP );
		sleepRobot( 0.7 );
		bucket.setAngle( BUCKET_ANGLE_INTAKE );
	}

	public double distanceFromShippingHub(ShippingHubHeight height) {
		switch( height ) {
			case LOW:
				return 10.5 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
			case MIDDLE:
				return 7.5 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
			case HIGH:
				return 6 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
			default:
				return 11 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
		}
	}

}
