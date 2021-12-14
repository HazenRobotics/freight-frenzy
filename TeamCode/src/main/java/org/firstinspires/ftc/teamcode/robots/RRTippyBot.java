package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.localization.Field;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Capper;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinnerMotor;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.NoodleIntake;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLift;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

public class RRTippyBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public BarcodeUtil barcodeUtil;

	public RRMecanumDriveTippy42 drive;
	public MecanumDrive mecanumDrive;
	public EncoderTracker encoderTracker;

	public CarouselSpinnerMotor spinner;

	public Lift lift;
	public Bucket bucket;
	public Capper capper;
	public Grabber grabber;

	public OdometryLift odometryLift;

	public NoodleIntake intake;

	public static final double LIFT_ANGLE = 50;

	public static final double BUCKET_ANGLE_RANGE = 190;
	// lift default is 35, max of 165, range = 200

	public static final double BUCKET_ANGLE_INTAKE = 90; // theoretically should be exactly 90 but 0.0 - 0.4 doesn't set position correctly on the servo
	public static final double BUCKET_ANGLE_MOVING = LIFT_ANGLE;
	public static final double BUCKET_ANGLE_DUMP = -25; // should be around -45 but the servo is weird

	public static final double CAPPER_PICKUP = 1.0;
	public static final double CAPPER_HOLD = 0.8;


	public static double ROBOT_LENGTH = 13.5;
	public static double ROBOT_WIDTH = 12.5;

	public RRTippyBot( OpMode op ) {

		super( op );

		Robot.writeToDefaultFile( "Creating " + getClass( ).getSimpleName( ), false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes

//		new SoundLibrary( hardwareMap );

		drive = new RRMecanumDriveTippy42( hardwareMap );

		spinner = new CarouselSpinnerMotor( hardwareMap );

		lift = new Lift( hardwareMap, "lift", false, 2.375, (38.2 / 25.4) / 2, LIFT_ANGLE, AngleUnit.DEGREES );
		// LIFT_ANGLE - 90 :: because the servo's one position is below and perpendicular to the lift
		bucket = new Bucket( hardwareMap, "bucket", BUCKET_ANGLE_INTAKE - BUCKET_ANGLE_RANGE, BUCKET_ANGLE_RANGE );
		capper = new Capper( hardwareMap, "capper" );
//		grabber = new Grabber( hardwareMap, "grabber" );

		mecanumDrive = new MecanumDrive( hardwareMap );
		// bevel gear madness
		mecanumDrive.setMotorDirections( Direction.REVERSE, Direction.FORWARD /**/, Direction.FORWARD, Direction.FORWARD );
		super.driveTrain = mecanumDrive;
		encoderTracker = new EncoderTracker( hardwareMap, "frontLeft", "frontRight", 38 / 25.4, 537.7, 1 );

		odometryLift = new OdometryLift( hardwareMap );

		intake = new NoodleIntake( hardwareMap );

		barcodeUtil = new BarcodeUtil( hardwareMap, "webcam", telemetry );

		capper.setPosition( 0 );
	}

	/**
	 * @param time wait time in seconds
	 */
	public void sleepRobot( double time ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + time > opMode.getRuntime( ) ) {
			try {
				Thread.sleep( 50 );
			} catch( InterruptedException ignored ) {
			}
		}
	}

	public double getLongitudinalPosition( ) {
		return encoderTracker.convertTicksDist( encoderTracker.getLongitudinalPosition( ) );
	}

	public double getLateralPosition( ) {
		return encoderTracker.convertTicksDist( encoderTracker.getLateralPosition( ) );
	}

	public double shippingHubHeightToInches( RRHexBot.ShippingHubHeight height ) {

		telemetry.addLine( "shippingHubHeightToInches: " + height );
		telemetry.update( );
		switch( height ) {
			default: // (LOW)
				return 8.5;
			case MIDDLE:
				return 13;
			case HIGH:
				return 20;
		}
	}

	public void liftToShippingHubHeight( RRHexBot.ShippingHubHeight height ) {
		lift.setHeightVelocity( 1200, shippingHubHeightToInches( height ) );
	}

	public RRHexBot.ShippingHubHeight barcodePosToShippingHubHeight( BarcodePositionDetector.BarcodePosition position ) {
		RRHexBot.ShippingHubHeight height;
		switch( position ) {
			default: // (LEFT)
				height = RRHexBot.ShippingHubHeight.LOW;
				break;
			case MIDDLE:
				height = RRHexBot.ShippingHubHeight.MIDDLE;
				break;
			case RIGHT:
				height = RRHexBot.ShippingHubHeight.HIGH;
				break;
		}
		return height;
	}

	public double shippingHubDistance( RRHexBot.ShippingHubHeight height ) {
		switch( height ) {
			default: // (LOW)
				return 1.5;
			case MIDDLE:
				return 5;
			case HIGH:
				return 7;
		}
	}

	public void dumpBucket( ) {
		bucket.setAngle( BUCKET_ANGLE_DUMP );
		sleepRobot( 0.65 );
		bucket.setAngle( BUCKET_ANGLE_INTAKE );
	}

	/*public double distanceFromShippingHub( RRHexBot.ShippingHubHeight height) {
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
	}*/

	/**
	 * @param angle       the number of degrees to turn to reach the side of the shipping hub
	 * @param angleOffset the starting angle of the robot
	 * @param indent      the distance away from the shipping hub base to be
	 * @param blueSide    whether or not the robot is on the blue side
	 * @return the position (Pose2D) of where the robot should move to fit the provided parameters
	 */
	public static Pose2d getHubPosition( double angle, double angleOffset, double indent, boolean blueSide   ) {
		return Robot.getHubPosition( ROBOT_LENGTH, angle, angleOffset, indent, blueSide );
	}

}
