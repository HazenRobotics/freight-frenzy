package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.NoodleIntake;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;
import org.firstinspires.ftc.teamcode.utils.GyroTracker;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

public class RRHexBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public BarcodeUtil barcodeUtil;

	public RRMecanumDriveHex42 drive;
	public MecanumDrive mecanumDrive;
	public CarouselSpinner spinnerLeft;
	public CarouselSpinner spinnerRight;
	public Lift lift;
	public Bucket bucket;
	public NoodleIntake intake;
	public GyroTracker gyroTracker;
	public EncoderTracker encoderTracker;

	public final double LIFT_ANGLE = 55;

	public final double BUCKET_ANGLE_INTAKE = 55; // theoretically should be exactly 90 but 0.0 - 0.4 doesn't set position correctly
	public final double BUCKET_ANGLE_MOVING = LIFT_ANGLE;
	public final double BUCKET_ANGLE_TOP = 45;
	public final double BUCKET_ANGLE_MIDDLE = 0.0;
	public final double BUCKET_ANGLE_BOTTOM = -45;
	public final double BUCKET_ANGLE_DUMP = -35;

	public enum ShippingHubHeight {
		LOW,
		MIDDLE,
		HIGH
	}

	public RRHexBot( OpMode op ) {

		super( op );

//		Robot.writeToDefaultFile( "", false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes
		barcodeUtil = new BarcodeUtil( hardwareMap, "webcam", opMode.telemetry );

		new SoundLibrary( hardwareMap );

		drive = new RRMecanumDriveHex42( hardwareMap );

		spinnerLeft = new CarouselSpinner( hardwareMap, "spinnerLeft" );
		spinnerRight = new CarouselSpinner( hardwareMap, "spinnerRight" );
		lift = new Lift( hardwareMap, "lift", 3, (38.2 / 25.4) / 2, LIFT_ANGLE, AngleUnit.DEGREES );
		// LIFT_ANGLE - 90 :: because the servo's one position is below and perpendicular to the lift
		bucket = new Bucket( hardwareMap, "bucket", LIFT_ANGLE - 90, 180 );
		intake = new NoodleIntake( hardwareMap );
		gyroTracker = new GyroTracker( hardwareMap, false );
		encoderTracker = new EncoderTracker( hardwareMap, "intake", "perpendicular" );

	}

	/**
	 * @param time wait time in seconds
	 */
	public void sleepRobot( double time ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + time > opMode.getRuntime( ) ) ;
	}

	public TrajectorySequenceBuilder getTrajectorySequenceBuilder( ) {
		return drive.trajectorySequenceBuilder( drive.getPoseEstimate( ) );
	}

	public double shippingHubHeightToInches( ShippingHubHeight height ) {
		switch( height ) {
			case LOW:
				return 6;
			case MIDDLE:
				return 11.5;
			case HIGH:
				return 17.75;
			default:
				return 5.9;
		}
	}

	public void liftToShippingHubHeight( ShippingHubHeight height ) {
		switch( height ) {
			case LOW:
				lift.setLiftHeightVel( 750, 6 );
//				lift.setLiftHeightPow( 0.8, 6 );
				opMode.telemetry.addLine( "LOW" );
				break;
			case MIDDLE:
				lift.setLiftHeightVel( 750, 11.5 );
//				lift.setLiftHeightPow( 0.8, 11.5 );
				opMode.telemetry.addLine( "MIDDLE" );
				break;
			case HIGH:
				lift.setLiftHeightVel( 750, 17.75 );
//				lift.setLiftHeightPow( 0.8, 17.75 );
				opMode.telemetry.addLine( "HIGH" );
				break;
		}
		opMode.telemetry.update( );
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
		sleepRobot( 1 );
		bucket.setAngle( BUCKET_ANGLE_INTAKE );
	}

}
