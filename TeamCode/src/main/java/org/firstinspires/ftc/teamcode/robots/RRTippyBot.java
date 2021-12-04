package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Capper;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinnerMotor;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.NoodleIntake;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;
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

	public NoodleIntake intake;

	public static final double LIFT_ANGLE = 50;

	public static final double BUCKET_ANGLE_RANGE = 200;
	// lift default is 35, max of 165, range = 200

	public static final double BUCKET_ANGLE_INTAKE = 35; // theoretically should be exactly 90 but 0.0 - 0.4 doesn't set position correctly on the servo
	public static final double BUCKET_ANGLE_MOVING = LIFT_ANGLE;
	public static final double BUCKET_ANGLE_DUMP = -55;

	public static final double CAPPER_PICKUP = 1.0;
	public static final double CAPPER_HOLD = 0.8;

	public RRTippyBot( OpMode op ) {

		super( op );

		Robot.writeToDefaultFile( "Creating " + getClass( ).getSimpleName( ), false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes

		// initialize util objects/classes
//		barcodeUtil = new BarcodeUtil( hardwareMap, "webcam", opMode.telemetry );

//		new SoundLibrary( hardwareMap );

		drive = new RRMecanumDriveTippy42( hardwareMap );

		lift = new Lift( hardwareMap, "lift", false, 2.375, (38.2 / 25.4) / 2, LIFT_ANGLE, AngleUnit.DEGREES );
		// LIFT_ANGLE - 90 :: because the servo's one position is below and perpendicular to the lift
		bucket = new Bucket( hardwareMap, "bucket", BUCKET_ANGLE_INTAKE - BUCKET_ANGLE_RANGE, BUCKET_ANGLE_RANGE ); //
		capper = new Capper( hardwareMap, "capper" );

		mecanumDrive = new MecanumDrive( hardwareMap );
		// bevel gear madness
		mecanumDrive.setMotorDirections( Direction.REVERSE, Direction.FORWARD /**/, Direction.FORWARD, Direction.FORWARD );
		super.driveTrain = mecanumDrive;
		encoderTracker = new EncoderTracker( hardwareMap, "frontLeft", "frontRight", 38 / 25.4, 537.7, 1 );

		intake = new NoodleIntake( hardwareMap );

		capper.setPosition( 0 );
	}

	/**
	 * @param time wait time in seconds
	 */
	public void sleepRobot( double time ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + time > opMode.getRuntime( ) ) ;
	}

	public double getLongitudinalPosition( ) {
		return encoderTracker.convertTicksDist( encoderTracker.getLongitudinalPosition( ) );
	}

	public double getLateralPosition( ) {
		return encoderTracker.convertTicksDist( encoderTracker.getLateralPosition( ) );
	}

}
