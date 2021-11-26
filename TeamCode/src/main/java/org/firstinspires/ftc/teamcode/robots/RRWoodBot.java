package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveWood42;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Capper;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinnerMotor;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.NoodleIntake;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

public class RRWoodBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public BarcodeUtil barcodeUtil;

	public RRMecanumDriveWood42 drive;
	public MecanumDrive mecanumDrive;
	public EncoderTracker encoderTracker;

	public CarouselSpinnerMotor spinner;

	public Lift lift;
	public Bucket bucket;
	public Capper capper;

	public NoodleIntake intake;

	public static final double LIFT_ANGLE = 55;

	public static final double BUCKET_ANGLE_INTAKE = 85; // theoretically should be exactly 90 but 0.0 - 0.4 doesn't set position correctly
	public static final double BUCKET_ANGLE_MOVING = LIFT_ANGLE;
	public static final double BUCKET_ANGLE_DUMP = -35;

	public static final double CAPPER_PICKUP = 1.0;
	public static final double CAPPER_HOLD = 0.8;

	public RRWoodBot( OpMode op ) {

		super( op );

		Robot.writeToDefaultFile( "Creating " + getClass( ).getSimpleName( ), false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes

		// initialize util objects/classes
//		barcodeUtil = new BarcodeUtil( hardwareMap, "webcam", opMode.telemetry );

//		new SoundLibrary( hardwareMap );

		drive = new RRMecanumDriveWood42( hardwareMap );

		lift = new Lift( hardwareMap, "lift", false, 2.375, (38.2 / 25.4) / 2, LIFT_ANGLE, AngleUnit.DEGREES );
		// LIFT_ANGLE - 90 :: because the servo's one position is below and perpendicular to the lift
		bucket = new Bucket( hardwareMap, "bucket", LIFT_ANGLE - 80, 170 ); // was , LIFT_ANGLE - 90, 180
		capper = new Capper( hardwareMap, "capper" );

		mecanumDrive = new MecanumDrive( hardwareMap );
		mecanumDrive.setMotorDirections( Direction.REVERSE, Direction.REVERSE, Direction.FORWARD, Direction.FORWARD );
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
