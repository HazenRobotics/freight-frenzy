package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinnerServo;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GyroTracker;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

public class HexBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public BarcodeUtil barcodeUtil;

	public MecanumDrive mecanumDrive;
	public CarouselSpinnerServo spinnerLeft;
	public CarouselSpinnerServo spinnerRight;
	public Lift lift;
	public Bucket bucket;
	public Intake intake;
	public GyroTracker gyroTracker;
	public EncoderTracker encoderTracker;

	public static double LIFT_ANGLE = 55;

	public static double BUCKET_ANGLE_INTAKE = 55; // theoretically should be exactly 90 but 0.0 - 0.4 doesn't set position correctly
	public static double BUCKET_ANGLE_MOVING = LIFT_ANGLE;
	public static double BUCKET_ANGLE_TOP = -20;
	public static double BUCKET_ANGLE_MIDDLE = -27.5;
	public static double BUCKET_ANGLE_BOTTOM = -35;

	public HexBot( OpMode op ) {

		super( op );

		Robot.writeToDefaultFile( "", false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes
		barcodeUtil = new BarcodeUtil( hardwareMap, "webcam1", opMode.telemetry );

		new SoundLibrary( hardwareMap );

		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;
		spinnerLeft = new CarouselSpinnerServo( hardwareMap, "spinnerLeft" );
		spinnerRight = new CarouselSpinnerServo( hardwareMap, "spinnerRight" );
		lift = new Lift( hardwareMap, "lift", true, 2.5, (38.2 / 25.4) / 2, LIFT_ANGLE, AngleUnit.DEGREES );
		// LIFT_ANGLE - 90 :: because the servo's one position is below and perpendicular to the lift
		bucket = new Bucket( hardwareMap, "bucket", LIFT_ANGLE - 90, 180 );
		intake = new Intake( hardwareMap );
		gyroTracker = new GyroTracker( hardwareMap, false );
		encoderTracker = new EncoderTracker( hardwareMap, "intake", "perpendicular", 1, 1, 1 );

	}

	/**
	 * @param time wait time in seconds
	 */
	public void sleepRobot( double time ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + time > opMode.getRuntime( ) );
	}

}
