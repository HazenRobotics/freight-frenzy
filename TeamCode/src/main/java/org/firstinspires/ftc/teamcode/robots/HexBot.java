package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.NoodleIntake;
import org.firstinspires.ftc.teamcode.subsystems.SquareIntake;
import org.firstinspires.ftc.teamcode.utils.GyroTracker;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

public class HexBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;

	BarcodeUtil barcodeUtil;

	public CarouselSpinner spinnerLeft;
	public CarouselSpinner spinnerRight;
	public Lift lift;
	public Bucket bucket;
	public NoodleIntake intake;
	public GyroTracker gyroTracker;
	public EncoderTracker encoderTracker;

	public HexBot( OpMode op ) {

		super( op );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes
		barcodeUtil = new BarcodeUtil( hardwareMap, "webcam", opMode.telemetry );

		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;
		spinnerLeft = new CarouselSpinner( hardwareMap );
		spinnerRight = new CarouselSpinner( hardwareMap );
		lift = new Lift( hardwareMap );
		bucket = new Bucket( hardwareMap );
		intake = new NoodleIntake( hardwareMap );
		gyroTracker = new GyroTracker( hardwareMap, false );
		encoderTracker = new EncoderTracker( hardwareMap, "intake", "perpendicular" );

	}

}
