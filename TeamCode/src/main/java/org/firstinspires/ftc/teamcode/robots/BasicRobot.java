package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinnerServo;
import org.firstinspires.ftc.teamcode.subsystems.SquareIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class BasicRobot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;
	public CarouselSpinnerServo spinner;
	public Lift lift;
	public Bucket bucket;
	public SquareIntake intake;

	public BasicRobot( OpMode op ) {

		super( op );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes
//		tfod = new TensorFlowUtil( opMode );

		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;
		spinner = new CarouselSpinnerServo( hardwareMap );
		lift = new Lift( hardwareMap );
		bucket = new Bucket( hardwareMap );
		intake = new SquareIntake( hardwareMap );

	}

	public static void addMultipleLines( Telemetry telemetry, String input ) {
		for( String s : input.split( "\n" ) )
			telemetry.addLine( s );
	}

}
