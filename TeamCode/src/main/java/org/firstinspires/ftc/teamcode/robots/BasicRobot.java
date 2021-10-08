package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;

public class BasicRobot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;
	public CarouselSpinner spinner;
	public Lift lift;
	public Bucket bucket;
	public Intake intake;

	public BasicRobot( OpMode op ) {

		super( op );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes
//		tfod = new TensorFlowUtil( opMode );

		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;
		spinner = new CarouselSpinner( hardwareMap );
		lift = new Lift( hardwareMap );
		bucket = new Bucket( hardwareMap );
		intake = new Intake( hardwareMap );

	}

}