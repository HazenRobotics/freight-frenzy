package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class BasicRobot extends Robot {

	public MecanumDrive mecanumDrive;
	public CarouselSpinner spinner;
	public Lift lift;
	public Bucket bucket;
	public Intake intake;

	public BasicRobot( OpMode op ) {

		super( op );

		// set opMode and hardwareMap from super (Robot)
		opMode = op;
		hardwareMap = opMode.hardwareMap;

		// initialize subsystems and utilities
		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;

		spinner = new CarouselSpinner( hardwareMap );
		lift = new Lift( hardwareMap );
		bucket = new Bucket( hardwareMap );
		intake = new Intake( hardwareMap );

//		tfod = new TensorFlowUtil( opMode );

	}

}
