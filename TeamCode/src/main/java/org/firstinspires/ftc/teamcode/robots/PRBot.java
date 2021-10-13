package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.TwoWheelDrive;

// public relations robot (big blue wheels, big yellow Scarab logo)
public class PRBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public TwoWheelDrive drive;

	public PRBot( OpMode op ) {

		super( op );

		opMode = op;
		hardwareMap = opMode.hardwareMap;

		// initialize util objects/classes

		super.driveTrain = new TwoWheelDrive( hardwareMap );
		drive = (TwoWheelDrive) driveTrain;
	}

}
