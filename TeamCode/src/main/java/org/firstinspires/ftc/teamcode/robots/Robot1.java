package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.vision.TensorFlowUtil;

public class Robot1 {

	public OpMode opMode;

	public TensorFlowUtil tfod;

	public Robot1( OpMode op ) {
		opMode = op;

		// initialize util objects/classes
		tfod = new TensorFlowUtil( opMode );
	}


}
