package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OdometryLift;

@TeleOp
public class MoveOdometryLift extends OpMode {
	OdometryLift odometryLift;

	@Override
	public void init( ) {
		odometryLift = new OdometryLift( hardwareMap );

	}

	@Override
	public void loop( ) {
		odometryLift.setPower( -gamepad1.left_stick_y );

	}
}
