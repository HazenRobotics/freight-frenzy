package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.SnowBot;

@TeleOp
public class SnowBotTeleop extends OpMode {
	SnowBot robot;

	@Override
	public void init( ) {
		robot = new SnowBot( hardwareMap );

	}

	@Override
	public void loop( ) {
		robot.drive.drive( gamepad1.left_stick_y, gamepad1.right_stick_x );

	}
}
