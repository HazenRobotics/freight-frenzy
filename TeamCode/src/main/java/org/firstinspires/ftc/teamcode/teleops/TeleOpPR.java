package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.PRBot;

@TeleOp(name = "TeleOpPR", group = "TeleOp")
//@Disabled
public class TeleOpPR extends OpMode {

	PRBot robot;

	@Override
	public void init( ) {

		robot = new PRBot( this );

		telemetry.addLine( "Init finished." );
	}

	@Override
	public void loop( ) {

		// negated because the y axis is inverted on the controllers
		robot.drive.drive( -gamepad1.left_stick_y, gamepad1.right_stick_x );
	}
}
