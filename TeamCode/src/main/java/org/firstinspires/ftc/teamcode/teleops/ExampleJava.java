package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@TeleOp( name = "Example Java Learning Class", group = "Test" )
public class ExampleJava extends OpMode {

	MecanumDrive driver;

	@Override
	public void init() {
		driver = new MecanumDrive( hardwareMap );
	}

	@Override
	public void loop( ) {

		driver.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

	}

}
