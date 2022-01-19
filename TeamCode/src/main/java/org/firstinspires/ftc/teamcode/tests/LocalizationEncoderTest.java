package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp(name = "LocalizationEncoderTest", group = "Test")
public class LocalizationEncoderTest extends OpMode {

	MecanumDrive drive;
	Encoder perpendicular;
	Encoder parallel;


	@Override
	public void init( ) {
		drive = new MecanumDrive( hardwareMap );
		perpendicular = new Encoder( drive.frontRight );
		parallel = new Encoder( drive.backRight );
	}

	@Override
	public void loop( ) {
		drive.drive( gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
		telemetry.addData( "perpendicular position", perpendicular.getCurrentPosition() );
		telemetry.addData( "parallel position", parallel.getCurrentPosition() );
	}
}
