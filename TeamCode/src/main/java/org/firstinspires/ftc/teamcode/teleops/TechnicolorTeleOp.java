package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@TeleOp(name = "TechnicolorTeleOp", group = "TeleOp")
public class TechnicolorTeleOp extends OpMode {

	public static String CLASS_NAME = "TechnicolorTeleOp";

	MecanumDrive drive;

	DcMotor spinRight;
	DcMotor spinLeft;

	Servo shooterServo;


	@Override
	public void init( ) {

		drive = new MecanumDrive( hardwareMap );

		spinRight = hardwareMap.dcMotor.get( "spinRight" );
		spinLeft = hardwareMap.dcMotor.get( "spinLeft" );
		shooterServo = hardwareMap.servo.get( "shooterServo" );

		spinRight.setDirection( DcMotorSimple.Direction.REVERSE );

		telemetry.addLine( CLASS_NAME + " : finished init" );
		telemetry.update( );

	}

	@Override
	public void loop( ) {

		drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

		spinLeft.setPower( gamepad1.left_trigger );
		spinRight.setPower( gamepad1.left_trigger );

		if( gamepad1.a )
			shooterServo.setPosition( 0.73 );
		else
			shooterServo.setPosition( 0.4 );
	}
}
