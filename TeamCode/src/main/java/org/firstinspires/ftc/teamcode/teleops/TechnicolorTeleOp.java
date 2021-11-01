package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@TeleOp(name = "TechnicolorTeleOp", group = "TeleOp")
public class TechnicolorTeleOp extends OpMode {

	public static String CLASS_NAME = "AutoDemonstration";

	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor backLeft;
	DcMotor backRight;
	DcMotor spinRight;
	DcMotor spinLeft;
	Servo shooterServo;

	MecanumDrive drive;


	@Override
	public void init( ) {

		drive = new MecanumDrive( hardwareMap );

//		frontLeft = hardwareMap.dcMotor.get("frontLeft");
//		frontRight = hardwareMap.dcMotor.get("frontRight");
//		backLeft = hardwareMap.dcMotor.get("backLeft");
//		backRight = hardwareMap.dcMotor.get("backRight");

		spinRight = hardwareMap.dcMotor.get( "spinRight" );
		spinLeft = hardwareMap.dcMotor.get( "spinLeft" );
		shooterServo = hardwareMap.servo.get( "shooterServo" );

//		frontRight.setDirection( DcMotorSimple.Direction.REVERSE);
//		backRight.setDirection(DcMotorSimple.Direction.REVERSE);

		spinRight.setDirection( DcMotorSimple.Direction.REVERSE );

		addAndUpdate( CLASS_NAME + " : finished init" );

	}


	@Override
	public void loop( ) {

		drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

//		double drive = -gamepad1.left_stick_y;
//		double strafe = gamepad1.left_stick_x;
//		double rotate = gamepad1.right_stick_x;
//
//		double frontLeftPower = drive + strafe - rotate;
//		double frontRightPower = drive - strafe - rotate;
//		double backLeftPower = drive - strafe + rotate;
//		double backRightPower = drive + strafe + rotate;
//
//		setPowers( frontLeftPower, frontRightPower, backLeftPower, backRightPower );

		spinLeft.setPower( gamepad1.left_trigger );
		spinRight.setPower( gamepad1.left_trigger );

		if( gamepad1.a )
			shooterServo.setPosition( 0.73 );
		else
			shooterServo.setPosition( 0.4 );
	}

	public void addAndUpdate( String text ) {
		telemetry.addLine( text );
		telemetry.update( );

	}

	public void setPowers( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {

		frontLeft.setPower( frontLeftPower );
		frontRight.setPower( frontRightPower );
		backLeft.setPower( backLeftPower );
		backRight.setPower( backRightPower );

		telemetry.addLine( "FL: " + frontLeftPower + ", FR: " + frontRightPower );
		telemetry.addLine( "BL: " + backLeftPower + ", BR: " + backRightPower );

	}
}
