package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@Disabled
@TeleOp(name = "WoodRobotBasicTeleOp", group = "TeleOp")
public class WoodRobotBasicTeleOp extends OpMode {
	DcMotorEx frontLeftMotor;
	DcMotorEx frontRightMotor;
	DcMotorEx backLeftMotor;
	DcMotorEx backRightMotor;
	double frontLeftPower;
	double frontRightPower;
	double backLeftPower;
	double backRightPower;

	@Override
	public void init( ) {


		frontLeftMotor = hardwareMap.get( DcMotorEx.class, "frontLeft");
		backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeft");
		frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRight");
		backRightMotor = hardwareMap.get(DcMotorEx.class,"backRight");

		backLeftMotor.setDirection( REVERSE );
		frontLeftMotor.setDirection( REVERSE );

		telemetry.addLine("Init Finished");
		telemetry.update();
	}

	@Override
	public void loop( ) {
		double drive = -gamepad1.left_stick_y;
		double strafe = gamepad1.left_stick_x;
		double rotate = gamepad1.right_stick_x;

		frontLeftPower = drive + strafe + rotate;
		backLeftPower = drive - strafe + rotate;
		frontRightPower = drive - strafe - rotate;
		backRightPower = drive + strafe - rotate;

		frontLeftMotor.setPower(frontLeftPower);
		backLeftMotor.setPower(backLeftPower);
		frontRightMotor.setPower(frontRightPower);
		backRightMotor.setPower(backRightPower);


		telemetry.addData("Front Left Power", frontLeftMotor.getPower() );
		telemetry.addData("Back Left Power", backLeftMotor.getPower() );
		telemetry.addData("Front Right Power", frontRightMotor.getPower() );
		telemetry.addData("Back Right Power", backRightMotor.getPower() );
		telemetry.addData("Front Left Position", frontLeftMotor.getCurrentPosition() );
		telemetry.addData("Back Left Position", backLeftMotor.getCurrentPosition() );
		telemetry.addData("Front Right Position", frontRightMotor.getCurrentPosition() );
		telemetry.addData("Back Right Position", backRightMotor.getCurrentPosition() );
		telemetry.update();
	}
}
