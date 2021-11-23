package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// 13" robot with laptop on it
@TeleOp(name = "MiniBot", group = "TeleOp")
public  class MiniBot extends OpMode {

	public static String CLASS_NAME = "AutoDemonstration";

	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor backLeft;
	DcMotor backRight;

	@Override
	public void init() {

		frontLeft = hardwareMap.dcMotor.get("frontLeft");
		frontRight = hardwareMap.dcMotor.get("frontRight");
		backLeft = hardwareMap.dcMotor.get("backLeft");
		backRight = hardwareMap.dcMotor.get("backRight");

		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


		addAndUpdate(CLASS_NAME + " : finished init");


	}
	


	@Override
	public void loop() {

		double drive =  -gamepad1.left_stick_y;
		double strafe = gamepad1.left_stick_x;
		double rotate = gamepad1.right_stick_x;

		double frontLeftPower = drive + strafe + rotate;
		double frontRightPower = drive - strafe - rotate;
		double backLeftPower = drive - strafe + rotate;
		double backRightPower = drive + strafe - rotate;

		setPowers( frontLeftPower, frontRightPower, backLeftPower, backRightPower );


		telemetry.update();
	}

	public void addAndUpdate( String text ) {
		telemetry.addLine( text );
		telemetry.update();

	}

	public void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {

		frontLeft.setPower(frontLeftPower);
		frontRight.setPower(frontRightPower);
		backLeft.setPower(backLeftPower);
		backRight.setPower(backRightPower);

		telemetry.addLine("FL: " + frontLeftPower + ", FR: " + frontRightPower);
		telemetry.addLine("BL: " + backLeftPower + ", BR: " + backRightPower);

	}
}
