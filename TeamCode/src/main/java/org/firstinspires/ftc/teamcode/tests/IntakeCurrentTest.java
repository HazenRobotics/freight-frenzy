package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// 13" robot with laptop on it
@TeleOp(name = "IntakeCurrentTest", group = "TeleOp")
public  class IntakeCurrentTest extends OpMode {

	public static String CLASS_NAME = "AutoDemonstration";

	DcMotorEx intake;

	@Override
	public void init() {

		intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");


		addAndUpdate(CLASS_NAME + " : finished init");


	}




	@Override
	public void loop() {

		intake.setPower( gamepad1.left_trigger );

		addAndUpdate( "" + intake.getCurrent( CurrentUnit.MILLIAMPS ) );
	}

	public void addAndUpdate( String text ) {
		telemetry.addLine( text );
		telemetry.update();

	}


}
