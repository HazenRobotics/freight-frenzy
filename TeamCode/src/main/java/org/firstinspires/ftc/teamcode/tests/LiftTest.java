package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftTest extends LinearOpMode {

	Lift lift;

	@Override
	public void runOpMode( ) {

		lift = new Lift( hardwareMap );

		telemetry.addLine( "init finished" );
		telemetry.update();

		waitForStart();

		lift.setPosition( 400, 10 );

	}
}
