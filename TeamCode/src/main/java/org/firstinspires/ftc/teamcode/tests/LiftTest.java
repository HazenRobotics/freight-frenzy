package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftTest extends LinearOpMode {

	Lift lift;

	@Override
	public void runOpMode( ) {

		lift = new Lift( hardwareMap );

		telemetry.addLine( "init finished" );
		telemetry.update();

		waitForStart();

		lift.setLiftPosition( 400, 10 );

	}
}
