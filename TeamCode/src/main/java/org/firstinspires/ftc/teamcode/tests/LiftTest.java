package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous(name = "LiftTest", group = "Tests")
//@Disabled
public class LiftTest extends LinearOpMode {

	Lift lift;

	@Override
	public void runOpMode( ) {

		lift = new Lift( hardwareMap );

		telemetry.addLine( "init finished" );
		telemetry.update( );

		waitForStart( );

		lift.moveLift( 0.5, 13 );

		telemetry.addLine( "set position" );
		telemetry.update( );

		while( !isStopRequested( ) ) ;

	}
}
