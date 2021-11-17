package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous(name = "Lift Distance Test", group = "Test")
@Disabled
public class LiftDistanceTest extends LinearOpMode {

	Lift lift;

	@Override
	public void runOpMode( ) {

		lift = new Lift( hardwareMap );

		telemetry.addLine( "init finished" );
		telemetry.update( );

		waitForStart( );

		lift.setHeightPower( 0.75, 17 );

		telemetry.addLine( "set position to 17" );
		telemetry.update( );

		while( !isStopRequested( ) ) ;

	}
}
