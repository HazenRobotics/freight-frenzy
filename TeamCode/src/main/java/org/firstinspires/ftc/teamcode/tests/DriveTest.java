package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.RRWoodBot;

//@Disabled
@Autonomous(name = "DriveTest", group = "Test")
public class DriveTest extends LinearOpMode {

	RRWoodBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new RRWoodBot( this );

		telemetry.addLine( "Finished init" );
		telemetry.update( );

		waitForStart( );

		for( double i = 0; i <= 1; i += 0.1 ) {
			robot.mecanumDrive.drive( i, 0, 0 );
			telemetry.addLine( "" + i );
			telemetry.update( );
			robot.sleepRobot( 0.5 );
		}

	}
}
