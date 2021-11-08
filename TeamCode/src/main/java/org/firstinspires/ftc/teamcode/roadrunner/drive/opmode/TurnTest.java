package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class TurnTest extends LinearOpMode {

	public static double ANGLE = 90; // deg

	@Override
	public void runOpMode( ) throws InterruptedException {
		RRMecanumDriveHex42 drive = new RRMecanumDriveHex42( hardwareMap );

		waitForStart( );

		if( isStopRequested( ) ) return;

		drive.turn( Math.toRadians( ANGLE ) );
	}
}
