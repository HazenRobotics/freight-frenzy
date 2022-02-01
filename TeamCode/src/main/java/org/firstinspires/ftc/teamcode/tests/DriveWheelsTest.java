package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Autonomous(name = "DriveWheelsTest", group = "Test")
//@Disabled
public class DriveWheelsTest extends LinearOpMode {

	MecanumDrive mecanumDrive;

	@Override
	public void runOpMode( ) throws InterruptedException {
		mecanumDrive = new MecanumDrive( hardwareMap );
		// bevel gear madness
		mecanumDrive.setMotorDirections( DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD );

		telemetry.addLine( "Finished init" );
		telemetry.update( );

		waitForStart( );

		mecanumDrive.frontLeft.setPower( 1 );
		Robot.waitTime( 500 );
		mecanumDrive.drive( 0, 0, 0 );

		mecanumDrive.backLeft.setPower( 1 );
		Robot.waitTime( 500 );
		mecanumDrive.drive( 0, 0, 0 );

		mecanumDrive.frontRight.setPower( 1 );
		Robot.waitTime( 500 );
		mecanumDrive.drive( 0, 0, 0 );

		mecanumDrive.backRight.setPower( 1 );
		Robot.waitTime( 500 );
		mecanumDrive.drive( 0, 0, 0 );


	}
}
