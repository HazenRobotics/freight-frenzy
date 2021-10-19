package org.firstinspires.ftc.teamcode.teleops;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.HexWoodBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "MotorDriveEncoderTest", group = "TeleOp")
//@Disabled
public class MotorDriveEncoderTest extends OpMode {

	// Declare the variables of motors and distance sens

	HexWoodBot robot;

	@Override
	public void init( ) {

		robot = new HexWoodBot( this );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		double frontLeftPower = gamepad1.y ? 1 : 0;
		double backLeftPower = gamepad1.x ? 1 : 0;
		double frontRightPower = gamepad1.b ? 1 : 0;
		double backRightPower = gamepad1.a ? 1 : 0;

		//gamepad inputs
		robot.mecanumDrive.setMotorPower( frontLeftPower, backLeftPower, frontRightPower, backRightPower );

		AngleUnit angleUnit = AngleUnit.DEGREES;

		telemetry.addLine( "front left vel: " + robot.mecanumDrive.getFrontLeftVelocity( angleUnit ) );
		telemetry.addLine( "back left vel: " + robot.mecanumDrive.getBackLeftVelocity( angleUnit ) );
		telemetry.addLine( "front right vel: " + robot.mecanumDrive.getFrontRightVelocity( angleUnit ) );
		telemetry.addLine( "back right vel: " + robot.mecanumDrive.getBackRightVelocity( angleUnit ) );

		telemetry.addLine( "front left pos: " + robot.mecanumDrive.getFrontLeftPosition( ) );
		telemetry.addLine( "back left pos: " + robot.mecanumDrive.getBackLeftPosition( ) );
		telemetry.addLine( "front right pos: " + robot.mecanumDrive.getFrontRightPosition( ) );
		telemetry.addLine( "back right pos: " + robot.mecanumDrive.getBackRightPosition( ) );

		//updates
		telemetry.update( );
	}

}