package org.firstinspires.ftc.teamcode.teleops;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name = "HexRobotTeleOp", group = "TeleOp")
//@Disabled
public class HexRobotTeleOp extends OpMode {

	// Declare the variables of motors and distance sens

	GamepadEvents gamepad;

	HexWoodBot robot;

	double intakePower = 0.5;

	@Override
	public void init( ) {

		robot = new HexWoodBot( this );
		gamepad = new GamepadEvents( gamepad1 );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		addControlTelemetry( );

		//gamepad inputs
		robot.mecanumDrive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

		telemetry.addLine( "front left: " + robot.mecanumDrive.getFrontLeftPosition( ) );
		telemetry.addLine( "back left: " + robot.mecanumDrive.getBackLeftPosition( ) );
		telemetry.addLine( "front right: " + robot.mecanumDrive.getFrontRightPosition( ) );
		telemetry.addLine( "back right: " + robot.mecanumDrive.getBackRightPosition( ) );

		// intake
		if( gamepad1.left_bumper )
			robot.intake.setPower( robot.intake.getPower( ) < intakePower ? intakePower : 0 );
		else if( gamepad1.right_bumper )
			robot.intake.setPower(  robot.intake.getPower( ) > -intakePower ? -intakePower : 0 );

		// lift velocity control
		if( gamepad1.left_trigger > 0 )
			robot.lift.setVelocity( gamepad1.left_trigger * Lift.MAX_VELOCITY );
		else if( gamepad1.right_trigger >= 0 )
			robot.lift.setVelocity( -gamepad1.right_trigger * Lift.MAX_VELOCITY );

		// bucket control
		// multiple positions

		// carousel spinner control
		if( gamepad.x.onPress( ) ) // toggles carousel spinner
			robot.spinner.setPower( robot.spinner.getPower( ) > 0 ? 0 : 1 );
		else if( gamepad.y.onPress( ) )
			robot.spinner.setPower( robot.spinner.getPower( ) < 0 ? 0 : -1 );

		//updates
		telemetry.update( );
		gamepad.update( );
	}

	public void addControlTelemetry( ) {

		telemetry.addLine( "            Controls:" );
		telemetry.addData( "Drive ", "Gp1: left stick y (axis)" )
				.addData( "Strafe", "Gp1: left stick x (axis)" )
				.addData( "Rotate", "Gp1: right stick x (axis)" )
				.addData( "Lift Up", "Gp1: right trigger" )
				.addData( "Lift Down", "Gp1: left trigger" )
				.addData( "example", "example" )
				.addData( "Intake Toggle", "Gp1/Gp2: right bumper" )
				.addData( "Negate Intake", "Gp1/Gp2: left bumper" );
		telemetry.addLine( );
	}

}