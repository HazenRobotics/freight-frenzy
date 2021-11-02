package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.HexBot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;

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

	HexBot robot;

	double intakePower = 0.9;

	@Override
	public void init( ) {

		robot = new HexBot( this );
		gamepad = new GamepadEvents( gamepad1 );

		SoundLibrary.playRandomStartup( );

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
		if( gamepad.left_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) > -intakePower + 0.1 ? -intakePower : 0 );
		else if( gamepad.right_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) < intakePower - 0.1 ? intakePower : 0 );

		// lift velocity control
		if( gamepad1.right_trigger > 0 )
			robot.lift.setPower( gamepad1.right_trigger );
		else if( gamepad1.left_trigger > 0 )
			robot.lift.setPower( -gamepad1.left_trigger );
		else
			robot.lift.setPower( 0 );

		// bucket control
		// multiple positions

		// carousel spinner control
		if( gamepad.x.onPress( ) ) // toggles carousel spinner
			robot.spinnerLeft.setPower( robot.spinnerLeft.getPower( ) > 0 ? 0 : 1 );
		else if( gamepad.y.onPress( ) )
			robot.spinnerRight.setPower( robot.spinnerRight.getPower( ) < 0 ? 0 : -1 );

		//updates
		telemetry.update( );
		gamepad.update( );
	}

	public void addControlTelemetry( ) {

		// 		y
		//   x     b
		//		a

		telemetry.addLine( "            Controls:" );
		telemetry.addLine( "Drive: Gp1: left stick y (axis)" );
		telemetry.addLine( "Strafe: Gp1: left stick x (axis)" );
		telemetry.addLine( "Rotate: Gp1: right stick x (axis)" );
		telemetry.addLine( "Lift Up: Gp1: right trigger" );
		telemetry.addLine( "Lift Down: Gp1: left trigger" );
		telemetry.addLine( "Intake In: Gp1: left bumper" );
		telemetry.addLine( "Intake Out: Gp1: right bumper" );
		telemetry.addLine( "Left Spinner: Gp1: x" );
		telemetry.addLine( "Right Spinner: Gp1: y" );
		telemetry.addLine( );
	}

}