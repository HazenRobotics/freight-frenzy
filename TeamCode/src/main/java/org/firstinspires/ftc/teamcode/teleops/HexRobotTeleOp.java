
package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.HexBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
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
public class HexRobotTeleOp extends OpMode {

	GamepadEvents player1;
	GamepadEvents player2;

	HexBot robot;

	double minDrive = 0.5, maxDrive = 0.8;
	double minStrafe = 0.7, maxStrafe = 1.0;
	double minRotate = 0.7, maxRotate = 1.0;

	double intakePower = 0.9;

	@Override
	public void init( ) {

		Robot.createMatchLogFile( "HexRobotTeleOp" );

		robot = new HexBot( this );
		player1 = new GamepadEvents( gamepad1 );
		player2 = new GamepadEvents( gamepad2 );

		SoundLibrary.playRandomStartup( );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		addControlTelemetry( );

		//gamepad inputs
		robot.mecanumDrive.drive( -gamepad1.left_stick_y * (gamepad1.left_stick_button ? maxDrive : minDrive),
				gamepad1.left_stick_x * (gamepad1.left_stick_button ? maxStrafe : minStrafe),
				gamepad1.right_stick_x * (gamepad1.right_stick_button ? maxRotate : minRotate) );

		telemetry.addLine( "front left: " + robot.mecanumDrive.getFrontLeftPosition( ) );
		telemetry.addLine( "back left: " + robot.mecanumDrive.getBackLeftPosition( ) );
		telemetry.addLine( "front right: " + robot.mecanumDrive.getFrontRightPosition( ) );
		telemetry.addLine( "back right: " + robot.mecanumDrive.getBackRightPosition( ) );

		// intake
		if( player1.left_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) < intakePower - 0.1 ? intakePower : 0 );
		else if( player1.right_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) > -intakePower + 0.1 ? -intakePower : 0 );

		// lift velocity control
		if( gamepad1.right_trigger > 0 )
			robot.lift.setPower( gamepad1.right_trigger );
		else if( gamepad1.left_trigger > 0 )
			robot.lift.setPower( -gamepad1.left_trigger );
		else
			robot.lift.setPower( 0 );

		// bucket control
		// dpad right - intake
		// dpad up - top layer
		// dpad left - middle layer
		// dpad down - bottom layer
		if( gamepad1.dpad_right || gamepad2.dpad_right )
			robot.bucket.setAngle( HexBot.BUCKET_ANGLE_INTAKE );
		else if( gamepad1.dpad_up || gamepad2.dpad_up )
			robot.bucket.setAngle( HexBot.BUCKET_ANGLE_TOP );
		else if( gamepad1.dpad_left || gamepad2.dpad_left )
			robot.bucket.setAngle( HexBot.BUCKET_ANGLE_MIDDLE );
		else if( gamepad1.dpad_down || gamepad2.dpad_down )
			robot.bucket.setAngle( HexBot.BUCKET_ANGLE_BOTTOM );

		if( gamepad1.a || gamepad2.a )
			robot.bucket.setPosition( 0 );
		else if( gamepad1.b || gamepad2.b )
			robot.bucket.setPosition( 1 );

		telemetry.addLine( "bucket: " + robot.bucket.getPosition( ) );

		// carousel spinner control
		if( player1.x.onPress( ) ) // toggles carousel spinner
			robot.spinnerLeft.setPower( robot.spinnerLeft.getPower( ) > 0 ? 0 : 1 );
		else if( player1.y.onPress( ) )
			robot.spinnerRight.setPower( robot.spinnerRight.getPower( ) < 0 ? 0 : -1 );

		//updates
		telemetry.update( );
		player1.update( );
		player2.update( );
	}

	/**
	 * <p>	&emsp;&emsp;&nbsp;&nbsp;  ↑  &emsp;&emsp;	&emsp;&emsp;  &emsp;&emsp;&nbsp;&nbsp;&nbsp;&nbsp;  y
	 * <p>	←  &emsp;&emsp;&emsp;&emsp;  →	&emsp;&emsp; x  &emsp;&emsp;&emsp;&emsp;  b
	 * <p>	&emsp;&emsp;&nbsp;&nbsp;  ↓  &emsp;&emsp;	&emsp;&emsp;  &emsp;&emsp;&nbsp;&nbsp;&nbsp;&nbsp;  a
	 */
	public void addControlTelemetry( ) {

		//			↑					y
		//		←		→			x		b
		//			↓					a

		telemetry.addLine( "          Controls:" );
		telemetry.addLine( "Drive: Gp1: left stick y (axis)" );
		telemetry.addLine( "Strafe: Gp1: left stick x (axis)" );
		telemetry.addLine( "Rotate: Gp1: right stick x (axis)" );
		telemetry.addLine( "Lift Up: Gp1: right trigger" );
		telemetry.addLine( "Lift Down: Gp1: left trigger" );
		telemetry.addLine( "Intake In: Gp1: right bumper" );
		telemetry.addLine( "Intake Out: Gp1: left bumper" );
		telemetry.addLine( "Left Spinner: Gp1/2: x" );
		telemetry.addLine( "Right Spinner: Gp1/2: y" );
		telemetry.addLine( "Bucket: Gp1/2: dpad right - intake, up - top, left - middle, down - bottom" );
		telemetry.addLine( "Capper: Gp1/2: a & b" );
		telemetry.addLine( );
	}

}