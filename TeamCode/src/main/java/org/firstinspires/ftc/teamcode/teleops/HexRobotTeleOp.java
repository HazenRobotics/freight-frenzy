
package org.firstinspires.ftc.teamcode.teleops;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.text.DecimalFormat;

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

	RRHexBot robot;

	DecimalFormat df = new DecimalFormat( "0.###" );

	double minDrive = 0.5, maxDrive = 0.8;
	double minStrafe = 0.7, maxStrafe = 1.0;
	double minRotate = 0.5, maxRotate = 1.0;

	double intakePower = 0.7;

	double capperPosition = 0.7;

	boolean inDriverAssist = false;

	double prevLiftPos = 0;

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

//		gamepad1.rumble( 10000 );

		player1 = new GamepadEvents( gamepad1 );
		player2 = new GamepadEvents( gamepad2 );

		robot = new RRHexBot( this );

//		SoundLibrary.playRandomStartup( );

		Log.e( "Mode", "waiting for start" );
		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );

		Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder( )
				.addStep( 1, 1, 70 )
				.addStep( 0, 0, 70 )
				.addStep( 1, 1, 70 )
				.build( );
		gamepad1.runRumbleEffect( effect );
//		gamepad1.stopRumble( );
	}

	@Override
	public void loop( ) {

		addControlTelemetry( );

		//gamepad inputs
		robot.mecanumDrive.drive( -gamepad1.left_stick_y * (gamepad1.left_stick_button ? maxDrive : minDrive),
				gamepad1.left_stick_x * (gamepad1.left_stick_button ? maxStrafe : minStrafe),
				gamepad1.right_stick_x * (gamepad1.right_stick_button ? maxRotate : minRotate) );

		// intake
		if( player1.left_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) > 0 ? 0 : intakePower );
		else if( player1.right_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) < 0 ? 0 : -intakePower );

		// bucket control
		// dpad right - intake
		// dpad down - dump
		if( gamepad1.dpad_up ) {
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_INTAKE );
		} else if( gamepad1.dpad_down ) {
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_DUMP );
		} else if( gamepad1.dpad_right ) {
			robot.lift.exitLoops( 500 );
		}

		// if you touch either of the triggers, it will exit the driver assist method
		if( gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5 )
			inDriverAssist = false;

		// lift velocity control
		String liftPower = "false";
		if( !inDriverAssist ) {
			liftPower = "true";
			if( gamepad1.right_trigger + gamepad1.left_trigger > 0 ) {
				robot.lift.setTeleOPower( gamepad1.right_trigger - gamepad1.left_trigger );
			} else {
				robot.lift.setTeleOPower( 0 );
				robot.lift.disableMotorIfUnused( );
			}
		}

		// capper position
		if( gamepad1.y )
			capperPosition = 0;
		else if( gamepad1.a )
			capperPosition = 1;

		if( gamepad2.y )
			capperPosition -= 0.01;
		else if( gamepad2.a )
			capperPosition += 0.01;

		robot.capper.setPosition( capperPosition );

		// bucket auto slant while moving up '(or below min height)
		if( robot.lift.getPositionInch( ) < Lift.LIFT_SWITCH_LIMIT )
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_INTAKE );
		else if( robot.lift.getPositionInch( ) >= Lift.LIFT_SWITCH_LIMIT && robot.lift.getPositionInch( ) > prevLiftPos )
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_MOVING );
		prevLiftPos = robot.lift.getPositionInch( );

		// driver assist methods
		if( gamepad2.dpad_up ) {
			inDriverAssist = true;
			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
		} else if( gamepad2.dpad_left ) {
			inDriverAssist = true;
			robot.lift.setHeightVelocity( 850, 0 );
//			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_INTAKE );
		} else if( gamepad2.dpad_down ) {
			inDriverAssist = true;
			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
		}

		// carousel spinner control
		if( player1.x.onPress( ) || player2.x.onPress( ) ) // toggles carousel spinner
			robot.spinnerLeft.setPower( robot.spinnerLeft.getPower( ) > 0 ? 0 : intakePower );
		else if( player1.b.onPress( ) || player2.b.onPress( ) )
			robot.spinnerRight.setPower( robot.spinnerRight.getPower( ) < 0 ? 0 : -intakePower );

		// reset the lift position to its current zero position
		if( player1.ps.onPress( ) )
			robot.lift.resetLift( );

//		telemetry.addLine( "FL: " + robot.mecanumDrive.getFrontLeftPosition( ) );
//		telemetry.addLine( "BL: " + robot.mecanumDrive.getBackLeftPosition( ) );
//		telemetry.addLine( "FR: " + robot.mecanumDrive.getFrontRightPosition( ) );
//		telemetry.addLine( "BR: " + robot.mecanumDrive.getBackRightPosition( ) );
		telemetry.addLine( "settingLiftPower: " + liftPower );
		telemetry.addLine( "inDriverAssist: " + inDriverAssist );
		telemetry.addLine( "capperPosition: " + capperPosition );
		telemetry.addLine( "intakePower: " + df.format( intakePower ) );
		telemetry.addLine( "bucket: " + robot.bucket.getPosition( ) );
		telemetry.addLine( "capper: " + robot.capper.getPosition( ) );
		telemetry.addLine( "lift ground & limit: " + robot.lift.getGroundBucketHeight( ) + ", " + Lift.LIFT_SWITCH_LIMIT );
		telemetry.addLine( "lift target: " + df.format( robot.lift.getTargetPositionInch( ) ) + " (" + robot.lift.getTargetPosition( ) + ")" );
		telemetry.addLine( "lift diff: " + df.format( robot.lift.getPositionInch( ) ) + " - " + df.format( prevLiftPos ) );


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
		telemetry.addLine( "Right Spinner: Gp1/2: b" );
		telemetry.addLine( "Bucket: Gp1/2: dpad: up - intake, down - dump" );
		telemetry.addLine( "Capper Toggle: Gp1: a - up, b - down" );
		telemetry.addLine( "Capper Increment: Gp2: a - up, b - down" );
		telemetry.addLine( "Driver Assist (Lift): Gp2: dpad: up - top layer, down - bottom layer" );
		telemetry.addLine( );
	}

}