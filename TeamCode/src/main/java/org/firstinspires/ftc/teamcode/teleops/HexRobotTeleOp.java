
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
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

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

	double spinnerPower = 0.75;
	boolean inSpinnerThread = false;
	List<Thread> spinnerThread = new ArrayList<>( );

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

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
		setBucketPosition( );

		// if you touch either of the triggers enough, it will exit the driver assist method
		if( gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5 )
			inDriverAssist = false;

		// lift velocity control
		if( !inDriverAssist ) {
			if( gamepad1.right_trigger + gamepad1.left_trigger > 0 ) {
				robot.lift.setTeleOPower( gamepad1.right_trigger - gamepad1.left_trigger );
			} else {
				robot.lift.setTeleOPower( 0 );
				robot.lift.disableMotorIfUnused( );
			}
		}

		// capper position
		if( player1.y.onPress( ) )
			capperPosition = capperPosition > 0.5 ? 0.0 : 1.0;
		else if( gamepad1.a )
			capperPosition = capperPosition > 0.2 ? 0.1 : 0.3; // prep the capper for the shipping element

		if( gamepad2.y )
			capperPosition -= 0.01;
		else if( gamepad2.a )
			capperPosition += 0.01;

		robot.capper.setPosition( capperPosition );

		// bucket auto slant while moving up '(or below min height)
		autoSlantBucket( );

		// driver assist methods
		runLiftAssistMethods( );

		runSpinnerAssistMethods( );

		// toggle carousel spinner

		if( player1.b.onPress( ) || player2.b.onPress( ) )
			robot.spinner.setPower( Math.abs( robot.spinner.getPower( ) ) < 0.1 ? spinnerPower : 0 );
//		else if( player1.x.onPress( ) || player2.x.onPress( ) )
//			robot.spinner.setPower( robot.spinner.getPower( ) < 0.1 ? spinnerPower : 0 );


		// reset the lift position to its current zero position
		if( gamepad1.ps )
			robot.lift.resetLift( );

		addControlTelemetry( );
		addInfoTelemetry( );

		if( player1.dpad_right.onPress( ) || player2.dpad_right.onPress( ) )
			spinnerPower *= -1;

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

	public void addInfoTelemetry( ) {

//		telemetry.addLine( "FL: " + robot.mecanumDrive.getFrontLeftPosition( ) );
//		telemetry.addLine( "BL: " + robot.mecanumDrive.getBackLeftPosition( ) );
//		telemetry.addLine( "FR: " + robot.mecanumDrive.getFrontRightPosition( ) );
//		telemetry.addLine( "BR: " + robot.mecanumDrive.getBackRightPosition( ) );
		telemetry.addLine( "settingLiftPower: " + (!inDriverAssist ? "true" : "false") );
		telemetry.addLine( "inDriverAssist: " + inDriverAssist );
		telemetry.addLine( "capperPosition: " + capperPosition );
		telemetry.addLine( "intakePower: " + df.format( intakePower ) );
		telemetry.addLine( "bucket: " + robot.bucket.getPosition( ) );
		telemetry.addLine( "capper: " + robot.capper.getPosition( ) );
		telemetry.addLine( "lift ground & limit: " + robot.lift.getGroundBucketHeight( ) + ", " + Lift.LIFT_SWITCH_LIMIT );
		telemetry.addLine( "lift target: " + df.format( robot.lift.getTargetPositionInch( ) ) + " (" + robot.lift.getTargetPosition( ) + ")" );
		telemetry.addLine( "lift diff: " + df.format( robot.lift.getPositionInch( ) ) + " - " + df.format( prevLiftPos ) );
	}

	public void runLiftAssistMethods( ) {
		if( gamepad2.dpad_up ) {
			inDriverAssist = true;
			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
		} else if( gamepad2.dpad_left ) {
			inDriverAssist = true;
			robot.lift.setHeightVelocity( 850, 0 );
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_INTAKE );
		} else if( gamepad2.dpad_down ) {
			inDriverAssist = true;
			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
		}
	}

	/**
	 * toggle the duck spinner assist thread
	 */
	public void runSpinnerAssistMethods( ) {

		if( player1.x.onPress( ) ) {
			Log.e( "runSpinnerAssistMethods", "pressed" );
			if( inSpinnerThread ) {
				Log.e( "runSpinnerAssistMethods", "stop" );
				inSpinnerThread = false;
				spinnerThread.get( 0 ).interrupt();
				spinnerThread.removeAll( spinnerThread );
				robot.spinner.setPower( 0 );
			} else {
				Log.e( "runSpinnerAssistMethods", "start" );
				runSpinnerThread( );
			}
		}
	}

	public void runSpinnerThread( ) {

		// 9 ducks in 25 seconds (10 if capstone)
		spinnerThread.add( new Thread( ( ) -> {
			int i = 0;
			inSpinnerThread = true;
			while( inSpinnerThread ) {
				robot.spinner.setPower( spinnerPower );
				robot.sleepRobot( 2.0 );
				robot.spinner.setPower( 0.0 );
				robot.sleepRobot( 0.5 );
				telemetry.addLine( "duckSpinAssist loop " + (i++) );
				telemetry.update( );
			}
		} ) );
		spinnerThread.get( 0 ).start( );
	}

	/**
	 * slants the bucket depending on the height of the lift
	 */
	public void autoSlantBucket( ) {
		if( robot.lift.getPositionInch( ) < Lift.LIFT_SWITCH_LIMIT )
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_INTAKE );
		else if( robot.lift.getPositionInch( ) >= Lift.LIFT_SWITCH_LIMIT && robot.lift.getPositionInch( ) > prevLiftPos )
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_MOVING );
		prevLiftPos = robot.lift.getPositionInch( );
	}

	/**
	 * sets the position of the bucket based on player 1 input
	 * <br/><br/>
	 * dpad up - intake
	 * <br/>
	 * dpad down - dump
	 * <br/>
	 * back button - exit lift loops
	 */
	public void setBucketPosition( ) {
		if( gamepad1.dpad_up )
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_INTAKE );
		else if( gamepad1.dpad_down )
			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_DUMP );
		else if( gamepad1.back || gamepad2.back )
			robot.lift.exitLoops( 250 );
	}

}