package org.firstinspires.ftc.teamcode.teleops;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.text.DecimalFormat;
import java.util.ArrayList;
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

@TeleOp(name = "TippyBotTeleOp", group = "TeleOp")
public class TippyBotTeleOp extends OpMode {

	GamepadEvents player1;
	GamepadEvents player2;

	RRTippyBot robot;

	DecimalFormat df = new DecimalFormat( "0.###" );

	double minDrive = 0.5, maxDrive = 0.8;
	double minStrafe = 0.7, maxStrafe = 1.0;
	double minRotate = 0.5, maxRotate = 1.0;

	double intakePower = 0.6;

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

		robot = new RRTippyBot( this );

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


		//gamepad inputs
		robot.mecanumDrive.drive( -gamepad1.left_stick_y * (gamepad1.left_stick_button ? maxDrive : minDrive),
				gamepad1.left_stick_x * (gamepad1.left_stick_button ? maxStrafe : minStrafe),
				gamepad1.right_stick_x * (gamepad1.right_stick_button ? maxRotate : minRotate) );

		// intake [right - in | left - out]
		if( player1.left_bumper.onPress( ) || player2.left_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) < 0 ? 0 : -intakePower );
		else if( player1.right_bumper.onPress( ) || player2.right_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) > 0 ? 0 : intakePower );

		// bucket control [dup - intake | ddown - dump | back - exit loops]
		if( gamepad1.dpad_up ) // parallel, intake
			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
		else if( gamepad1.dpad_down ) // like -45°, dump
			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_DUMP );

		// exits all loops inside lift methods
		if( gamepad1.back || gamepad2.back )
			robot.lift.exitLoops( 250 );

		// if you touch either of the triggers enough, it will exit the driver assist method
		if( gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5 )
			inDriverAssist = false;

		// lift power control
		if( !inDriverAssist ) {
			if( gamepad1.right_trigger + gamepad1.left_trigger > 0 ) {
				robot.lift.setTeleOPower( gamepad1.right_trigger - gamepad1.left_trigger );
			} else {
				robot.lift.setTeleOPower( 0 );
				robot.lift.disableMotorIfUnused( );
			}
		}

		// capper position
//		if( player1.y.onPress( ) ) // toggle between min/max
//			capperPosition = capperPosition > 0.5 ? 0.0 : 1.0;
//		else
		if( player1.a.onPress( ) ) // toggle between hold/pickup
			capperPosition = capperPosition >= RRTippyBot.CAPPER_HOLD + 0.05 ?  RRTippyBot.CAPPER_HOLD : RRTippyBot.CAPPER_PICKUP; // prep the capper for the shipping element

		if( gamepad2.y )
			capperPosition -= 0.01;
		else if( gamepad2.a )
			capperPosition += 0.01;

		robot.capper.setPosition( capperPosition );

		if( player2.x.onPress( ) )
			robot.odometryLift.raise( );

		// bucket auto slant while moving up '(or below min height)
		autoSlantBucket( ); // no need to since the bucket is already slanted



		// carousel spinner
		// spinner timed intervals
		if( player1.x.onPress( ) ) {
			Log.e( "runSpinnerAssistMethods", "pressed" );
			if( inSpinnerThread ) {
				Log.e( "runSpinnerAssistMethods", "stop" );
				inSpinnerThread = false;
				spinnerThread.get( 0 ).interrupt();
				spinnerThread.clear();
				robot.spinner.setPower( 0 );
			} else {
				Log.e( "runSpinnerAssistMethods", "start" );
				runSpinnerThread( );
			}
		}

		if( player1.b.onPress( ) || player2.b.onPress( ) ) // toggles power
			robot.spinner.setPower( Math.abs( robot.spinner.getPower( ) ) < 0.1 ? spinnerPower : 0 );

		if( player1.dpad_right.onPress( ) || player2.dpad_right.onPress( ) ) // switches direction
			spinnerPower *= -1;

		// reset the lift position to its current zero position
		if( gamepad1.ps )
			robot.lift.resetLift( );

		addControlTelemetry( );
		addInfoTelemetry( );

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
		telemetry.addLine( "Drive: [Gp1] left stick y (axis)" );
		telemetry.addLine( "Strafe: [Gp1] left stick x (axis)" );
		telemetry.addLine( "Rotate: [Gp1] right stick x (axis)" );
		telemetry.addLine( "Intake: [Gp1/2] right/left bumper (in/out)" );
		telemetry.addLine( "Bucket Position: [Gp1] dpad up/down (intake/dump)" );
		telemetry.addLine( "Lift Up: [Gp1] right/left triggers (up/down)" );
		telemetry.addLine( "Grabber Open/Closed Toggle: [Gp1] x" );
//		telemetry.addLine( "Capper Max/Min Toggle: [Gp1] y" ); // y is unused
		telemetry.addLine( "Capper Hold/Pickup Toggle: [Gp1] a" );
		telemetry.addLine( "Capper Position: [Gp2] y/a (increase/decrease)" );
		telemetry.addLine( "Spinner Power Toggle: [Gp1/2] b" );
		telemetry.addLine( "Spinner Direction Toggle: [Gp1/2] dpad right" );
		telemetry.addLine( "Reset Lift Position: [Gp1] ps" );
		telemetry.addLine( "Exit Loops: [Gp1/2] back" );
		telemetry.addLine( );

		/*

		          Controls:
		Drive: [Gp1] left stick y (axis)
		Strafe: [Gp1] left stick x (axis)
		Rotate: [Gp1] right stick x (axis)
		Intake: [Gp1] right bumper
		Outtake: [Gp1] left bumper
		Bucket Position: [Gp1] dpad up/down (intake/dump)
		Lift Up: [Gp1] right/left triggers (up/down)
		Capper Max/Min Toggle: [Gp1] y
		Capper Hold/Pickup Toggle: [Gp1] a
		Capper Position: [Gp2] y/a (increase/decrease)
		Spinner Power Toggle: [Gp1/2] b
		Spinner Direction Toggle: [Gp1/2] dpad right
		Reset Lift Position: [Gp1] ps
		Exit Loops: [Gp1/2] back


		 */


	}

	public void addInfoTelemetry( ) {


		telemetry.addLine( "Intaken blocks: " + robot.intake.getIntakenBlocks( ) );
		telemetry.addLine( );
		telemetry.addLine( "Spinner Velocity " + robot.spinner.getVelocity( ) );
		telemetry.addLine( );
		telemetry.addLine( "front left position: " + df.format( robot.mecanumDrive.getFrontLeftPosition( ) ) );
		telemetry.addLine( "back left position: " + df.format( robot.mecanumDrive.getBackLeftPosition( ) ) );
		telemetry.addLine( "front right position: " + df.format( robot.mecanumDrive.getFrontRightPosition( ) ) );
		telemetry.addLine( "back right position: " + df.format( robot.mecanumDrive.getBackRightPosition( ) ) );
		telemetry.addLine( );
		telemetry.addLine( "longitudinal position (ticks, in): " +  robot.encoderTracker.getLongitudinalPosition( ) + ", " + df.format( robot.getLongitudinalPosition( ) ) );
		telemetry.addLine( "lateral position (ticks, in): " + robot.encoderTracker.getLateralPosition( ) + ", " + df.format( robot.getLateralPosition( ) ) );

	}

	public void runSpinnerThread( ) {

		// note: 9 ducks in 25 seconds (10 if capstone)
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
			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
		else if( robot.lift.getPositionInch( ) >= Lift.LIFT_SWITCH_LIMIT && robot.lift.getPositionInch( ) > prevLiftPos )
			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_MOVING );
		prevLiftPos = robot.lift.getPositionInch( );
	}






// unused older code

	// Grabber open/closed
//		if( player1.y.onPress( ) ) { // toggles position
//			if( robot.grabber.getPosition( ) > 0.5 )
//				robot.grabber.setPosition( 0.0 );
//			else
//				robot.grabber.setPosition( 1 );
//	}

	// driver assist methods
	// lift presets
//		if( gamepad2.dpad_up ) {
//			inDriverAssist = true;
//			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
//		} else if( gamepad2.dpad_left ) {
//			inDriverAssist = true;
//			robot.lift.setHeightVelocity( 850, 0 );
//			robot.bucket.setAngle( RRHexBot.BUCKET_ANGLE_INTAKE );
//		} else if( gamepad2.dpad_down ) {
//			inDriverAssist = true;
//			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
//		}

}