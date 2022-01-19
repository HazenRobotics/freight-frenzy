package org.firstinspires.ftc.teamcode.teleops;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GameTimer;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.RGBLights;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;

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

	double intakePower = 0.55; // .6

	double capperPosition = 0.7;

	boolean inDriverAssist = false;

	double prevLiftPos = 0;

	double spinnerVelocity = 325;
	boolean inSpinnerThread = false;
	boolean inThread = false;
	List<Thread> spinnerThread = new ArrayList<>( );
	boolean firstTime = true;

	public static boolean isBlueSide = true;

	private long lightControlEnd = 0;

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		player1 = new GamepadEvents( gamepad1 );
		player2 = new GamepadEvents( gamepad2 );

		robot = new RRTippyBot( this, false );

		addWarnIntakeThread( );

		SoundLibrary.playRandomStartup( );

		Log.e( "Mode", "waiting for start" );
		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );

		Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder( )
				.addStep( 1, 1, 100 )
				.addStep( 0, 0, 100 )
				.addStep( 1, 1, 100 )
				.build( );
		gamepad1.runRumbleEffect( effect );
		robot.lights.showStatus( RGBLights.StatusLights.WAITING );
	}

	@Override
	public void loop( ) {

		if( firstTime ) {
			addWarnEndGameThread( );
			firstTime = false;
		}

		//gamepad inputs
		robot.mecanumDrive.drive( -gamepad1.left_stick_y * (gamepad1.left_stick_button ? maxDrive : minDrive),
				gamepad1.left_stick_x * (gamepad1.left_stick_button ? maxStrafe : minStrafe),
				gamepad1.right_stick_x * (gamepad1.right_stick_button ? maxRotate : minRotate) );

		// intake [right - in | left - out]
		if( player1.left_bumper.onPress( ) || player2.left_bumper.onPress( ) ) {
//			robot.intake.intakeNum( 0.6, 1 );
			robot.intake.setPower( robot.intake.getPower( ) < 0 ? 0 : -intakePower );
		} else if( player1.right_bumper.onPress( ) || player2.right_bumper.onPress( ) )
			robot.intake.setPower( robot.intake.getPower( ) > 0 ? 0 : intakePower );

		// lift control [up - top layer | down - dump | left - shared]
		if( player1.dpad_up.onPress( ) || player2.dpad_up.onPress( ) ) { // lift to top layer (for alliance hub)
			inDriverAssist = true;
			robot.lift.setHeightVelocity( 1400, 21 );
		} else if( /*player1.dpad_left.onPress() ||*/ player2.dpad_left.onPress( ) ) { // lift to middle layer (for shared hub)
			inDriverAssist = true;
			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.MIDDLE );
		} else if( player1.dpad_left.onPress( ) || player2.dpad_right.onPress( ) ) { // lift to bottom layer (for shared hub)
			inDriverAssist = true;
			robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
		} else if( /*player1.dpad_down.onPress() ||*/ player2.dpad_down.onPress( ) ) { // lift to default position
			inDriverAssist = true;
			robot.lift.setDefaultHeightVel( 1200 );
		}

		telemetry.addLine( "" + robot.lift.getVelocity( ) );

		// bucket control
		if( gamepad1.dpad_down || gamepad2.right_trigger > 0.5 ) // like -45°, dump
			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_DUMP );
//		else if( gamepad1.dpad_up ) // parallel, intake
//			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );


		// exits all loops inside lift methods
		if( gamepad1.back || gamepad2.back )
			robot.lift.exitLoops( 250 );

		// if you touch either of the triggers enough, it will exit the driver assist method
		if( gamepad1.right_trigger > 0.3 || gamepad1.left_trigger > 0.3 )
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

		telemetry.addLine( (spinnerVelocity > 0 ? "Blue" : "Red") + " Side Velocity" );
		if(System.currentTimeMillis() > lightControlEnd) {
			if( spinnerVelocity > 0 )
				robot.lights.setPattern( RevBlinkinLedDriver.BlinkinPattern.BLUE );
			else
				robot.lights.setPattern( RevBlinkinLedDriver.BlinkinPattern.RED );
		}


		// capper position
		if( player1.y.onPress( ) ) // toggle between min/max
			capperPosition = capperPosition > 0.5 ? 0.0 : 1.0;
		else if( player1.a.onPress( ) ) // toggle between hold/pickup
			capperPosition = capperPosition >= RRTippyBot.CAPPER_HOLD + 0.05 ? RRTippyBot.CAPPER_HOLD : RRTippyBot.CAPPER_PICKUP; // prep the capper for the shipping element
		if( player1.a.onHeldFor( 2000 ) )
			capperPosition = 1.0;

		if( gamepad2.y )
			capperPosition -= 0.01;
		else if( gamepad2.a )
			capperPosition += 0.01;

		robot.capper.setPosition( capperPosition );

		if( player2.x.onPress( ) )
			robot.odometryLift.raise( );


		// bucket auto slant while moving up/down '(or below min height)
		autoSlantBucket( );


		// carousel spinner
		// spinner timed intervals
		if( player1.x.onPress( ) ) {
			Log.e( "runSpinnerAssistMethods", "pressed" );
			if( inSpinnerThread ) {
				Log.e( "runSpinnerAssistMethods", "stop" );
				stopSpinnerThread( );
			} else {
				Log.e( "runSpinnerAssistMethods", "start" );
				runSpinnerThread( );
			}
		}

		if( player1.b.onPress( ) ) // toggles velocity
			robot.spinner.setPower( Math.abs( robot.spinner.getVelocity( ) ) < 100 ? ( isBlueSide ? spinnerVelocity : -spinnerVelocity ): 0 );

		if( player1.dpad_right.onPress( ) || player2.dpad_right.onPress( ) ) // switches direction
			spinnerVelocity *= -1;


		if( player2.b.onPress( ) )
			home( );

		// reset the lift position to its current zero position
		if( gamepad1.ps || robot.lift.getCurrent( CurrentUnit.AMPS ) > 7.9 )
			robot.lift.resetLift( );

		addControlTelemetry( );
		addInfoTelemetry( );

		//updates
		telemetry.update( );
		player1.update( );
		player2.update( );
	}

	@Override
	public void stop( ) {

		inThread = false;
		if( inSpinnerThread )
			stopSpinnerThread( );
	}

	public void addWarnIntakeThread( ) {
		new Thread( ( ) -> {

			inThread = true;
			int prevIntaken = 0;
			while( inThread ) {

				if( robot.intake.getIntakenBlocks( ) > prevIntaken ) {
					gamepad1.runRumbleEffect( new Gamepad.RumbleEffect.Builder( )
							.addStep( 1, 1, 250 )
							.build( ) );
					lightControlEnd = System.currentTimeMillis() + 200;
					robot.lights.setPattern( RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE );

				}

				prevIntaken = robot.intake.getIntakenBlocks( );

				try {
					Thread.sleep( 100 );
				} catch( InterruptedException ignored ) {
				}
			}

		} ).start( );
	}

	public void addWarnEndGameThread( ) {
		new Thread( ( ) -> {
			inThread = true;
			while( inThread && GameTimer.inEndgame() ) {
				try {
					Thread.sleep( 1000 );
				} catch( InterruptedException ignored ) {
				}
			}

			gamepad1.runRumbleEffect( new Gamepad.RumbleEffect.Builder( )
					.addStep( 1, 1, 70 )
					.addStep( 0, 0, 70 )
					.addStep( 1, 1, 70 )
					.addStep( 0, 0, 70 )
					.addStep( 1, 1, 70 )
					.build( ) );
		} ).start( );
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
//		telemetry.addLine( "Grabber Open/Closed Toggle: [Gp1] x" );
		telemetry.addLine( "Raise Odometry: [Gp2] x" );
		telemetry.addLine( "Capper Max/Min Toggle: [Gp1] y" ); // y is unused
		telemetry.addLine( "Capper Hold/Pickup Toggle: [Gp1] a" );
		telemetry.addLine( "Capper Position: [Gp2] y/a (increase/decrease)" );
		telemetry.addLine( "Spinner Assist Method: [Gp1/2] x" );
		telemetry.addLine( "Spinner Power Toggle: [Gp1/2] b" );
		telemetry.addLine( "Spinner Direction Toggle: [Gp1/2] dpad right" );
		telemetry.addLine( "Reset Lift Position: [Gp1] ps" );
		telemetry.addLine( "Exit Loops: [Gp1/2] back" );

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
		telemetry.addLine( "longitudinal position (ticks, in): " + robot.encoderTracker.getLongitudinalPosition( ) + ", " + df.format( robot.getLongitudinalPosition( ) ) );
		telemetry.addLine( "lateral position (ticks, in): " + robot.encoderTracker.getLateralPosition( ) + ", " + df.format( robot.getLateralPosition( ) ) );

	}

	public void home( ) {

		robot.capper.setPosition( 0.0 );
		robot.lift.setDefaultHeightVel( 1200 );
	}

	public void stopSpinnerThread( ) {

		inSpinnerThread = false;
		spinnerThread.get( 0 ).interrupt( );
		spinnerThread.clear( );
		robot.spinner.setPower( 0 );
	}

	public void runSpinnerThread( ) {

		// note: 9 ducks in 25 seconds (10 if capstone)
		spinnerThread.add( new Thread( ( ) -> {
			int i = 0;
			inSpinnerThread = true;
			while( inSpinnerThread ) {
				robot.spinner.setVelocity( spinnerVelocity );
				// total of 2.5
				robot.sleepRobot( 0.7 );
				robot.spinner.setVelocity( Math.signum( spinnerVelocity ) * 9000 );
				robot.sleepRobot( 0.6 );
				// stop and rest 1 sec
				robot.spinner.setPower( 0.0 );
				robot.sleepRobot( 0.8 );
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

		if( robot.lift.getPositionInch( ) < Lift.LIFT_SWITCH_LIMIT ) // at bottom
			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
		else if( Math.abs( robot.lift.getVelocity( ) ) > 100 )
			robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_MOVING );
	}


// unused older code

	// Grabber open/closed
//		if( player1.y.onPress( ) ) { // toggles position
//			if( robot.grabber.getPosition( ) > 0.5 )
//				robot.grabber.setPosition( 0.0 );
//			else
//				robot.grabber.setPosition( 1 );
//	}

}