package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.subsystems.TwoDBucket;
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

@TeleOp(name = "TwoDBucketTest", group = "Test")
//@Disabled
public class TwoDBucketTest extends OpMode {

	TwoDBucket bucket;

	GamepadEvents player;

	boolean inDuelMode = false;

	double verBucketAngle = 0;
	double horBucketAngle = 0;

	public static double VER_RANGE = 185;
	public static double VER_MAX_ANGLE = 90 - VER_RANGE;

	public static double HOR_RANGE = 135;
	public static double HOR_MAX_ANGLE = -90;

	/*
	range = RRTippyBot.BUCKET_ANGLE_RANGE
	max angle = RRTippyBot.BUCKET_ANGLE_INTAKE - RRTippyBot.BUCKET_ANGLE_RANGE
	 */

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating..." );
		telemetry.update( );

		player = new GamepadEvents( gamepad1 );

		// hwmap, ver name, hor name, ver max angle, hor max angle,
		bucket = new TwoDBucket( hardwareMap, "verBucket", "horBucket", VER_MAX_ANGLE, HOR_MAX_ANGLE, VER_RANGE, HOR_RANGE );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		if( player.dpad_up.onPress( ) )
			verBucketAngle += 22.5;
		else if( player.dpad_down.onPress( ) )
			verBucketAngle -= 22.5;

		if( player.y.onPress( ) )
			horBucketAngle += 22.5;
		else if( player.a.onPress( ) )
			horBucketAngle -= 22.5;

		if( player.left_stick_button.onPress( ) )
			inDuelMode = !inDuelMode;

		if( !inDuelMode ) {
			if( player.dpad_right.onPress( ) )
				bucket.setVerAngle( verBucketAngle );
			else if( player.dpad_left.onPress( ) )
				bucket.setVerPosition( 0.5 );

			if( player.b.onPress( ) )
				bucket.setHorAngle( horBucketAngle );
			else if( player.x.onPress( ) )
				bucket.setHorPosition( 0.5 );
		} else {

			bucket.setVerAngle( ((-gamepad1.right_stick_y + 1) / 2) * VER_RANGE + VER_MAX_ANGLE );
			bucket.setHorAngle( ((gamepad1.right_stick_x + 1) / 2) * HOR_RANGE + HOR_MAX_ANGLE );
//			bucket.setAngle( height, width, 0, true );
		}

		telemetry.addLine( "press a to set position" );
		telemetry.addLine( "-----------------------" );
		telemetry.addLine( "Gamepad Pos: ( " + gamepad1.right_stick_x + ", " + -gamepad1.right_stick_y + ")" );
		telemetry.addLine( "verBucketAngle: " + verBucketAngle );
		telemetry.addLine( "verBucketPosition: " + bucket.getVerPosition( ) );
		telemetry.addLine( "horBucketAngle: " + horBucketAngle );
		telemetry.addLine( "horBucketPosition: " + bucket.getHorPosition( ) );

		//updates
		telemetry.update( );
		player.update( );
	}
}