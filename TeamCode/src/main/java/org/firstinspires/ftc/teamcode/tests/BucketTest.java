package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.HexBot;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
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

@TeleOp(name = "BucketTest", group = "Test")
//@Disabled
public class BucketTest extends OpMode {

	Bucket bucket;

	GamepadEvents player;

	double bucketAngle = 0;

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating..." );
		telemetry.update( );

		player = new GamepadEvents( gamepad1 );

//		Robot.createMatchLogFile( "HexRobotTeleOp" );

		bucket = new Bucket( hardwareMap, "bucket", RRTippyBot.BUCKET_ANGLE_INTAKE - RRTippyBot.BUCKET_ANGLE_RANGE, RRTippyBot.BUCKET_ANGLE_RANGE );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		if( player.dpad_up.onPress( ) )
			bucketAngle += 22.5;
		else if( player.dpad_down.onPress( ) )
			bucketAngle -= 22.5;

		if( gamepad1.a )
			bucket.setAngle( bucketAngle );

		telemetry.addLine( "press a to set position" );
		telemetry.addLine( "-----------------------" );
		telemetry.addLine( "bucketAngle" + bucketAngle );
		telemetry.addLine( "bucketPosition" + bucket.getPosition( ) );

		//updates
		telemetry.update( );
		player.update( );
	}
}