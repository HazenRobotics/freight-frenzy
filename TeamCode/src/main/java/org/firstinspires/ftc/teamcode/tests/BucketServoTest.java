package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name = "BucketServoTest", group = "Test")
@Disabled
public class BucketServoTest extends OpMode {

	GamepadEvents player;

	double servoPosition = 0;

	Servo servo;

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating..." );
		telemetry.update( );

		player = new GamepadEvents( gamepad1 );

//		Robot.createMatchLogFile( "HexRobotTeleOp" );

		servo = hardwareMap.servo.get( "verBucket" );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		servoPosition = (gamepad1.left_stick_x + 1) / 4 + (gamepad1.right_stick_x + 1) / 4;
		servo.setPosition( servoPosition );

		telemetry.addLine( "press a to set position" );
		telemetry.addLine( "-----------------------" );
		telemetry.addLine( "Servo pos: " + servoPosition );

		//updates
		telemetry.update( );
		player.update( );
	}
}