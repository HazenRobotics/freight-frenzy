package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.HexBot;
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

@TeleOp(name = "LiftTeleOpTest", group = "TeleOp")
//@Disabled
public class LiftTeleOpTest extends OpMode {

	HexBot robot;

	GamepadEvents player;

	double distance = 0;

	double height = 0;

	@Override
	public void init( ) {

		gamepad1.rumble( 2000 );

		robot = new HexBot( this );

		player = new GamepadEvents( gamepad1 );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		if( player.dpad_up.onPress( ) )
			height += 4;
		else if( player.dpad_down.onPress( ) )
			height -= 4;

		else if( player.dpad_right.onPress( ) )
			distance += 2;
		else if( player.dpad_left.onPress( ) )
			distance -= 2;

		if( gamepad1.a )
			robot.lift.setLiftHeightVel( 500, height );
		else if( gamepad1.y )
			robot.lift.runDistanceVelAsync( 750, distance );

		telemetry.addLine( "Height (a): " + height );
		telemetry.addLine( "Distance (y): " + distance );
		telemetry.addLine( "Lift Position: " + Lift.getCurrentPosition( true ) );

		//updates
		telemetry.update( );
		player.update( );
	}

}