package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.RRHexBot;
import org.firstinspires.ftc.teamcode.robots.RRTippyBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.text.DecimalFormat;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the TeleOp period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive TeleOp for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "LiftTeleOpTest", group = "TeleOp")
@Disabled
public class LiftTeleOpTest extends OpMode {

	GamepadEvents player;

	DecimalFormat df = new DecimalFormat( "0.###" );

	Lift lift;

	double bottom, middle, top;

	double distance = 0;

	double height = 0;

	@Override
	public void init( ) {

		lift = new Lift( hardwareMap, "lift", false, 2.375, (38.2 / 25.4) / 2, 50, AngleUnit.DEGREES );

		gamepad1.rumble( 1500 );

		RRTippyBot robot = new RRTippyBot( this, true );
		bottom = robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.LOW );
		middle = robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.MIDDLE );
		top = robot.shippingHubHeightToInches( RRHexBot.ShippingHubHeight.HIGH );

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

		if( player.a.onPress() )
			lift.setHeightVelocity( 500, height );
		else if( player.y.onPress() )
			lift.moveDistanceVelocity( 750, distance, true );

		// default heights
		else if( player.left_bumper.onPress() )
			lift.setHeightVelocity( 1000, bottom );
		else if( player.b.onPress() )
			lift.setHeightVelocity( 1000, middle );
		else if( player.right_bumper.onPress() )
			lift.setHeightVelocity( 1000, top );

		telemetry.addLine( "Default distances (bot, mid, top): " + bottom + ", " + middle + ", " + top );
		telemetry.addLine( "left bump, b, right bump" );
		telemetry.addLine( );
		telemetry.addLine( "Height (a): " + height );
		telemetry.addLine( "Distance (y): " + distance );
		telemetry.addLine( );
		telemetry.addLine( "Lift Power: " + lift.getPower( ) );
		telemetry.addLine( "Lift Moved Position: " + Lift.getPosition( true ) );
		telemetry.addLine( );
		telemetry.addLine( "lift position: " + df.format( lift.getPositionInch( ) ) + " (" + lift.getPosition( ) + ")" );

		//updates
		telemetry.update( );
		player.update( );
	}

}