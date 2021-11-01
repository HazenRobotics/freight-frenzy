package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.HexBot;

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
@Disabled
public class LiftTeleOpTest extends OpMode {

	HexBot robot;

	@Override
	public void init( ) {

		robot = new HexBot( this );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		// lift velocity control
		if( gamepad1.left_trigger > 0 )
			robot.lift.setPower( gamepad1.left_trigger );
		else if( gamepad1.right_trigger > 0 )
			robot.lift.setPower( -gamepad1.right_trigger );
		else
			robot.lift.setPower( 0 );

		telemetry.addLine( "Lift Position: " + robot.lift.getCurrentPosition( ) );

		//updates
		telemetry.update( );
	}

}