
package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.BasicRobot;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.subsystems.NoodleIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp (name = "TeleOpAllSystemsTest" ,group = "TeleOp")
//@Disabled
public class TeleOpAllSystems extends OpMode {

    // Declare the variables of motors and distance sens

    GamepadEvents gamepad;

    BasicRobot robot;

    @Override
    //put stuff for before the robot is given command (so get ready)
    public void init( ) {

        robot = new BasicRobot( this );
        gamepad = new GamepadEvents( gamepad1 );

        telemetry.addData( "Mode", "waiting for start" );
        telemetry.update( );
    }

    @Override
    //put stuff for after init
    public void loop( ) {

        //gamepad inputs
        robot.mecanumDrive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

        // intake
        if( gamepad1.left_bumper )
            robot.intake.setPower( 1 );
        else if( gamepad1.right_bumper )
            robot.intake.setPower( -1 );

        // lift velocity control
        if( gamepad1.left_trigger > 0 )
            robot.lift.setVelocity( gamepad1.left_trigger * Lift.MAX_VELOCITY );
        else if( gamepad1.right_trigger >= 0 )
            robot.lift.setVelocity( -gamepad1.right_trigger * Lift.MAX_VELOCITY );

        // bucket control
        // multiple positions

        // carousel spinner control
        if( gamepad.x.onPress( ) ) // toggles carousel spinner
            robot.spinner.setPower( robot.spinner.getPower( ) > 0 ? 0 : 1 );
        else if( gamepad.y.onPress( ) )
            robot.spinner.setPower( robot.spinner.getPower( ) < 0 ? 0 : -1 );

        //updates
        telemetry.update( );
        gamepad.update( );
    }

}