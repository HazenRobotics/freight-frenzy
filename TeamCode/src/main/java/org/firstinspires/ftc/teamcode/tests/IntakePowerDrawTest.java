package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.NoodleIntake;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.Logger;

import java.util.Date;

@TeleOp(name = "IntakePowerDrawTest", group = "Test")
@Disabled
public class IntakePowerDrawTest extends OpMode {

	NoodleIntake intake;
	Lift lift;

	String fileName = "*powerDraw.csv";

	Date date = new Date( );

	GamepadEvents player;

	boolean logging = false;

	long startTime = 0;

	double intakePower = 0.7;

	LynxModule controlHub;

	@Override
	public void init( ) {

		player = new GamepadEvents( gamepad1 );

		intake = new NoodleIntake( hardwareMap, "intake" );
		lift = new Lift( hardwareMap, "lift", true, 2.375, (38.2 / 25.4) / 2, 55, AngleUnit.DEGREES );

		Logger.writeAFile( fileName, "Time,Power,Motor Current,Hub Current", false, false );

		controlHub = (LynxModule) hardwareMap.get( LynxModule.class, "Control Hub" );

		telemetry.addLine( "Init finished!" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

//		intake.setPower( -player.right_trigger.getTriggerValue( ) );

		if( player.left_bumper.onPress( ) )
			intake.setPower( intake.getPower( ) > 0 ? 0 : intakePower );
		else if( player.right_bumper.onPress( ) )
			intake.setPower( intake.getPower( ) < 0 ? 0 : -intakePower );

		if( player.a.onPress( ) ) {
			logging = !logging;
			startTime = System.currentTimeMillis( );
		}

		if( logging )
			Logger.writeAFile( fileName, formatData( System.currentTimeMillis( ) - startTime, intake.getPower( ), intake.getCurrent( ), controlHub.getCurrent( CurrentUnit.AMPS ) ), true, false );

		telemetry.addLine( "intake: right bumper" );
		telemetry.addLine( "outtake: right bumper" );
		telemetry.addLine( "start logging: a" );
		telemetry.addLine( );
		telemetry.addLine( "logging: " + logging );
		telemetry.addLine( "time: " + time );
		telemetry.addLine( );
		telemetry.addLine( "intake power: " + intake.getPower( ) );
		telemetry.addLine( "intake current: " + intake.getCurrent( ) );
		telemetry.addLine( "lift current: " + lift.getCurrent( ) );
		telemetry.addLine( "battery level: " + controlHub.getCurrent( CurrentUnit.AMPS ) );

		telemetry.update( );
		player.update( );
	}

/*	public String formatData( long time, double power, double current ) {
		return time + "," + power + "," + current;
	}*/

	public String formatData( long time, double... data ) {
		StringBuilder text = new StringBuilder( time + "," );
		for( double datum : data )
			text.append( datum ).append( "," );
		return text.substring( 0, text.length( ) - 2 );
	}


}
