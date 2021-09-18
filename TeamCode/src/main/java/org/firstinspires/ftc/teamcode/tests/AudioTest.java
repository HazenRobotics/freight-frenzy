package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;

@TeleOp(name = "AudioTest", group = "Test")
@Disabled
public class AudioTest extends OpMode {

	GamepadEvents gamepad1;
	SoundLibrary library;

	boolean showAudios = true;

	@Override
	public void init( ) {

		Robot.createMatchLogFile( this.getClass( ).getSimpleName( ) );

		gamepad1 = new GamepadEvents( super.gamepad1 );
		library = new SoundLibrary( hardwareMap );

		telemetry.addLine( "Initialization Complete" );
		telemetry.update( );

		Robot.writeToMatchFile( "Init Finished", true );
	}

	@Override
	public void loop( ) {

		if( gamepad1.a.onPress( ) )
			telemetry.addLine( SoundLibrary.playAudio( "pew_default" ) );

		if( gamepad1.b.onPress( ) )
			telemetry.addLine( SoundLibrary.playRandomPew( ) );

		if( gamepad1.x.onPress( ) )
			telemetry.addLine( SoundLibrary.playRandomSound( ) );

		if( gamepad1.y.onPress( ) )
			telemetry.addLine( SoundLibrary.playAudio( "ps_startup" ) );

		if( gamepad1.right_bumper.onPress( ) )
			showAudios = !showAudios;

		if( showAudios )
			telemetry.addLine( SoundLibrary.getAudios( ) );

		telemetry.update( );
		gamepad1.update( );
	}
}
