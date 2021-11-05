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

	GamepadEvents gamepad;
	SoundLibrary library;

	int selected = 0;
	String[] audios;

	@Override
	public void init( ) {

		Robot.createMatchLogFile( this.getClass( ).getSimpleName( ) );

		gamepad = new GamepadEvents( super.gamepad1 );
		library = new SoundLibrary( hardwareMap );

		audios = SoundLibrary.getAudios( );

		telemetry.addLine( "x - random" );
		telemetry.addLine( "y - ps startup" );
		telemetry.addLine( "a - selected" );
		telemetry.update( );

		Robot.writeToMatchFile( "Init Finished", true );
	}

	@Override
	public void loop( ) {

		if( gamepad.x.onPress( ) )
			telemetry.addLine( SoundLibrary.playRandomSound( ) );

		if( gamepad.y.onPress( ) )
			telemetry.addLine( SoundLibrary.playAudio( "ps_startup" ) );

		if( gamepad.a.onPress( ) )
			telemetry.addLine( SoundLibrary.playAudio( audios[selected] ) );

		if( gamepad.dpad_up.onPress( ) && selected - 1 >= 0 )
			selected--;

		if( gamepad.dpad_down.onPress( ) && selected + 1 <= audios.length - 1 )
			selected++;

		for( int i = 0; i < audios.length; i++ )
			telemetry.addLine( (i == selected ? "* " : "- ") + audios[i] );

		telemetry.update( );
		gamepad.update( );
	}
}
