package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.Audio;
import org.firstinspires.ftc.teamcode.utils.AudioPlayer;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;

@TeleOp(name = "MediaTest", group = "Test")
@Disabled
public class MediaTest extends OpMode {

	GamepadEvents gamepad;
	SoundLibrary library;

	int selected = 0;
	int prevSelected = -1;
	String[] audios;

	AudioPlayer test;

	boolean wasPlaying = false;
	boolean wasPaused = false;
	boolean wasLooping = false;

	boolean showTestEvents = false;

	@Override
	public void init( ) {

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		gamepad = new GamepadEvents( super.gamepad1 );
		library = new SoundLibrary( hardwareMap );

		audios = SoundLibrary.getAudioNames( );

		telemetry.addLine( "x - random" );
		telemetry.addLine( "y - ps startup" );
		telemetry.addLine( "a - play select" );
		telemetry.addLine( "dpad up/down - select up/down" );
		telemetry.addLine( "::" + SoundLibrary.audioList.size( ) );
		telemetry.update( );

		telemetry.addLine( "" );
		test = new AudioPlayer( hardwareMap, "ps_startup", 1f );
//		test.prepareAsync( );

		Audio startup = new Audio( hardwareMap, "gamecube_startup", 1f );
		startup.play( );

		Robot.writeToMatchFile( "Init Finished", true );
	}

	@Override
	public void loop( ) {

		if( gamepad.x.onPress( ) ) {
			telemetry.addLine( SoundLibrary.playRandomSound( ) );
			telemetry.update( );
		}

		if( gamepad.y.onPress( ) ) {
			telemetry.addLine( SoundLibrary.playAudio( "ps_startup" ) );
			telemetry.update( );
		}

		if( gamepad.a.onPress( ) ) {
			telemetry.addLine( SoundLibrary.playAudio( audios[selected] ) );
			telemetry.update( );
		}

		if( gamepad.b.onPress( ) ) {
			if( test.isPlaying( ) ) {
				test.stop( );
				telemetry.addLine( test.getName( ) + " playing: " + false );
			} else
				telemetry.addLine( test.getName( ) + " playing: " + test.play( ) );

			telemetry.update( );
		}

		boolean playing = false,
				paused = false,
				looping = false;

		if( showTestEvents ) {
			playing = test.isPlaying( );
			paused = test.isPaused( );
			looping = test.isLooping( );
		} else {
			AudioPlayer current = SoundLibrary.getAudioPlayer( audios[selected] );
			if( current != null ) {
				playing = current.isPlaying( );
				paused = current.isPaused( );
				looping = current.isLooping( );
			}
		}

		if( playing != wasPlaying || paused != wasPaused || looping != wasLooping ) {

			wasPlaying = playing;
			wasPaused = paused;
			wasLooping = looping;

			telemetry.addLine( "Test Playing: " + playing );
			telemetry.addLine( "Test Paused: " + paused );
			telemetry.addLine( "Test Looping: " + looping );
			telemetry.update( );
		}

		if( gamepad.dpad_up.onPress( ) ) {
			if( selected - 1 < 0 )
				selected = audios.length - 1;
			else
				selected--;
		}

		if( gamepad.dpad_down.onPress( ) ) {
			if( selected + 1 > audios.length - 1 )
				selected = 0;
			else
				selected++;
		}

		if( prevSelected != selected ) {
			prevSelected = selected;
			for( int i = 0; i < audios.length; i++ )
				telemetry.addLine( (i == selected ? "* " : "- ") + audios[i] );
			telemetry.update( );
		}

//		telemetry.update( );
		gamepad.update( );
	}
}
