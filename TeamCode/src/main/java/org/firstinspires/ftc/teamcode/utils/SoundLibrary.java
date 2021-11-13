package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class SoundLibrary {

	private static HardwareMap hardwareMap;

	private static final ArrayList<Audio> audioList = new ArrayList<>( );

	public SoundLibrary( HardwareMap hardwareMap ) {

		SoundLibrary.hardwareMap = hardwareMap;

		initSounds( );
	}

	private void initSounds( ) {

		// other audios
		audioList.add( new Audio( hardwareMap, "ps_startup", 0.5f ) );
		audioList.add( new Audio( hardwareMap, "slurp_yummy", 1f ) );
		audioList.add( new Audio( hardwareMap, "fine_addition", 1f ) );
		audioList.add( new Audio( hardwareMap, "have_hulk", 1f ) );
		audioList.add( new Audio( hardwareMap, "hello_there_startup", 1f ) );
		audioList.add( new Audio( hardwareMap, "my_precious", 1f ) );
		audioList.add( new Audio( hardwareMap, "falcon_punch_smash", 1f ) );
		audioList.add( new Audio( hardwareMap, "seismic_charge_smash", 1f ) );
		audioList.add( new Audio( hardwareMap, "smash", 1f ) );
		audioList.add( new Audio( hardwareMap, "nooo", 1f ) );
		audioList.add( new Audio( hardwareMap, "windows_startup", 1f ) );
		audioList.add( new Audio( hardwareMap, "gamecube_startup", 1f ) );
		audioList.add( new Audio( hardwareMap, "hallelujah_chorus", 1f ) );
		//audioList.add( new Audio("", hardwareMap) );

		//audioList.add( new Audio("gold", hardwareMap) );
		//audioList.add( new Audio("silver", hardwareMap) );

		// checks all of the sounds and removes the ones that aren't found
		for( int i = 0; i < audioList.size( ); i++ )
			if( !audioList.get( i ).found( ) )
				audioList.remove( i-- );
	}

	/**
	 *
	 * @param audioName the name of the audio to play
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playAudio( String audioName ) {
		for( int i = 0; i < audioList.size( ); i++ ) {
			if( audioList.get( i ).getName( ).equals( audioName ) ) {
				audioList.get( i ).play( );
				return "Audio \"" + audioName + "\" :: playing";
			}
		}
		return "Audio \"" + audioName + "\" :: not found";
	}

	/**
	 * plays the playstation startup (the audio file with the name "ps_startup")
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playStartup( ) {
		return playAudio( "ps_startup" );
	}

	/**
	 * plays a random startup (any audio file containing the name "startup")
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playRandomStartup( ) {
		return playRandomAudioOfType( "startup" );
	}

	/**
	 * plays a random smash (any audio file containing the name "smash")
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playRandomSmash( ) {
		return playRandomAudioOfType( "smash" );
	}

	/**
	 * plays a random sound loaded
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playRandomSound( ) {
		int randomPos = (int) (Math.random( ) * audioList.size( ));
		return playAudio( audioList.get( randomPos ).getName( ) );
	}

	/**
	 * @return a list with all the loaded audio names
	 */
	public static String[] getAudioNames( ) {
		String[] audios = new String[audioList.size( )];

		for( int i = 0; i < audioList.size( ); i++ )
			audios[i] = audioList.get( i ).getName( );

		return audios;
	}

	/**
	 * @return a string with a list of all the names separated by line separators & dashes
	 */
	public String toString( ) {
		String audios = "", listSeparator = "\n";

		for( int i = 0; i < audioList.size( ); i++ )
			audios += "- " + audioList.get( i ).getName( ) + (i != audioList.size( ) - 1 ? listSeparator : "");

		return audios;
	}

	public static void stopAllAudios( ) {
		Audio.stopAllAudios( );
	}

	/**
	 * @param audioType the string to look for audios containing it
	 * @return a string saying whether the audio is playing or was not found
	 */
	private static String playRandomAudioOfType( String audioType ) {
		ArrayList<Audio> list = new ArrayList<>( );
		for( int i = 0; i < audioList.size( ); i++ )
			if( audioList.get( i ).getName( ).contains( audioType ) )
				list.add( audioList.get( i ) );
		// changed this
		if( list.size( ) > 0 )
			return playAudio( list.get( (int) (Math.random( ) * list.size( )) ).getName( ) );
		return "No " + audioType + " audio found";
	}
}
