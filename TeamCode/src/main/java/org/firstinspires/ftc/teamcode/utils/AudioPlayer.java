package org.firstinspires.ftc.teamcode.utils;

import android.content.Context;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AudioPlayer {

	public MediaPlayer mediaPlayer;

	private boolean audioFound = false;

	private int audioID = 0;

	private String audioName = "";

	private float volume = 1f;

	private HardwareMap hardwareMap;

	public AudioPlayer( HardwareMap hardwareMap, String audioName ) {

		initSound( hardwareMap, audioName, 1f );
	}

	public AudioPlayer( HardwareMap hardwareMap, String audioName, float volume ) {

		initSound( hardwareMap, audioName, volume );
	}

	public void initSound( HardwareMap hardwareMap, String audioName, float volume ) {

		this.hardwareMap = hardwareMap;

		this.audioName = audioName;

		this.volume = volume;

		Context context = this.hardwareMap.appContext;

		// Determine Resource IDs for sounds built into the RC application.
		audioID = context.getResources( ).getIdentifier( this.audioName, "raw", context.getPackageName( ) );

		mediaPlayer = MediaPlayer.create( context, audioID );

		audioFound = mediaPlayer != null;

		setVolume( volume );

//		if( audioFound )
//			mediaPlayer.setAudioAttributes( new AudioAttributes.Builder( )
//					.setContentType( AudioAttributes.CONTENT_TYPE_MUSIC )
//					.build( ) );

	}

	/**
	 * @return whether the audio was played
	 */
	public boolean play( ) {
		if( audioFound )
			mediaPlayer.start( ); // starts the audio
		return audioFound;
	}

	/**
	 * Note: will return false (no matter what) if the audio wasn't found
	 *
	 * @return whether the audio is playing
	 */
	public boolean isPlaying( ) {
		if( audioFound )
			return mediaPlayer.isPlaying( );
		return false;
	}

	public void pause( ) {
		if( audioFound )
			mediaPlayer.pause( ); // pauses the audio
	}

	public void prepareAsync( ) {
		mediaPlayer.prepareAsync( );
	}

	public void seekTo( int milliseconds ) {
		if( audioFound )
			mediaPlayer.seekTo( milliseconds );
	}

	public boolean found( ) {
		return audioFound;
	}

	public void setLooping( boolean loop ) {
		if( audioFound )
			mediaPlayer.setLooping( loop );
	}

	/**
	 * Note: will return false (no matter what) if the audio wasn't found
	 *
	 * @return whether the MediaPlayer is set to loop the media playing
	 */
	public boolean isLooping( ) {
		if( audioFound )
			return mediaPlayer.isLooping( );
		return false;
	}

	public void stop( ) {

		if( audioFound )
			mediaPlayer.stop( );
	}

	public void reset( ) {
		mediaPlayer.reset( );
	}

	public void end( ) {
		mediaPlayer.release( );
		mediaPlayer = null;
	}


	public void setVolume( float volume ) {
		setVolume( volume, volume );
	}

	public void setVolume( float leftVolume, float rightVolume ) {
		if( audioFound )
			mediaPlayer.setVolume( volume, volume );
	}


	public int getAudioID( ) {
		return audioID;
	}

	public String getName( ) {
		return audioName;
	}

}
