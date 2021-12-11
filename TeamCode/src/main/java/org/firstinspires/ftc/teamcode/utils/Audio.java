package org.firstinspires.ftc.teamcode.utils;

import android.content.res.Resources;
import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.Robot;

public class Audio {

	private boolean audioFound = false;

	private int audioID = 0;

	private String audioName = "";

	private float volume = 1;

	private HardwareMap hardwareMap;

	public Audio( HardwareMap hardwareMap, String audioName ) {

		initSound( hardwareMap, audioName, 1f );
	}

	public Audio( HardwareMap hardwareMap, String audioName, float volume ) {

		initSound( hardwareMap, audioName, volume );
	}

	private void initSound( HardwareMap hardwareMap, String audioName, float volume ) {

		this.hardwareMap = hardwareMap;

		this.audioName = audioName;

		this.volume = volume;

		// Determine Resource IDs for sounds built into the RC application.
		audioID = this.hardwareMap.appContext.getResources( ).getIdentifier( this.audioName, "raw", this.hardwareMap.appContext.getPackageName( ) );

		//Robot.writeToDefaultFile( "" + audioID, true, true );

		// Determine if sound resources are found.
		// Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
		try {
			if( audioID != 0 )
				audioFound = SoundPlayer.getInstance( ).preload( this.hardwareMap.appContext, audioID );
		} catch( Resources.NotFoundException e ) {
			Robot.writeToDefaultFile( "NotFoundException :: " + e.getLocalizedMessage( ), true, true );
			Robot.writeToMatchFile( "NotFoundException :: " + e.getLocalizedMessage( ), true );
		}

	}

	public void play( ) {
		String textToWrite = (audioFound ? "Successfully played" : "Failed to find & play") + " audio " + audioName;
		Robot.writeToDefaultFile( textToWrite, true, true );
		Robot.writeToMatchFile( textToWrite, true );

		setMasterVolume( volume > 0 ? volume : 1 );

		SoundPlayer.getInstance( ).startPlaying( hardwareMap.appContext, audioID );
	}

	public static void stopAllAudios( ) {
		SoundPlayer.getInstance( ).stopPlayingAll( );
	}

	/**
	 * @return if the audio was found (if it even exists)
	 */
	public boolean found( ) {
		return audioFound;
	}

	public String getName( ) {
		return audioName;
	}

	public int getID( ) {
		return audioID;
	}

	public float getVolume( ) {
		return volume;
	}

	// master volume getters and setters

	public float getMasterVolume( ) {
		return SoundPlayer.getInstance( ).getMasterVolume( );
	}

	public static void setMasterVolume( float masterVolume ) {
		SoundPlayer.getInstance( ).setMasterVolume( masterVolume );
	}


}
