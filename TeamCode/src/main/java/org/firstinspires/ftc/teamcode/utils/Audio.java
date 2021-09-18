package org.firstinspires.ftc.teamcode.utils;

import android.content.res.Resources;
import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.Robot;

public class Audio {

	private boolean audioFound = false;
	private final boolean isPlaying = false;

	private int audioID = 0;

	private String audioName = "";

	private final HardwareMap hardwareMap;

	public Audio( HardwareMap hw ) {

		hardwareMap = hw;
	}

	public Audio( String name, HardwareMap hw ) {

		audioName = name;

		hardwareMap = hw;

		initSound( 1 );
	}

	public Audio( String name, float masterVolume, HardwareMap hw ) {

		audioName = name;

		hardwareMap = hw;

		initSound( masterVolume );
	}

	public void initSound( float masterSound ) {

		// Determine Resource IDs for sounds built into the RC application.
		audioID = hardwareMap.appContext.getResources( ).getIdentifier( audioName, "raw", hardwareMap.appContext.getPackageName( ) );

		//Robot.writeToDefaultFile( "" + audioID, true, true );

		// Determine if sound resources are found.
		// Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
		try {
			if( audioID != 0 )
				audioFound = SoundPlayer.getInstance( ).preload( hardwareMap.appContext, audioID );
		} catch( Resources.NotFoundException e ) {
			Log.e( "|-|-|-|", e.getLocalizedMessage( ) );
			Robot.writeToDefaultFile( "NotFoundException :: " + e.getLocalizedMessage( ), true, true );
		}

		setMasterVolume( masterSound );
	}

	public void play( ) {
		String textToWrite = (audioFound ? "Successfully played" : "Failed to find & play") + " audio " + audioName;
		Robot.writeToDefaultFile( textToWrite, true, true );

		SoundPlayer.getInstance( ).startPlaying( hardwareMap.appContext, audioID );
	}

	public String getName( ) {
		return audioName;
	}

	public int getID( ) {
		return audioID;
	}

	public boolean exists( ) {
		return audioFound;
	}

	public float getMasterVolume( ) {
		return SoundPlayer.getInstance( ).getMasterVolume( );
	}

	public void setMasterVolume( float masterVolume ) {
		SoundPlayer.getInstance( ).setMasterVolume( masterVolume );
	}

	public static void stopAllAudios( ) {
		SoundPlayer.getInstance( ).stopPlayingAll( );
	}


}
