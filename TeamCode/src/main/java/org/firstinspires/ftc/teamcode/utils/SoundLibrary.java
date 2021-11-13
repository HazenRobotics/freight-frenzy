/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class SoundLibrary {

	private static HardwareMap hardwareMap;

	private static final ArrayList<Audio> audioList = new ArrayList<>( );

	public SoundLibrary( HardwareMap hw ) {

		hardwareMap = hw;

		initSounds( );
	}

	private void initSounds( ) {

		// other audios
		audioList.add( new Audio( "ps_startup", 0.5f, hardwareMap ) );
		audioList.add( new Audio( "slurp_yummy", 1, hardwareMap ) );
		audioList.add( new Audio( "fine_addition", 1, hardwareMap ) );
		audioList.add( new Audio( "have_hulk", hardwareMap ) );
		audioList.add( new Audio( "hello_there_startup", hardwareMap ) );
		audioList.add( new Audio( "my_precious", hardwareMap ) );
		audioList.add( new Audio( "falcon_punch_smash", hardwareMap ) );
		audioList.add( new Audio( "seismic_charge_smash", hardwareMap ) );
		audioList.add( new Audio( "smash", hardwareMap ) );
		audioList.add( new Audio( "nooo", hardwareMap ) );
		audioList.add( new Audio( "windows_startup", hardwareMap ) );
		audioList.add( new Audio( "gamecube_startup", hardwareMap ) );
		audioList.add( new Audio( "hallelujah_chorus", hardwareMap ) );
		//audioList.add( new Audio("", hardwareMap) );

		//audioList.add( new Audio("gold", hardwareMap) );
		//audioList.add( new Audio("silver", hardwareMap) );

		// checks all of the sounds and removes the ones that aren't found
		for( int i = 0; i < audioList.size( ); i++ )
			if( !audioList.get( i ).exists( ) )
				audioList.remove( i-- );
	}

	public static String playAudio( String audioName ) {
		for( int i = 0; i < audioList.size( ); i++ ) {
			if( audioList.get( i ).getName( ).equals( audioName ) ) {
				audioList.get( i ).play( );
				return "Audio \"" + audioName + "\" :: playing";
			}
		}
		return "Audio \"" + audioName + "\" :: not found";
	}

	public static String playStartup( ) {
		return playAudio( "ps_startup" );
	}

	public static String playRandomStartup( ) {
		return playRandomAudioOfType( "startup" );
	}

	public static String playRandomSmash( ) {
		return playRandomAudioOfType( "smash" );
	}

	public static String playRandomSound( ) {
		int randomPos = (int) (Math.random( ) * audioList.size( ));
		return playAudio( audioList.get( randomPos ).getName( ) );
	}

	public static String[] getAudios( ) {
		String[] audios = new String[audioList.size( )];

		for( int i = 0; i < audioList.size( ); i++ )
			audios[i] = audioList.get( i ).getName( );

		return audios;
	}

	public String toString( ) {
		String audios = "", listSeparator = "\n";

		for( int i = 0; i < audioList.size( ); i++ )
			audios += "- " + audioList.get( i ).getName( ) + (i != audioList.size( ) - 1 ? listSeparator : "");

		return audios;
	}

	public static void stopAllAudios( ) {
		Audio.stopAllAudios( );
	}

	private static String playRandomAudioOfType( String audioType ) {
		ArrayList<Audio> list = new ArrayList<>( );
		for( int i = 0; i < audioList.size( ); i++ )
			if( audioList.get( i ).getName( ).contains( audioType ) )
				list.add( audioList.get( i ) );
		// changed this
		if( list.size( ) == 0 )
			return "No " + audioType + " audio found";
		int randomPos = (int) (Math.random( ) * list.size( ) );
		return playAudio( list.get( randomPos ).getName( ) );
	}
}
