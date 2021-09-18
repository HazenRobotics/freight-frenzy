package org.firstinspires.ftc.teamcode.utils;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class VuforiaLocalizerPlus extends VuforiaLocalizerImpl {

	public VuforiaLocalizerPlus( Parameters parameters ) {
		super( parameters );
		super.opModeNotifications = new OpModeNotifications( );
	}

	public void close( ) {
		super.close( );
	}

	public void pause( ) {
		super.pauseAR( );
	}

	public void resume( ) {
		super.resumeAR( );
	}

	private class OpModeNotifications extends VuforiaLocalizerImpl.OpModeNotifications {

		@Override
		public void onOpModePostStop( OpMode opMode ) {
			//We don't stop Vuforia between opModes, and instead pause it between opModes
			if( glSurface != null ) {
				glSurface.setVisibility( View.INVISIBLE );
				glSurface.onPause( );
			}
			pauseAR( );
		}

	}
}
