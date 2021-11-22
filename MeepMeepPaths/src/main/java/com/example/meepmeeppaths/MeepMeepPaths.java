package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;

public class MeepMeepPaths {
	public static void main(String args[]) {
		MeepMeep mm = new MeepMeep( 80 )
				.setBackground( MeepMeep.Background.FIELD_FREIGHT_FRENZY )
				.setTheme( new ColorSchemeBlueDark() )
				.setBackgroundAlpha( 1f )
				.setConstraints( 30, 30, Math.toRadians( 60 ), Math.toRadians( 60 ), 17 )
				.followTrajectorySequence( drive ->
						drive.trajectorySequenceBuilder( new Pose2d( 0, 0, 0 ) )
						.build()
						);
	}

}