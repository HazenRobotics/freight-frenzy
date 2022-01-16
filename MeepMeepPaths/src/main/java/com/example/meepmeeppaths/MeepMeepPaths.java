package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class MeepMeepPaths {

	static double robotLength = 13.25;
	static double robotWidth = 12.5;

	static final double tileSize = 23;
	static final double tileConnector = 0.75;

	static final double hubRadius = 9;

	static final double cameraRightIndent = 1.25;

	public static void main( String[] args ) {

		MeepMeep mm = new MeepMeep( 700 )
				.setBackground( MeepMeep.Background.FIELD_FREIGHT_FRENZY )
				.setTheme( new ColorSchemeRedDark() )
//				.set
				.setBackgroundAlpha( 1f )
				.setBotDimensions( robotWidth, robotLength )
				.setConstraints( 30, 30, Math.toRadians( 60 ), Math.toRadians( 60 ), 17 )
				.followTrajectorySequence( drive -> {
					try {
						return ((MeepMeepPath) Class.forName( args[0] ).newInstance()).getTrajectorySequence( drive );
					} catch( InstantiationException | IllegalAccessException | ClassNotFoundException e ) {
						e.printStackTrace( );
					}
					return new TrajectorySequence( new ArrayList<>(  ) );
				} )
				.start( );
	}


}