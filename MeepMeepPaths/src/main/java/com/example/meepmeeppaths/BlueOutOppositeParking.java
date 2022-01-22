package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueOutOppositeParking implements MeepMeepPath{

	double wallPos = 64.625;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( -30.5, 64.125, Math.toRadians( 270 ) ) )

				//drop preload in alliance hub
				.splineToLinearHeading( MeepMeepPath.getHubPosition(45, 270, 10, true), Math.toRadians( 270 ) )
				.waitSeconds( 0.5 )
				.waitSeconds( 1.2 )

				//duck spin
				.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( -61, 57, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.waitSeconds( 1.2 )

				//pickup duck sweep
				.setTangent( Math.toRadians( 270 ) )
				.splineToLinearHeading( new Pose2d( -48, 63, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( -24, 63 ) )

				//drop duck in alliance hub while turning
				.splineToSplineHeading( new Pose2d( -36, 24, 0 ), Math.toRadians( 270 ) )
				.splineToSplineHeading( MeepMeepPath.getHubPosition( 180, 270, 10, true ), Math.toRadians( 0 ) )
				.splineToSplineHeading( new Pose2d( 12, 24, Math.toRadians( 180 ) ), Math.toRadians( 90 ) )

				//travel over barrier
				.lineToConstantHeading( new Vector2d( 12, 38 ) )
				.lineToConstantHeading( new Vector2d( 48, 38 ) )

				//drive to other side to prepare for tele-op
				.lineToSplineHeading( new Pose2d( wallPos, 38, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( wallPos, -20 ) )

				.build();
	}
}
