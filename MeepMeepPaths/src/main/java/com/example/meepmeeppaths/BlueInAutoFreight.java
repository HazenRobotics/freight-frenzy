package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class BlueInAutoFreight implements MeepMeepPath{

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 17, 64.125, Math.toRadians( 270 ) ) )
				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( MeepMeepPath.getHubPosition( -22.5, 270, 1 /*shippingHubHeightToInches( height )*/, true ), Math.toRadians( 270 - 22.5 ) )

				.waitSeconds( 1.2 )

				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, 63.5, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 48, 63.5 ) )
				.lineToConstantHeading( new Vector2d( 18, 63.5 ) )
				.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 1 /*shippingHubHeightToInches( height )*/, true ), Math.toRadians( 270 ) )
				.waitSeconds( 1.2 )

				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, 63.5, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 48, 63.5 ) )
				.lineToConstantHeading( new Vector2d( 18, 63.5 ) )
				.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 1 /*shippingHubHeightToInches( height )*/, true ), Math.toRadians( 270 ) )
				.waitSeconds( 1.2 )

				// park
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, 63.5, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 48, 63.5 ) )
				.build();
	}
}
