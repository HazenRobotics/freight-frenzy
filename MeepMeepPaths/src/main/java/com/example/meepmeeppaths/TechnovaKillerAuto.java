package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class TechnovaKillerAuto implements MeepMeepPath{

	double wallPos = 64.625;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 17, 64.125, Math.toRadians( 270 ) ) )
				// move to dump initial block in designated layer
				/*.addTemporalMarker( ( ) -> {
			robot.liftToShippingHubHeight( height );
		} )*/
				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( MeepMeepPath.getHubPosition( -22.5, 270, 10, true ), Math.toRadians( 270 - 22.5 ) )
				/*.addTemporalMarker( ( ) -> {
					robot.dumpBucket( );
					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )*/

				.waitSeconds( 0.8 )
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
					/*robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
					robot.intake.intakeNum( 0.6, 1 );*/
//					robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.lineToConstantHeading( new Vector2d( 38, wallPos ) )
				.lineToConstantHeading( new Vector2d( 38, 38 ) )
				.lineToLinearHeading( new Pose2d( wallPos, 38, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( wallPos, -36 ) )
				.lineToConstantHeading( new Vector2d( 38, -36 ) )
				.lineToConstantHeading( new Vector2d( 38, -64.125 ) )
				.waitSeconds( 4 )
				.lineToConstantHeading( new Vector2d( 39, -64.125 ) )
				.build();
	}
}
