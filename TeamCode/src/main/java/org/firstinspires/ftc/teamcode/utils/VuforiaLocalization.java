package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * A class for tracking Vuforia targets on the field.
 */
public class VuforiaLocalization {

	private final Vuforia vuforia = Vuforia.getInstance( );
	private final VuforiaLocalizer vuforiaLocalizer;


	private VuforiaTrackables targets;

	public List<VuforiaTrackable> trackables;

	private boolean tracking = false;


	/**
	 * Creates a VuforiaTargetTracker
	 *
	 * @param trackablesAssetName name of the tensorflow asset file
	 */
	public VuforiaLocalization( String trackablesAssetName ) {
		if( !vuforia.isRunning( ) ) {
			vuforia.start( );
		}
		vuforiaLocalizer = vuforia.getLocalizer( );

		loadTrackables( trackablesAssetName );

		setCameraPosition( );
	}

	/**
	 * Creates the trackable objects from a tensorflow asset
	 *
	 * @param assetName name of the tensorflow asset file
	 */
	private void loadTrackables( String assetName ) {

		targets = vuforiaLocalizer.loadTrackablesFromAsset( assetName );

		trackables = new ArrayList<VuforiaTrackable>( );

		for( int i = 0; i < targets.size( ); i++ ) {
			trackables.add( targets.get( i ) );
			targets.get( i ).setLocation( FieldMap.VuforiaTargets.TARGET_POSITIONS[i] );
		}
	}

	/**
	 * Tells Vuforia where the camera is located on the robot
	 */
	private void setCameraPosition( ) {
		for( VuforiaTrackable trackable : trackables ) {
			((VuforiaTrackableDefaultListener) trackable.getListener( )).setPhoneInformation( FieldMap.RobotInfo.CAMERA_FROM_ROBOT, VuforiaLocalizer.CameraDirection.BACK );
		}
	}

	/**
	 * Gets the list of trackables
	 *
	 * @return list of trackables
	 */
	public List<VuforiaTrackable> getTrackables( ) {
		return trackables;
	}

	/**
	 * Activates tracking of the targets
	 */
	public void activateTracking( ) {
		targets.activate( );
		tracking = true;
	}

	public boolean isTracking( ) {
		return tracking;
	}

	/**
	 * Deactivates tracking of the targets
	 */
	public void deactivateTracking( ) {
		targets.deactivate( );
		tracking = false;
	}

	/**
	 * Checks if a Vuforia target is currently visible and returns the one that is
	 *
	 * @return the trackable that is visible. If no trackable is visible, returns null
	 */
	public VuforiaTrackable getVisibleTarget( ) {

		for( VuforiaTrackable trackable : trackables )
			if( ((VuforiaTrackableDefaultListener) trackable.getListener( )).isVisible( ) )
				return trackable;

		return null;
	}

	/**
	 * Gets a transformation matrix of the robots location data
	 *
	 * @return robot's current location transform
	 */
	public OpenGLMatrix getRobotLocationTransform( ) {
		VuforiaTrackable currentVisibleTarget = getVisibleTarget( );
		if( currentVisibleTarget == null )
			return null;

		return ((VuforiaTrackableDefaultListener) currentVisibleTarget.getListener( )).getUpdatedRobotLocation( );
	}

	/**
	 * Updates the robot's position in the {@link FieldMap}
	 */
	public void updateRobotLocation( ) {
		OpenGLMatrix robotLocation = getRobotLocationTransform( );
		if( robotLocation != null ) {
			FieldMap.RobotInfo.robotLocation = robotLocation;
		}
	}

	/**
	 * Gets the robot's position, expressed as a {@link VectorF}
	 *
	 * @return robot's current position
	 */
	public VectorF getRobotPosition( ) {
		OpenGLMatrix robotLocation = FieldMap.RobotInfo.robotLocation;
		if( robotLocation == null ) {
			return null;
		}
		return robotLocation.getTranslation( );
	}

	/**
	 * Gets the robot's rotation, expressed as a {@link Orientation}
	 *
	 * @return robot's current orientation
	 */
	public Orientation getRobotRotation( ) {
		OpenGLMatrix robotLocation = FieldMap.RobotInfo.robotLocation;
		if( robotLocation == null ) {
			return null;
		}
		return Orientation.getOrientation( robotLocation, EXTRINSIC, XYZ, DEGREES );
	}
}
