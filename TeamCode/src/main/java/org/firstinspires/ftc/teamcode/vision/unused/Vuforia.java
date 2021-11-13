package org.firstinspires.ftc.teamcode.vision.unused;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.utils.VuforiaLocalizerPlus;
import org.jetbrains.annotations.NotNull;

/**
 * A class that holds an instance of the Vuforia engine.
 */
public class Vuforia {

	private static final Vuforia vuforia = new Vuforia( );

	private static VuforiaLocalizerPlus vuforiaLocalizer = null;

	private static VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters( );


	private Vuforia( ) {
	}

	/**
	 * Sets the vuforia engine parameters
	 *
	 * @param vuforiaKey    vuforia license key
	 * @param webcamName    name of the webcam in the robot configuration
	 * @param monitorCamera if the camera monitor should run
	 * @param hardwareMap   robot's hardware map
	 */
	public void setParameters( String vuforiaKey, String webcamName, boolean monitorCamera, @NotNull HardwareMap hardwareMap ) {
		parameters = new VuforiaLocalizer.Parameters( );
		parameters.cameraMonitorViewIdParent = monitorCamera ? hardwareMap.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName( ) ) : null;
		parameters.vuforiaLicenseKey = vuforiaKey;
		parameters.cameraName = hardwareMap.get( WebcamName.class, webcamName );
		parameters.useExtendedTracking = false;
	}

	/**
	 * Whether or not the Vuforia engine is running
	 *
	 * @return if vuforia engine is running or not
	 */
	public boolean isRunning( ) {
		return vuforiaLocalizer != null;
	}

	/**
	 * Stops and closes the current running instance of the Vuforia engine
	 */
	public void close( ) {
		vuforiaLocalizer.close( );
		vuforiaLocalizer = null;
	}

	/**
	 * The instance of the Vuforia class
	 *
	 * @return Vuforia instance
	 */
	public static Vuforia getInstance( ) {
		return vuforia;
	}

	/**
	 * Starts the Vuforia engine
	 */
	public void start( ) {
		if( vuforiaLocalizer == null ) {
			vuforiaLocalizer = new VuforiaLocalizerPlus( parameters );
		} else {
			vuforiaLocalizer.resume( );
		}
	}

	/**
	 * Pauses the current running instance of the Vuforia engine
	 */
	public void pause( ) {
		vuforiaLocalizer.pause( );
	}

	public void resume( ) {
		vuforiaLocalizer.resume( );
	}

	public VuforiaLocalizer getLocalizer( ) {
		return vuforiaLocalizer;
	}
}
