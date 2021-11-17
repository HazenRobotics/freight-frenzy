package org.firstinspires.ftc.teamcode.vision.unused;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * A class for utilizing a Tensor Flow model.
 */
public class TensorFlow {

	private TFObjectDetector tfod;

	private boolean isActive = false;

	private List<Recognition> recognitions = null;

	private final Vuforia vuforia = Vuforia.getInstance( );

	/**
	 * Creates a TensorFlow
	 *
	 * @param tfodModelAssetName  name of the model asset, found in the assets folder
	 * @param minResultConfidence minimum result confidence to be considered a "Recognition". Value from 0 to 1.
	 * @param monitorCamera       if the camera monitor should run
	 * @param hardwareMap         robot's hardware map
	 * @param labels              labels of the entries in the .tflite file
	 */
	public TensorFlow( String tfodModelAssetName, float minResultConfidence, boolean monitorCamera, HardwareMap hardwareMap, String... labels ) {
		if( !vuforia.isRunning( ) )
			vuforia.start( );

		initTfod( tfodModelAssetName, minResultConfidence, monitorCamera, hardwareMap, labels );
	}

	/**
	 * Initializes Tensor Flow for use
	 *
	 * @param tfodModelAssetName  name of the model asset, found in the assets folder
	 * @param minResultConfidence minimum result confidence to be considered a "Recognition". Value from 0 to 1.
	 * @param monitorCamera       if the camera monitor should run
	 * @param hardwareMap         robot's hardware map
	 * @param labels              labels of the entries in the .tflite file
	 */
	private void initTfod( String tfodModelAssetName, float minResultConfidence, boolean monitorCamera, HardwareMap hardwareMap, String... labels ) {
		TFObjectDetector.Parameters tfodParameters = monitorCamera ? new TFObjectDetector.Parameters( hardwareMap.appContext.getResources( ).getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName( ) ) ) : new TFObjectDetector.Parameters( );
		tfodParameters.minResultConfidence = minResultConfidence;
		tfod = ClassFactory.getInstance( ).createTFObjectDetector( tfodParameters, vuforia.getLocalizer( ) );
		tfod.loadModelFromAsset( tfodModelAssetName, labels );
	}

	/**
	 * Activates object recognition
	 */
	public void activate( ) {
		tfod.activate( );
		isActive = true;
	}

	/**
	 * Stops object recognition
	 */
	public void shutdown( ) {
		tfod.shutdown( );
		isActive = false;
	}

	public void setZoom( double zoom, double aspectRatio ) {
		tfod.setZoom( zoom, aspectRatio );
	}

	/**
	 * Tells if object recognition is currently running
	 *
	 * @return whether Tensor Flow is active or not
	 */
	public boolean isActive( ) {
		return isActive;
	}

	/**
	 * Gets the current recognition
	 *
	 * @return current recognition
	 */
	public Recognition getRecognition( ) {

		updateRecognitions( );
		if( recognitions == null || recognitions.isEmpty( ) )
			return null;

		Log.e( "TFOD_TEST", "checked tfod recognitions " );

		Recognition mostConfidentRecognition = null;
		for( Recognition recognition : recognitions ) {
			if( mostConfidentRecognition == null || recognition.getConfidence( ) > mostConfidentRecognition.getConfidence( ) ) {
				mostConfidentRecognition = recognition;
			}
		}
		return mostConfidentRecognition;

	}

	/**
	 * Updates the current recognition
	 */
	public void updateRecognitions( ) {
		recognitions = tfod.getRecognitions( );
		Log.e( "TFOD_TEST", "got tfod recognitions: " + recognitions );
	}


}
