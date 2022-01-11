package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;

import java.util.List;

/**
 * A class for utilizing a Tensor Flow model.
 */
public class TensorFlow {

	private TFObjectDetector tfod;

	private boolean isActive = false;

	private List<Recognition> recognitions = null;

	public VuforiaLocalizer vuforia;

	HardwareMap hardwareMap;

	/**
	 * Creates a TensorFlow
	 *
	 * @param tfodModelAssetName  name of the model asset, found in the assets folder
	 * @param minResultConfidence minimum result confidence to be considered a "Recognition". Value from 0 to 1.
	 * @param hardwareMap         robot's hardware map
	 * @param labels              labels of the entries in the .tflite file
	 */
	public TensorFlow( HardwareMap hardwareMap, String cameraName, String tfodModelAssetName, float minResultConfidence, String... labels ) {

		this.hardwareMap = hardwareMap;
		initVuforia( cameraName );
		initTfod( tfodModelAssetName, minResultConfidence, labels );
	}

	/**
	 * Initialize the Vuforia localization engine.
	 */
	private void initVuforia( String cameraName ) {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters( );

		parameters.vuforiaLicenseKey = hardwareMap.appContext.getResources( ).getString( R.string.vuforia_key );
		parameters.cameraName = hardwareMap.get( WebcamName.class, cameraName );

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance( ).createVuforia( parameters );

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	/**
	 * Initialize the TensorFlow Object Detection engine.
	 */
	private void initTfod( String tfodModelAssetName, float minResultConfidence, String... labels ) {
		int tfodMonitorViewId = hardwareMap.appContext.getResources( ).getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName( ) );
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters( tfodMonitorViewId );
		tfodParameters.minResultConfidence = minResultConfidence;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance( ).createTFObjectDetector( tfodParameters, vuforia );
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
	 * @return the most confident current recognition
	 */
	public Recognition getMostConfRecognition( ) {

		updateRecognitions( );
		if( recognitions == null || recognitions.isEmpty( ) )
			return null;

//		Log.d( "TFOD_TEST", "checked tfod recognitions " );

		Recognition mostConfidentRecognition = null;
		for( Recognition recognition : recognitions ) {
			if( mostConfidentRecognition == null || recognition.getConfidence( ) > mostConfidentRecognition.getConfidence( ) )
				mostConfidentRecognition = recognition;

//			Log.d( "TFOD_TEST", "current recognition: " + recognition.getLabel( ) );
		}

		return mostConfidentRecognition;
	}

	/**
	 * Gets the current recognitions
	 *
	 * @return the current non-null recognitions
	 */
	public List<Recognition> getRecognitions( ) {

		updateRecognitions( );
		if( recognitions == null || recognitions.isEmpty( ) )
			return null;

		Log.d( "TFOD_TEST", "checked tfod recognitions " );

		for( int i = 0; i < recognitions.size( ); i++ ) {
			if( recognitions.get( i ) == null )
				recognitions.remove( i-- );

			Log.d( "TFOD_TEST", "current recognition: " + recognitions.get( i ).getLabel( ) );
		}

		return recognitions;
	}

	/**
	 * Updates the current recognition
	 */
	public void updateRecognitions( ) {
		recognitions = tfod.getRecognitions( );
		Log.d( "TFOD_TEST", "got tfod recognitions: " + recognitions );
	}

}
