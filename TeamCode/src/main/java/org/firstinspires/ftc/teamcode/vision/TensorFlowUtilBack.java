package org.firstinspires.ftc.teamcode.vision;

/**
 * Author: Sam DePoule
 * <p>
 * Made using keyboard shortcuts ONLY
 */

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class TensorFlowUtilBack {

	private final String TENSOR_FLOW_MODEL_NAME = "full_model_with_metadata.tflite";

	// b = ball, d = duck, e = element, h = heavy, l = light, m = medium
	public static final String[] LABELS = new String[]{ "b", "d", "e", "h", "l", "m" };


	TensorFlow tensorFlow;
	OpMode opMode;
	HardwareMap hardwareMap;

	public TensorFlowUtilBack( OpMode op ) {
		opMode = op;
		hardwareMap = opMode.hardwareMap;
	}

	public void initTensorFlow( ) {

		tensorFlow = new TensorFlow( hardwareMap, "webcam2", TENSOR_FLOW_MODEL_NAME, 0.6f, LABELS );
	}

	public void startTF( ) {
		tensorFlow.activate( );
	}

	public boolean isActive( ) {
		return tensorFlow.isActive( );
	}

	/**
	 *
	 * @return the most confident recognition (null if there aren't any)
	 */
	public Recognition identifyObject( ) {

		Recognition recognition = tensorFlow.getMostConfRecognition( );

		Log.d( "TFOD_TEST", "got recognitions: " + recognition );
		if( recognition != null ) {
			String label = recognition.getLabel( );
			switch( label ) {
				// "b", "d", "e", "h", "l", "m"
				case "b": // ball
					label = "Ball";
					break;
				case "d": // duck
					label = "Duck";
					break;
				case "e": // element
					label = "Element";
					break;
				case "h": // heavy
					label = "Heavy";
					break;
				case "m": // medium
					label = "Medium";
					break;
				case "l": // light
					label = "Light";
					break;
			}

			if( label.length() > 1 )
				return recognition;
		}

		Log.d( "TFOD_TEST", "didn't find a tfod recognition " );

		return null;
	}

	public List<Recognition> identifyObjects( ) {

		List<Recognition> recognitions = tensorFlow.getRecognitions( );

		Log.d( "TFOD_TEST", "got recognitions: " + recognitions );
		if( recognitions != null && recognitions.size( ) > 0 ) {
			for( int i = 0; i < recognitions.size( ); i++ ) {

				Recognition currentRec = recognitions.get( i );
				if( currentRec != null ) {
					String label = currentRec.getLabel( );
					switch( label ) {
						// "b", "d", "e", "h", "l", "m"
						case "b": // ball
							label = "Ball";
							break;
						case "d": // duck
							label = "Duck";
							break;
						case "e": // element
							label = "Element";
							break;
						case "h": // heavy
							label = "Heavy";
							break;
						case "m": // medium
							label = "Medium";
							break;
						case "l": // light
							label = "Light";
							break;
					}

					if( label.length( ) < 1 ) {
						Log.d( "TFOD_TEST", "tfod recognition name invalid: " + label );
						recognitions.remove( i-- );
					}

				}
			}
			return recognitions;
		}


		Log.d( "TFOD_TEST", "didn't find any tfod recognitions" );

		return null;
	}

	public void stopTF( ) {
		tensorFlow.shutdown( );
	}

	public void setZoom( double zoom ) {
		tensorFlow.setZoom( zoom, 16.0 / 9.0 );
	}

	public void setZoom( double zoom, double aspectRatio ) {
		tensorFlow.setZoom( zoom, aspectRatio );
	}

	public void logAndPrint( String text, boolean includeTimeStamp ) {

//        Robot.writeToMatchFile(text, includeTimeStamp);
		opMode.telemetry.addLine( text );
		opMode.telemetry.update( );
	}

}
