package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

import java.util.List;

public class BucketDetector {

	private Detector tfDetector;

	private static String MODEL_FILE_NAME;
	private static String LABEL_FILE_NAME;
	private static Classifier.Model MODEl_TYPE;
	private OpMode opMode;

	public BucketDetector( String tfliteFile, String labelFile, OpMode opMode ) {
		this.opMode = opMode;
		tfDetector = null;
		MODEL_FILE_NAME = tfliteFile;
		LABEL_FILE_NAME = labelFile;
		MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
		try {
			tfDetector = new Detector( MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, opMode.hardwareMap, "webcam3", opMode.telemetry );
		} catch( Exception ex ) {
			opMode.telemetry.addData( "Error", String.format( "Unable to initialize Detector. %s", ex.getMessage( ) ) );
		}
	}


	public List<Classifier.Recognition> getResults( ) {
		List<Classifier.Recognition> results = tfDetector.getLastResults( );
		if( results == null || results.size( ) == 0 ) {
			opMode.telemetry.addData( "Info", "No results" );
		} else {
			for( Classifier.Recognition r : results ) {
				String item = String.format( "%s: %.2f", r.getTitle( ), r.getConfidence( ) );
				opMode.telemetry.addData( "Found", item );
			}
		}
		return results;
	}

	public Classifier.Recognition getMostConfidentResult( ) {
		List<Classifier.Recognition> results = getResults( );
		Classifier.Recognition mostConfidentRecognition = null;
		for( Classifier.Recognition r : results ) {
			if( mostConfidentRecognition == null || r.getConfidence( ) > mostConfidentRecognition.getConfidence( ) ) {
				mostConfidentRecognition = r;
			}
		}
		return mostConfidentRecognition;
	}

	public void activate( ) {
		try {
			tfDetector.activate( );
		} catch( Exception e ) {
			opMode.telemetry.addData( "Error starting detector", e );
			Log.d( "HERE", "Error starting detextor" + e );
		}
	}

	public void stopProcessing( ) {
		tfDetector.stopProcessing( );
	}

}
