package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

import java.util.List;

import javax.annotation.Nullable;

public class ObjectDetector {

	private Detector tfDetector;

	private OpMode opMode;

	private boolean isActive = false;

	public ObjectDetector(String tfliteFile, String labelFile, Classifier.Model modelType, String cameraName, OpMode opMode) {
		this.opMode = opMode;

		try {
			tfDetector = new Detector( modelType, tfliteFile, labelFile, opMode.hardwareMap, cameraName, opMode.telemetry );
		} catch( Exception ex ) {
			opMode.telemetry.addData( "Error", String.format( "Unable to initialize Detector. %s", ex.getMessage( ) ) );
		}
	}

	@Nullable
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

	@Nullable
	public Classifier.Recognition getMostConfidentResult( ) {
		List<Classifier.Recognition> results = getResults();
		if(results == null) {
			return null;
		}
		Classifier.Recognition mostConfidentRecognition = null;
		for( Classifier.Recognition r : results ) {
			if (mostConfidentRecognition == null || r.getConfidence() > mostConfidentRecognition.getConfidence() ) {
				mostConfidentRecognition = r;
			}
		}
		return mostConfidentRecognition;
	}

	public void activate() {
		try{
			tfDetector.activate( );
			isActive = true;
		} catch( Exception e ) {
			opMode.telemetry.addData( "Error starting detector", e );
		}
	}

	public void stopProcessing() {
		tfDetector.stopProcessing();
		isActive = false;
	}

	public boolean isActive() {
		return isActive;
	}

}
