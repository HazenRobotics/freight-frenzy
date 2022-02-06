package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

public class ObjectBoxDetector extends ObjectDetector{

	public ObjectBoxDetector( String tfliteFile, String labelFile, String cameraName, OpMode opMode ) {
		super( tfliteFile, labelFile, Classifier.Model.FLOAT_MOBILENET, cameraName, opMode );
	}
}
