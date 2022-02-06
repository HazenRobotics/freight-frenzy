package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

public class ObjectClassifier extends ObjectDetector{

	public ObjectClassifier( String tfliteFile, String labelFile, String cameraName, OpMode opMode ) {
		super( tfliteFile, labelFile, Classifier.Model.FLOAT_EFFICIENTNET , cameraName, opMode );
	}
}
