package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.BarcodeDetection;
import org.firstinspires.ftc.teamcode.vision.TensorFlowUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.tensorflow.lite.task.vision.detector.Detection;

@Autonomous(name = "New Barcode Test", group = "Test")
public class NewBarcodeTest extends LinearOpMode {


	@Override
	public void runOpMode( ) throws InterruptedException {

		BarcodeDetection detector = new BarcodeDetection( hardwareMap, "webcam", telemetry );
		detector.init();
		telemetry.addLine( "" + detector.getAnalysis() );

		telemetry.addLine( "TensorFlow init finished" );
		telemetry.update( );

		waitForStart( );

		Log.e( "TFOD_TEST", "started opmode: " );



	}
}
