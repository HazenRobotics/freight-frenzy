package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.vision.TensorFlowUtil;

@Autonomous(name="Barcode Test", group = "Test")
public class BarcodeTest extends LinearOpMode {
	TensorFlowUtil tensorFlow;


	@Override
	public void runOpMode( ) throws InterruptedException {
		tensorFlow = new TensorFlowUtil( this );
		tensorFlow.initTensorFlow();

		waitForStart();
		telemetry.addLine( "Running position detection..." );
		telemetry.update();
		tensorFlow.runPositionDetection();
		telemetry.addData( "Detected barcode position", tensorFlow.getBarcodePosition() );
		telemetry.update();
		while( !isStopRequested() );
	}
}
