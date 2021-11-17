package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp
public class CameraTest extends OpMode {
	T265Camera slamra;


	@Override
	public void init( ) {
		slamra = new T265Camera( new Transform2d(  ), 0, hardwareMap.appContext );
		slamra.start();
	}

	@Override
	public void loop( ) {
		telemetry.addData( "Camera position", slamra.getLastReceivedCameraUpdate().pose );
		telemetry.addData( "Camera velocity", slamra.getLastReceivedCameraUpdate().velocity );
		telemetry.update();
	}
}
