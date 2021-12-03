package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;
import org.firstinspires.ftc.teamcode.vision.CubeWeightUtil;

@TeleOp(name = "CubeWeightAutoTest", group = "camera")
public class CubeWeightAutoTest extends OpMode {

	public CubeWeightUtil cubeWeightUtil;

	@Override
	public void init( ) {
		cubeWeightUtil = new CubeWeightUtil( hardwareMap, "webcam", telemetry );
		cubeWeightUtil.init();
	}

	@Override
	public void loop( ) {

	}
}
