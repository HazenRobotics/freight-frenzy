package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class WeightSensorTest extends OpMode {

	AnalogInput weightSensor;

	@Override
	public void init( ) {
		weightSensor = hardwareMap.analogInput.get( "weight" );
	}

	@Override
	public void loop( ) {
		telemetry.addData( "Weight sensor reading", weightSensor.getVoltage() );
	}
}
