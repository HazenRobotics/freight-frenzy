package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
@Disabled
public class ServoTester extends OpMode {

	Servo servo;

	@Override
	public void init( ) {

		servo = hardwareMap.servo.get( "servo" );
		telemetry.addLine( "finished init" );
		telemetry.update( );

	}

	@Override
	public void loop( ) {

		if( gamepad1.right_trigger > 0 )
			servo.setPosition( servo.getPosition( ) + 0.0001 );
		else if( gamepad1.left_trigger > 0 )
			servo.setPosition( servo.getPosition( ) - 0.0001 );

		telemetry.addLine( "Servo position: " + servo.getPosition( ) );
		telemetry.update( );

	}
}