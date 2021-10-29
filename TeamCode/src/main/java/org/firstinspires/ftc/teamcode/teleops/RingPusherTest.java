package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.Robot;

@TeleOp(name = "Servo Test", group = "Test")
public class RingPusherTest extends OpMode {

	Servo shooterServo;
	@Override
	public void init( ) {

		shooterServo = hardwareMap.servo.get( " shooterServo " );
		telemetry.addLine( "finished init" );
		telemetry.update( );

	}

	@Override
	public void loop( ) {

		if ( gamepad1.right_trigger > 0 )
			shooterServo.setPosition( shooterServo.getPosition() + 0.0001 );
		else if ( gamepad1.left_trigger > 0)
			shooterServo.setPosition( shooterServo.getPosition() - 0.0001 );


		telemetry.addData( "Servo position", shooterServo.getPosition( ) );
		telemetry.update( );

	}
}