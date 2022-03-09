package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ReleaseCapper;

@TeleOp
public class ReleaseCapperTest extends OpMode {

	ReleaseCapper capper;

	@Override
	public void init( ) {
		capper = new ReleaseCapper( "capper", hardwareMap );
	}

	@Override
	public void loop( ) {
		if(gamepad1.y) {
			capper.setPosition( ReleaseCapper.Position.STORE );
		}
		if(gamepad1.a) {
			capper.setPosition( ReleaseCapper.Position.PICKUP );
		}
		if(gamepad1.x) {
			capper.setPosition( ReleaseCapper.Position.RELEASE );
		}
		telemetry.addData( "Capper Position", capper.getPosition() );
	}
}
