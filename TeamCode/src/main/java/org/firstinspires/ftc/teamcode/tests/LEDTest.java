package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RGBLights;

@Autonomous
public class LEDTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		RGBLights lights = new RGBLights( hardwareMap, "blinkin" );
		waitForStart();
		while( opModeIsActive() ) {
			lights.showStatus( RGBLights.StatusLights.ERROR );
			long startTime = System.currentTimeMillis();
			while( opModeIsActive() && System.currentTimeMillis() < startTime + 3000 );
			lights.showStatus( RGBLights.StatusLights.ERROR );
			startTime = System.currentTimeMillis();
			while( opModeIsActive() && System.currentTimeMillis() < startTime + 3000 );
			lights.showStatus( RGBLights.StatusLights.SUCCESS );
			startTime = System.currentTimeMillis();
			while( opModeIsActive() && System.currentTimeMillis() < startTime + 3000 );
			lights.showStatus( RGBLights.StatusLights.WAITING );
			startTime = System.currentTimeMillis();
			while( opModeIsActive() && System.currentTimeMillis() < startTime + 3000 );
			lights.showStatus( RGBLights.StatusLights.NORMAL );
			startTime = System.currentTimeMillis();
			while( opModeIsActive() && System.currentTimeMillis() < startTime + 3000 );
			lights.showStatus( RGBLights.StatusLights.CELEBRATION );
			startTime = System.currentTimeMillis();
			while( opModeIsActive() && System.currentTimeMillis() < startTime + 3000 );
		}


	}
}
