package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class CreateLocalizationMap extends OpMode {

	RRMecanumDriveHex42 drive;
	final String MAP_NAME = "hazen.bin";
	GamepadEvents gamepad;


	@Override
	public void init( ) {
		drive = new RRMecanumDriveHex42( hardwareMap, false );
		gamepad = new GamepadEvents( gamepad1 );
	}

	@Override
	public void init_loop() {
		drive.update();
		if(drive.getPoseConfidence().compareTo( T265Camera.PoseConfidence.High ) < 0) {
			telemetry.addLine( "Getting location estimate. Please wait..." );
		}
		else {
			telemetry.addLine( "Ready for tracking!" );
		}
		telemetry.update();
	}

	@Override
	public void loop( ) {
		telemetry.addLine( "Press X to set position to 0. Press B to save the map." );
		telemetry.addData( "Pose Confidence", drive.getPoseConfidence() );
		drive.setWeightedDrivePower( new Pose2d( -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x ) );
		drive.update();
		if(gamepad.x.onPress()) {
			drive.setPoseEstimate( new Pose2d( 0, 0, 0 ) );
		}
		if(gamepad.b.onPress()) {
			new Thread( () -> {
				drive.exportLocalizationMap(MAP_NAME);
				telemetry.addLine( "Map was saved!" );
				telemetry.update();
				requestOpModeStop();
			} ).start();
		}
		gamepad.update();
	}

	@Override
	public void stop() {
		//drive.exportLocalizationMap(MAP_NAME);
	}
}
