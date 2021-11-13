package org.firstinspires.ftc.teamcode.localization;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

//@TeleOp
public class DeleteMaps extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		telemetry.addLine( "Press start to delete all saved maps" );
		telemetry.update();
		waitForStart();
		File mapDirectory = new File(  "/sdcard/FIRST/localization/maps/" );
		for( File map : mapDirectory.listFiles()) {
			map.delete();
		}
		telemetry.addLine( "Maps deleted!" );
		telemetry.update();
	}
}
