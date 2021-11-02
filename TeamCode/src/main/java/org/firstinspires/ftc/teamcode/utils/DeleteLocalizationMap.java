package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;

public class DeleteLocalizationMap extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Pressing start will erase the saved localization map");
        telemetry.update();
        waitForStart();
        File map = new File("/localization/maps/map");
        if(map.delete()) {
            telemetry.addLine("Map was deleted");
        } else {
            telemetry.addLine("Map was not deleted. Error.");
        }
        telemetry.update();
        requestOpModeStop();
    }
}
