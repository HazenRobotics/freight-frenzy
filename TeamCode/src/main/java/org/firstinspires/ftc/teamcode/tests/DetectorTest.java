/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

import java.util.List;

import javax.annotation.Nullable;

//Opmode for quick testing of motors
@TeleOp(name="TFDetector", group="Robot15173")

public class DetectorTest extends LinearOpMode{

    // Declare OpMode members.
    private Detector tfDetector;
    private ElapsedTime runtime;

    private static String MODEL_FILE_NAME;
    private static String LABEL_FILE_NAME;
    private static Classifier.Model MODEl_TYPE;


    public void initializeClassificationDetector( String tfliteFile, String labelFile, HardwareMap hm ) {
        tfDetector = null;
        runtime = new ElapsedTime();
        MODEL_FILE_NAME = tfliteFile;
        LABEL_FILE_NAME = labelFile;
        MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
        try {
            tfDetector = new Detector(MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hm.appContext, telemetry);
            tfDetector.activate();
        }
        catch (Exception ex){
            telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
            sleep(3000);
            return;
        }
    }

    @Nullable
    public List<Classifier.Recognition> getResults() {
        List<Classifier.Recognition> results = tfDetector.getLastResults();
        if (results == null || results.size() == 0){
            telemetry.addData("Info", "No results");
        }
        else {
            for (Classifier.Recognition r : results) {
                String item = String.format("%s: %.2f", r.getTitle(), r.getConfidence());
                telemetry.addData("Found", item);
            }
        }
        return results;
    }

    @Override
    public void runOpMode() {
        try {
            initializeClassificationDetector( "loaded_vs_unloaded.tflite", "loaded_vs_unloaded.tflite", hardwareMap );

            telemetry.addData("Detector", "Ready");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0){
                    telemetry.addData("Info", "No results");
                }
                else {
                    for (Classifier.Recognition r : results) {
                        String item = String.format("%s: %.2f", r.getTitle(), r.getConfidence());
                        telemetry.addData("Found", item);
                    }
                }
                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (tfDetector != null){
                tfDetector.stopProcessing();
            }
        }
    }
}
