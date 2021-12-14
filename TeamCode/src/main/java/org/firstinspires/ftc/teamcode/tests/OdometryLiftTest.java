package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OdometryLift;

@TeleOp(name = "OdometryLiftTest", group = "Test")
@Disabled
public class OdometryLiftTest extends LinearOpMode {
	OdometryLift odometryLift;

	@Override
	public void runOpMode( ) throws InterruptedException {
		odometryLift = new OdometryLift( hardwareMap );

		waitForStart();
		odometryLift.raise();
		sleep( 5000 );
		odometryLift.lower();
		sleep( 3000 );
	}
}
