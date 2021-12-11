package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OdometryLift;

@TeleOp(name = "OdometryLiftTest", group = "Test")
public class OdometryLiftTest extends LinearOpMode {
	OdometryLift odometryLift;

	@Override
	public void runOpMode( ) throws InterruptedException {
		odometryLift = new OdometryLift( hardwareMap );

		waitForStart();
		odometryLift.liftOdometry();
		sleep( 5000 );
		odometryLift.lowerOdometry();
		sleep( 3000 );
	}
}
