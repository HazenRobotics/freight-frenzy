package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;

@TeleOp(name = "SetPositionTest", group = "Test")
public class SetPositionTest extends OpMode {

	RRMecanumDriveTippy42 drive;

	@Override
	public void init( ) {

		drive = new RRMecanumDriveTippy42( hardwareMap );
	}

	@Override
	public void loop( ) {
		drive.setWeightedDrivePower(
				new Pose2d(
						-gamepad1.left_stick_y,
						-gamepad1.left_stick_x,
						-gamepad1.right_stick_x
				)
		);

		if( gamepad1.a )
			drive.setPoseEstimate( new Pose2d( -20, -20, Math.toRadians( 270 ) ) );

		drive.update( );
	}
}
