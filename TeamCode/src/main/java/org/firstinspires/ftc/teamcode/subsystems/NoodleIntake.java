package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NoodleIntake {

	DcMotor intakeMotor;

	public NoodleIntake( HardwareMap hw ) {
		setup( hw );
	}

	public void setup( HardwareMap hw ) {

		intakeMotor = hw.dcMotor.get( "intake" );

		intakeMotor.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	public void setPower( double power ) {

		intakeMotor.setPower( power );
	}

	public void eject( ) {
		setPower( -1 );
	}

	public void stop( ) {
		setPower( 0 );
	}

	public double getPower( ) {
		return intakeMotor.getPower( );
	}
}