package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem {

	DcMotor intakeMotor1;
	DcMotor intakeMotor2;
	
	public Intake( HardwareMap hw ) {
		setup( hw );
	}

	@Override
	public void setup( HardwareMap hw ) {

		intakeMotor1 = hw.dcMotor.get( "intake1" );
		intakeMotor2 = hw.dcMotor.get( "intake2" );

		intakeMotor1.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	public void setPower( double power ) {

		intakeMotor1.setPower( power );
		intakeMotor2.setPower( power );
	}

	public void eject( ) {
		setPower( -1 );
	}

	public void stop( ) {
		setPower( 0 );
	}

}