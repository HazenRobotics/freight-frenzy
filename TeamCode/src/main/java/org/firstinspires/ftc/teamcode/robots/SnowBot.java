package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.FourWheelDrive;

public class SnowBot {
	public FourWheelDrive drive;

	public SnowBot( HardwareMap hw ) {
		drive = new FourWheelDrive( hw );
	}

}
