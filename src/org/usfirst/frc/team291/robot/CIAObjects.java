package org.usfirst.frc.team291.robot;

import org.usfirst.frc.team291.auto.AutoSelector;
import org.usfirst.frc.team291.subsystems.Climber;
import org.usfirst.frc.team291.subsystems.CubeArm;
import org.usfirst.frc.team291.subsystems.CubeIntake;
import org.usfirst.frc.team291.subsystems.DriveBase;
import org.usfirst.frc.team291.subsystems.LEDLights;

public class CIAObjects {
	
	public static final DriveBase driveBase = new DriveBase();
	public static final CubeIntake cubeIntake = new CubeIntake();
	public static final CubeArm cubeArm = new CubeArm();
	public static final LEDLights lights = new LEDLights();
	public static final Climber climber = new Climber();
		
	public static final AutoSelector autoSelector = new AutoSelector();
	
	public static void initRobot() {}

}
