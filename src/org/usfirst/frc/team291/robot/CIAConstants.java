package org.usfirst.frc.team291.robot;

/*
 * CIAConstants.java
 * Defines constants used all around the robot.
 * @author Julia Cecchetti
 */

public class CIAConstants {
	
	// Motor Port Mappings
	public static final int cubeIntakePort = 8;
	
	public static final int winchMotorPort = 4;
	
	public static final int armMotorA = 7;
	public static final int armMotorB = 6;
	
	public static final int LEDPort = 5;

	public static final int rightDrivePortA = 0;
	public static final int rightDrivePortB = 1;
	public static final int leftDrivePortA = 2;
	public static final int leftDrivePortB = 3;
	
	// Sensor DIO port Mappings
	public static final int leftEncoderPortA = 4;
	public static final int leftEncoderPortB = 5;
	public static final int rightEncoderPortA = 6;
	public static final int rightEncoderPortB = 7;
	
	public static final int tertiaryArmEncoderPortA = 2;
	public static final int tertiaryArmEncoderPortB = 3;
	
	public static final int ultrasonicInPort = 1;
	public static final int ultrasonicOutPort = 0;
	
	// Sensor Analog port Mappings
	public static final int primaryArmEncoderPort = 0;
	public static final int secondaryArmEncoderPort = 1;

	//DIO MXP pins, used to select the autonomous mode
	public static final int AutoPort1 = 19;
	public static final int AutoPort2 = 18;
	public static final int AutoPort3 = 17;
	public static final int AutoPort4 = 16;
	public static final int AutoPort5 = 15;
	public static final int AutoPort6 = 14;
	public static final int LeftSideAutoPort = 10;
	
	// Drive Encoders are configured to read in feet
	public static final double driveEncoderDistancePerPulse = 1;///217.3;
	public static final double tertiaryEncoderAngleConversion = 360.0/4096.0;//converts pulses to degrees
	
	public static final double primaryEncoderZero = 49.39;
	public static final double secondaryEncoderZero = 322.76;
//	public static final double primaryEncoderZero = 98.13;//
//	public static final double secondaryEncoderZero = 84.4;//
	public static final double stowedArmAngle = 15.0;
	
	public static final double primaryUnpluggedVoltage = .24;
	public static final double secondaryUnpluggedVoltage = .41;

	// Solenoid port Mappings
	public static final int shifterPort = 2;//PCM 0
	public static final int firstStageWristPort = 4;//PCM 1
	public static final int secondStageWristPort = 1;//PCM 0
	public static final int cubeClampPort = 7;//PCM 1
	
	/*-------------JOYSTICKS----------------*/
	
	//Operator Joystick Mappings
	/*public static final int highScaleButton = 4;//Y
	public static final int middleScaleButton = 3;//B
	public static final int lowScaleButton = 2;//A
	public static final int switchButton = 1;//X
	public static final int vaultButton = 5;//left bumper
	public static final int intakeButton = 6;//right bumper
	public static final int powerIntakeButton = 8;//right trigger
	public static final int stowButton = 10;//start button
*/	
	public static final int highScaleButton = 4;//Y
	public static final int middleScaleButton = 3;//B
	public static final int middleLowScaleButton = 2;//TODO X
	public static final int lowScaleButton = 1;//A
	public static final int frontScaleButton = 11;//Left joystick press
	public static final int switchButton = 12;//Right joystick press
	public static final int vaultButton = 5;//left bumper
	public static final int intakeButton = 6;//right bumper
	public static final int powerIntakeButton = 8;//right trigger
	public static final int stowButton = 10;//start button
	public static final int armAdjustAxis = 1;

	//Driver Joystick Mappings
	public static final int throttleAxis = 1;//left vertical axis
	public static final int headingAxis = 4;//right horizontal axis
	public static final int cubeEjectAxis = 3;//right trigger
	public static final int powerEjectAxis = 2;//left trigger
	public static final int shiftButton = 9;//right bumper TODO 6
	public static final int antiTipEnableButton = 8;
	public static final int cubeArmOverrideButton = 7;
	public static final int armAdjustUpButton = 5;
	public static final int armAdjustDownButton = 6;
	public static final int winchAxis = 5;
	public static final int enableWinchButton = 4;
	public static final int disableWinchButton = 2;
	public static final int hookLevelButton = 3;
	public static final int hookingButton = 1;
	
	public static final double antiTipThreshold = 6.5;//pitch at which anti tipping code is enabled
	public static final double kpAntiTip = 1.0/10.0;
	
	//Autonomous Constants
	public static final double kp = 3.5;//3.5
	public static final double kd = 0;
	public static final double kv = .08;//.08
	public static final double ka = .06;//.06

	public static final double Pturn = .05;//.055
	public static final double Iturn = 0;
	public static final double Dturn = .1;
	public static final double turnEpsilon = 3.0;
	
/*	public static final double Pdriveturn = .05;//.055
	public static final double Idriveturn = 0;
	public static final double Ddriveturn = .05;*/
	
	public static final double Pdriveturn = .06;//.055
	public static final double Idriveturn = 0;
	public static final double Ddriveturn = 0.0;
	
	public static final double Pdrive = 1.5;
	public static final double Idrive = 0.0;
	public static final double Ddrive = 4.0;
	public static final double driveEbsilon = 2.0;

	public static final double Phold = -.0001;
	
	
}
