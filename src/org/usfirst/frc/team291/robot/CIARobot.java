/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team291.robot;

import org.usfirst.frc.team291.auto.AutoMode;
import org.usfirst.frc.team291.auto.TestSequence;
import org.usfirst.frc.team291.subsystems.CubeArm;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class CIARobot extends TimedRobot {

	//UsbCamera camera;
	//Joysticks
	public static final Joystick driver = new Joystick(0);
	public static final Joystick operator = new Joystick(1);
	
	private ArmState wantedArmState;

	@Override
	public void robotInit() {
		//camera = CameraServer.getInstance().startAutomaticCapture();
		//camera.setResolution(320, 240);
		wantedArmState = CubeArm.ArmState.IDLE;
	}
	
	@Override
	public void robotPeriodic(){
		CIAObjects.cubeArm.outputToSmartDashboard();
		CIAObjects.cubeIntake.outputToSmartDashboard();
		CIAObjects.driveBase.outputToSmartDashboard();
		CIAObjects.lights.updateLights();
		CIAObjects.cubeIntake.hasCube();
	}

	private AutoMode autoMode;
	@Override
	public void autonomousInit() {
		autoMode = CIAObjects.autoSelector.selectAuto();
	}

	@Override
	public void autonomousPeriodic() {
		//System.out.println("running");
		autoMode.execute();
	}
	
	@Override
	public void teleopInit(){
		wantedArmState = CIAObjects.cubeArm.getSystemState();
	}
	
	@Override
	public void teleopPeriodic() {
		
		if(operator.getRawButton(CIAConstants.highScaleButton)) wantedArmState = ArmState.HIGH_SCALE_LEVEL;
		else if(operator.getRawButton(CIAConstants.middleScaleButton)) wantedArmState = ArmState.MIDDLE_SCALE_LEVEL;
		else if(operator.getRawButton(CIAConstants.middleLowScaleButton)) wantedArmState = ArmState.MIDDLE_LOW_SCALE_LEVEL;
		else if(operator.getRawButton(CIAConstants.lowScaleButton)) wantedArmState = ArmState.LOW_SCALE_LEVEL;
		else if(operator.getRawButton(CIAConstants.switchButton)) wantedArmState = ArmState.SWITCH_LEVEL;
		else if(operator.getRawButton(CIAConstants.frontScaleButton)) wantedArmState = ArmState.FAST_SCALE_LEVEL;
		else if(driver.getRawButton(CIAConstants.hookLevelButton)) wantedArmState = ArmState.HOOK_PREP;
		else if(driver.getRawButton(CIAConstants.hookLevelButton) && CIAObjects.cubeArm.getSystemState() == ArmState.HOOK_PREP) wantedArmState = ArmState.HOOK_LEVEL;
		else if(driver.getRawButton(CIAConstants.hookingButton)) wantedArmState = ArmState.HOOKING;
		else if(operator.getRawButton(CIAConstants.intakeButton) || operator.getRawButton(CIAConstants.powerIntakeButton)
				|| operator.getRawButton(CIAConstants.vaultButton)) wantedArmState = ArmState.STOWED; 
		else if(operator.getRawButton(CIAConstants.stowButton)) wantedArmState = ArmState.IDLE;
		if(wantedArmState == ArmState.STOWED || wantedArmState == ArmState.ACQUIRING || wantedArmState == ArmState.VAULT_LEVEL){
			if(operator.getRawButton(CIAConstants.intakeButton) || operator.getRawButton(CIAConstants.powerIntakeButton)) wantedArmState = ArmState.ACQUIRING;
			else if(operator.getRawButton(CIAConstants.vaultButton)) wantedArmState = ArmState.VAULT_LEVEL;
			else wantedArmState = ArmState.STOWED;
		}
		
		CIAObjects.cubeIntake.powerIntake(operator.getRawButton(CIAConstants.powerIntakeButton));
		CIAObjects.climber.setPower(driver.getRawAxis(CIAConstants.winchAxis));
		if(driver.getRawButton(CIAConstants.enableWinchButton)) CIAObjects.climber.enableWinch(true);
		if(driver.getRawButton(CIAConstants.disableWinchButton)) CIAObjects.climber.enableWinch(false);
		
		double adjustment;
		if(driver.getRawButton(CIAConstants.armAdjustUpButton)) adjustment = .8;
		else if(driver.getRawButton(CIAConstants.armAdjustDownButton)) adjustment = -.8;
		else adjustment = operator.getRawAxis(CIAConstants.armAdjustAxis);
		CIAObjects.cubeArm.adjustArmAngle(adjustment);
		
		CIAObjects.cubeArm.toggleVaultState(operator.getRawButton(CIAConstants.vaultButton));
		
		CIAObjects.cubeIntake.adjustCubeOut(operator.getPOV(0) == 0);//POV is pressed Up
		CIAObjects.cubeIntake.adjustCubeIn(operator.getPOV(0) == 180);//POV is pressed down
		
		CIAObjects.cubeArm.setWantedArmState(wantedArmState);
		
		boolean slowMode = false;
		if(CIAObjects.cubeArm.armIsHigh()) slowMode = true;
		
		CIAObjects.driveBase.CIADrive(driver.getRawAxis(CIAConstants.throttleAxis), driver.getRawAxis(CIAConstants.headingAxis), slowMode);
		CIAObjects.driveBase.shift(driver.getRawButton(CIAConstants.shiftButton));
		CIAObjects.cubeArm.override(driver.getRawButton(CIAConstants.cubeArmOverrideButton));
		
		CIAObjects.driveBase.enableAntiTip(driver.getRawButton(CIAConstants.antiTipEnableButton));
		
		if(driver.getRawAxis(CIAConstants.cubeEjectAxis) > .02) CIAObjects.cubeIntake.ejectCube(.5*driver.getRawAxis(CIAConstants.cubeEjectAxis));
		else CIAObjects.cubeIntake.ejectCube(driver.getRawAxis(CIAConstants.powerEjectAxis));

	}
	
	private AutoMode testMode;
	
	@Override
	public void testInit(){
		testMode = new TestSequence();
	}
	@Override
	public void testPeriodic() {
		testMode.execute();
	}
	
	@Override
	public void disabledInit(){};
	
	@Override
	public void disabledPeriodic(){};
}
