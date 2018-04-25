package org.usfirst.frc.team291.util;

import org.usfirst.frc.team291.robot.CIAConstants;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnalogEncoder {
	
	private AnalogInput encoder;
	private Timer timer = new Timer();
	private double workingZero = 0;
	private boolean inverted = false;
	private double unpluggedVoltage = .24;
	
	public AnalogEncoder(int analogPort){
		timer.start();
		encoder = new AnalogInput(analogPort);
	}
	
	public void setWorkingZero(double workingZero){
		this.workingZero = workingZero;
	}
	
	public void setUnpluggedVoltage(double unpluggedVoltage){
		this.unpluggedVoltage = unpluggedVoltage;
	}
	public void setInverted(boolean inverted){
		this.inverted = inverted;
	}
	
	public double getAngle(){
		double angle = encoder.getVoltage()*72 - workingZero + CIAConstants.stowedArmAngle;
		if(angle > 346) angle -= 346;
		if(inverted) return -angle;
		else return angle;
	}
	
	public double getRawAngle(){
		return encoder.getVoltage()*72;
	}
	
	
	public boolean isConnected(){
		if(Math.abs(encoder.getVoltage() - unpluggedVoltage) < .1) return false;//encoder is bad
		else return true;
	}
	

}
