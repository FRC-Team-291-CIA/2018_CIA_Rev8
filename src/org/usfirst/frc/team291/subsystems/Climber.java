package org.usfirst.frc.team291.subsystems;

import org.usfirst.frc.team291.robot.CIAConstants;

import edu.wpi.first.wpilibj.Victor;

public class Climber extends Subsystem{
	
	private Victor winchMotor = new Victor(CIAConstants.winchMotorPort);
	
	private boolean winchEnabled = false;
	
	public Climber(){
		
	}

	public void setPower(double power){
		if(winchEnabled) winchMotor.set(power);
		else winchMotor.set(0.0);
	}
	
	public void enableWinch(boolean enable){
		if(enable != winchEnabled){
			if(enable) System.out.println("The winch has been enabled");
			else System.out.println("The winch has been disabled");
		}
		winchEnabled = enable;
	}
	
	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

}
