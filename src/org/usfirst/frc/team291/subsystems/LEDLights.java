package org.usfirst.frc.team291.subsystems;

import org.usfirst.frc.team291.robot.CIAConstants;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;

public class LEDLights extends Subsystem{
	
	private Spark lights = new Spark(CIAConstants.LEDPort);
	
	public boolean hasCube = false;
	public boolean encoderMalfunction = false;
	public boolean antiTip = false;
	private Timer lightTimer = new Timer();
	
	private double cubePattern = .75;//Aqua
	private double antiTipPattern = .35;//strobe
	private double nominalPattern = .99;
	
	public LEDLights(){
		lightTimer.start();
	}
	
	public void updateLights(){
		if(antiTip) lights.set(antiTipPattern);
		else if(hasCube) flashLights(true);
		else{
			flashLights(false);
			lights.set(nominalPattern);
		}
	}
	
	private boolean lastFlash = false;
	
	private void flashLights(boolean flash){
		if(flash && !lastFlash){
			lightTimer.reset();
			lightTimer.start();
		}
		lastFlash = flash;
		double t = lightTimer.get();
		
		if(t < .125) lights.set(cubePattern);
		else if(t < .25) lights.set(nominalPattern);
		else if(t < .375) lights.set(cubePattern);
		else if(t < .5) lights.set(nominalPattern);
		else if(t < .625) lights.set(cubePattern);
		else if(t < .75) lights.set(nominalPattern);
		else if(t < .875) lights.set(cubePattern); 
		else lights.set(cubePattern);
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
