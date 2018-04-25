package org.usfirst.frc.team291.auto;

import edu.wpi.first.wpilibj.Timer;

public class GoForwardAuto extends AutoMode{

	Timer timer = new Timer();
	public GoForwardAuto(){
		timer.start();
	}
	@Override
	public void init() {
	}

	@Override
	public void execute() {
		if(timer.get() < 3.0){
			driveBase.setLeftRightPower(-.4, -.4);
		}
		else{
			driveBase.setLeftRightPower(0, 0);
		}
		
	}
	
	@Override
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		return true;
	}


	@Override
	public void outputToSmartDashboard() {
	}


}

