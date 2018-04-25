package org.usfirst.frc.team291.auto;

public class DoNothingAuto extends AutoMode{

	@Override
	public void init() {
		System.out.println("Starting DoNothingAuto");
	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		return true;
	}

	@Override
	public void outputToSmartDashboard() {
	}

}
