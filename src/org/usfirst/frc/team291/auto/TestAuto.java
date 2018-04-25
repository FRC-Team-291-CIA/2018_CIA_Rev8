package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.StraightTestPath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;

public class TestAuto extends AutoMode {
	
	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	
	public TestAuto(){
		System.out.println("Starting TestAuto");
		// Get the left and right trajectories from auto.arrays
		// if the robot is on the blue side, run normally
		if(true){
			trajectoryLeft = StraightTestPath.trajectoryArray[0];
			trajectoryRight = StraightTestPath.trajectoryArray[1];
		} 
		//if on the red side, invert the path
		else {
			trajectoryLeft = StraightTestPath.trajectoryArray[1];
			trajectoryRight = StraightTestPath.trajectoryArray[0];
			trajectoryLeft.setInvertedY(true);
			trajectoryRight.setInvertedY(true);
		}
		// Create a new TDC controller using the trajectories from 
		controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
	}
	
	@Override
	public void init() {
		//timer.start();
		driveBase.zeroSensors();
		
	}
	
	private double angle = 90.0;
	
	/* execute() should be called in autonomousPeriodic */
	@Override
	public void execute() {
		//controller.update();// does the calculations and updates the driveBase
		if(!driveBase.angleIsStable){
			driveBase.turnToAngle(angle);
		}
		else{
			angle+=90.0;
			driveBase.setLeftRightPower(0, 0);
			driveBase.resetPID();
		}
		//driveBase.driveToDistancePID(-6.0, .7, 0.0);
	}

	@Override
	public void outputToSmartDashboard() {
		
		
	}

	@Override
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		// TODO Auto-generated method stub
		return true;
	}

}
