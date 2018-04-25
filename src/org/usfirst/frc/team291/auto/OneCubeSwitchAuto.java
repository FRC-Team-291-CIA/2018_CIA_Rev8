package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.LeftSwitchPath;
import org.usfirst.frc.team291.auto.arrays.RightSwitchPath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class OneCubeSwitchAuto extends AutoMode{
	
	
	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	
	public OneCubeSwitchAuto(boolean switchOnLeft){
		System.out.println("Starting OneCubeSwitchAuto");
		timer = new Timer();
		if(switchOnLeft){
			trajectoryLeft = LeftSwitchPath.trajectoryArray[0];
			trajectoryRight = LeftSwitchPath.trajectoryArray[1];
		}
		else {
			trajectoryLeft = RightSwitchPath.trajectoryArray[0];
			trajectoryRight = RightSwitchPath.trajectoryArray[1];
		}
		controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
		timer.start();
	}

	@Override
	public void init() {
		driveBase.zeroSensors();
		
	}
	
	private State state = State.INIT;
	
	private enum State {
		INIT,
		DRIVE_TO_SWITCH,
		EJECT,
		FINISHED
	}

	@Override
	public void execute() {
		switch(state){
		case INIT:
			init();
			state = State.DRIVE_TO_SWITCH;
			break;

		case DRIVE_TO_SWITCH:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				state = State.EJECT;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
			}
			if(timer.get() > 2) cubeIntake.ejectCube(1);
			break;

		case EJECT:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(timer.get() < 1) cubeIntake.ejectCube(.75);
			else{
				state = State.FINISHED;
				System.out.println("Finshed Switch Auto!");
			}
			break;
			
		case FINISHED:
			cubeIntake.ejectCube(0);//stop ejecting the cube
			cubeArm.setWantedArmState(ArmState.STOWED);
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
