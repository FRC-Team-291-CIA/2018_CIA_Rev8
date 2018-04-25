package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.NearScalePath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class NearSideOneCubeScaleAuto extends AutoMode{
	
	
	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	
	
	private boolean startOnLeft;
	
	public NearSideOneCubeScaleAuto(boolean startOnLeft){
		System.out.println("Starting NearSideOneCubeScaleAuto");
		timer = new Timer();
		this.startOnLeft = startOnLeft;
		if(startOnLeft){
			trajectoryLeft = NearScalePath.trajectoryArray[0];
			trajectoryRight = NearScalePath.trajectoryArray[1];
		}
		else {
			trajectoryLeft = NearScalePath.trajectoryArray[1];
			trajectoryRight = NearScalePath.trajectoryArray[0];
		}
		controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
		timer.start();
	}

	@Override
	public void init() {
		driveBase.zeroSensors();
		
	}
	
	private State state = State.INIT;
	private State lastState = State.INIT;
	
	private enum State {
		INIT,
		DRIVE_TO_SCALE,
		RAISE_ARM,
		EJECT,
		FINISHED
	}

	@Override
	public void execute() {
		switch(state){
		case INIT:
			init();
			state = State.DRIVE_TO_SCALE;
			timer.reset();
			break;

		case DRIVE_TO_SCALE:
			if(controller.almostDone()){
				cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			}
			else if(timer.get() < 2){
				cubeArm.setWantedArmState(ArmState.IDLE);
			}
			else cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				state = State.RAISE_ARM;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
			}
			break;

		case RAISE_ARM:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			if(timer.get() > 2 && cubeArm.getSystemState() == ArmState.MIDDLE_SCALE_LEVEL){
				state = State.EJECT;
				timer.reset();
			}
			break;
			
			
		case EJECT:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < 1) cubeIntake.ejectCube(.6);
			else{
				state = State.FINISHED;
				System.out.println("Finshed Scale Auto!");
			}
			break;
			
		case FINISHED:
			cubeIntake.ejectCube(0);//stop ejecting the cube
			cubeArm.setWantedArmState(ArmState.STOWED);
			driveBase.setLeftRightPower(0,0);
			break;
		}
		
		if(state != lastState) System.out.println(state);
		lastState = state;
		
	}

	@Override
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		if(startOnLeft == scaleOnLeft) return true;
		else return false;
	}

	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}
	

}
