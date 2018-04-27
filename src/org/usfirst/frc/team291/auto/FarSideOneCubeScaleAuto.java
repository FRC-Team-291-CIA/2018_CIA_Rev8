package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.FarScaleTurnPath;
import org.usfirst.frc.team291.auto.arrays.LeftFarScalePath;
import org.usfirst.frc.team291.auto.arrays.RightFarScalePath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class FarSideOneCubeScaleAuto extends AutoMode{
	
	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	
	
	private boolean startOnLeft;
	
	public FarSideOneCubeScaleAuto(boolean startOnLeft){
		System.out.println("Starting OldFarSideOneCubeScaleAuto");
		timer = new Timer();
		this.startOnLeft = startOnLeft;
		if(startOnLeft){
				//Start on the left, scale on the right
				trajectoryLeft = RightFarScalePath.trajectoryArray[0];
				trajectoryRight = RightFarScalePath.trajectoryArray[1];
		}
		else {
				//Start on the right, scale on the left
				trajectoryLeft = LeftFarScalePath.trajectoryArray[1];
				trajectoryRight = LeftFarScalePath.trajectoryArray[0];
		}
		controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
		timer.start();
		timer.reset();
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
			break;

		case DRIVE_TO_SCALE:
			cubeArm.setWantedArmState(ArmState.IDLE);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
					trajectoryLeft = FarScaleTurnPath.trajectoryArray[0];
					trajectoryRight = FarScaleTurnPath.trajectoryArray[1];
					if(startOnLeft) controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
					else controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
				state = State.RAISE_ARM;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
			}
			break;

		case RAISE_ARM:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
				if(driveBase.gyroIsConnected()){
					if(!startOnLeft) driveBase.turnToAngle(20);
					else driveBase.turnToAngle(-20);
					
					if(driveBase.angleIsStable) state = State.EJECT;
				}
				else{
					if(!controller.onTarget()){// Still driving the path.
						controller.updateTurn();// does the calculations and updates the driveBase
					}
					else{
						driveBase.setLeftRightPower(0, 0);
						state = State.EJECT;
					}
				}
			break;
				
		case EJECT:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < 1) cubeIntake.ejectCube(.85);
			else{
				state = State.FINISHED;
				System.out.println("Finshed One Cube Far Side Scale Auto!");
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
		if(startOnLeft != scaleOnLeft) return true;
		else return false;
	}

	@Override
	public void outputToSmartDashboard() {
	}

}
