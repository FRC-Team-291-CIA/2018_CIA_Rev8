package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.CubeToSwitchPath;
import org.usfirst.frc.team291.auto.arrays.ForwardCubePickupPath;
import org.usfirst.frc.team291.auto.arrays.LeftSwitchPath;
import org.usfirst.frc.team291.auto.arrays.RightSwitchPath;
import org.usfirst.frc.team291.auto.arrays.SwitchToCubeLineUpPath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class TwoCubeSwitchAuto extends AutoMode{
	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	
	private boolean switchOnLeft;
	
	public TwoCubeSwitchAuto(boolean switchOnLeft){
		System.out.println("Starting TwoCubeSwitchAuto");
		timer = new Timer();
		this.switchOnLeft = switchOnLeft;
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
	private State lastState = State.INIT;
	
	private enum State {
		INIT,
		DRIVE_TO_SWITCH,
		EJECT,
		ALIGN_TO_CUBE,
		DRIVE_FORWARD_TO_INTAKE,
		BACK_UP,
		DRIVE_BACK_TO_SWITCH,
		EJECT_AGAIN,
		FINISHED
	}

	@Override
	public void execute() {
		switch(state){
		case INIT:
			init();
			state = State.DRIVE_TO_SWITCH;
			timer.reset();
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
				driveBase.zeroSensors();
			}
			if(timer.get() > 2) cubeIntake.ejectCube(1);
			
			break;

		case EJECT:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(timer.get() < .5) cubeIntake.ejectCube(1);
			else{
				if(switchOnLeft){
					trajectoryLeft = SwitchToCubeLineUpPath.trajectoryArray[0];
					trajectoryRight = SwitchToCubeLineUpPath.trajectoryArray[1];
				}
				else{
					trajectoryLeft = SwitchToCubeLineUpPath.trajectoryArray[1];
					trajectoryRight = SwitchToCubeLineUpPath.trajectoryArray[0];
				}
				state = State.ALIGN_TO_CUBE;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
			}
			break;
			
		case ALIGN_TO_CUBE:
			cubeArm.setWantedArmState(ArmState.ACQUIRING);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				trajectoryLeft = ForwardCubePickupPath.trajectoryArray[0];
				trajectoryRight = ForwardCubePickupPath.trajectoryArray[1];
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
				state = State.DRIVE_FORWARD_TO_INTAKE;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroSensors();
			}
			break;
			
		case DRIVE_FORWARD_TO_INTAKE:
			cubeArm.setWantedArmState(ArmState.ACQUIRING);
			cubeIntake.powerIntake(true);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				trajectoryLeft = ForwardCubePickupPath.trajectoryArray[0];
				trajectoryRight = ForwardCubePickupPath.trajectoryArray[1];
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				state = State.BACK_UP;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroSensors();
			}
			break;
		
		case BACK_UP:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				if(switchOnLeft){
					trajectoryLeft = CubeToSwitchPath.trajectoryArray[0];
					trajectoryRight = CubeToSwitchPath.trajectoryArray[1];
				}
				else{
					trajectoryLeft = CubeToSwitchPath.trajectoryArray[1];
					trajectoryRight = CubeToSwitchPath.trajectoryArray[0];
				}
				state = State.DRIVE_BACK_TO_SWITCH;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
				driveBase.zeroSensors();
			}
			break;
			
		case DRIVE_BACK_TO_SWITCH:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				state = State.EJECT_AGAIN;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroSensors();
			}
			break;
			
		case EJECT_AGAIN:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(timer.get() < .5) cubeIntake.ejectCube(0);
			else if(timer.get() < 2) cubeIntake.ejectCube(1);
			else{
				state = State.FINISHED;
				cubeIntake.ejectCube(0);
			}
			break;
			
		case FINISHED:
			cubeIntake.ejectCube(0);//stop ejecting the cube
			cubeArm.setWantedArmState(ArmState.IDLE);
			driveBase.setLeftRightPower(0,0);
			break;
		}
		
		if(state != lastState) System.out.println(state);
		lastState = state;
		
	}

	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		return true;
	}
	

}
