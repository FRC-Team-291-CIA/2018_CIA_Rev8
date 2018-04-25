package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.ForwardSwitchPath;
import org.usfirst.frc.team291.auto.arrays.SideSwitchPath;
import org.usfirst.frc.team291.auto.arrays.Turn90Path;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class SideSwitchAuto extends AutoMode{
	
	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	
	
	private boolean startOnLeft;
	
	public SideSwitchAuto(boolean startOnLeft){
		System.out.println("Starting SideSwitchAuto");
		timer = new Timer();
		this.startOnLeft = startOnLeft;
		
		trajectoryLeft = SideSwitchPath.trajectoryArray[1];
		trajectoryRight = SideSwitchPath.trajectoryArray[0];
		
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
		DRIVE_FORWARD,
		TURN_TOWARD_SWITCH,
		DRIVE_INTO_SWITCH,
		EJECT_INTO_SWITCH,
		FINISHED
	}

	@Override
	public void execute() {
		switch(state){
		case INIT:
			init();
			state = State.DRIVE_FORWARD;
			timer.reset();
			break;
	
		case DRIVE_FORWARD:
			cubeArm.setWantedArmState(ArmState.IDLE);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				state = State.TURN_TOWARD_SWITCH;
				trajectoryLeft = Turn90Path.trajectoryArray[0];
				trajectoryRight = Turn90Path.trajectoryArray[1];
				if(startOnLeft) controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
				else controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
				driveBase.resetPID();
				if(!driveBase.gyroIsConnected()) System.out.println("Gyro is not connected!!! Using encoders to execute turn.");
			}
			break;
			
		case TURN_TOWARD_SWITCH:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(startOnLeft){
				if(driveBase.gyroIsConnected()){
					if(!driveBase.angleIsStable && timer.get() < 4.0) driveBase.turnToAngle(-90);//TODO
					else{
						trajectoryLeft = ForwardSwitchPath.trajectoryArray[0];
						trajectoryRight = ForwardSwitchPath.trajectoryArray[1];
						controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
						driveBase.setLeftRightPower(0, 0);
						driveBase.zeroEncoders();
						state = State.DRIVE_INTO_SWITCH;
					}
				}
				else{
					if(!controller.onTarget()){// Still driving the path.
						controller.updateTurn();// does the calculations and updates the driveBase
					}
					else{
						trajectoryLeft = ForwardSwitchPath.trajectoryArray[0];
						trajectoryRight = ForwardSwitchPath.trajectoryArray[1];
						controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
						driveBase.setLeftRightPower(0, 0);
						driveBase.zeroEncoders();
						state = State.DRIVE_INTO_SWITCH;
					}
				}
			}
			else{
				if(driveBase.gyroIsConnected()){
					if(!driveBase.angleIsStable && timer.get() < 4.0) driveBase.turnToAngle(90);//TODO
					else{
						trajectoryLeft = ForwardSwitchPath.trajectoryArray[0];
						trajectoryRight = ForwardSwitchPath.trajectoryArray[1];
						controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
						driveBase.setLeftRightPower(0, 0);
						driveBase.zeroEncoders();
						state = State.DRIVE_INTO_SWITCH;
					}
				}
				else{
					if(!controller.onTarget()){// Still driving the path.
						controller.updateTurn();// does the calculations and updates the driveBase
					}
					else{
						trajectoryLeft = ForwardSwitchPath.trajectoryArray[0];
						trajectoryRight = ForwardSwitchPath.trajectoryArray[1];
						controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
						driveBase.setLeftRightPower(0, 0);
						driveBase.zeroEncoders();
						state = State.DRIVE_INTO_SWITCH;
					}
				}
			}
			break;
			
		case DRIVE_INTO_SWITCH:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				state = State.EJECT_INTO_SWITCH;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
			}
			break;
			
		case EJECT_INTO_SWITCH:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(timer.get() < 2) cubeIntake.setRollerPower(-.8);//TODO
			else{
				cubeIntake.ejectCube(0);
				state = State.FINISHED;
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
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		if(startOnLeft == switchOnLeft) return true;
		else return false;
	}

	@Override
	public void outputToSmartDashboard() {		
	}
	

}
