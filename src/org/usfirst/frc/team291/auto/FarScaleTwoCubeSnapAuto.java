package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.FarIntakeToScalePath;
import org.usfirst.frc.team291.auto.arrays.RightFarScaleToIntakePath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class FarScaleTwoCubeSnapAuto extends AutoMode{
	
	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	private Timer autoTime;
	
	
	private boolean startOnLeft;

	public FarScaleTwoCubeSnapAuto(boolean startOnLeft){
		System.out.println("Starting FarScaleTwoCubeSnapAuto");
		this.startOnLeft = startOnLeft;
		timer = new Timer();
		autoTime = new Timer();
		timer.start();
		autoTime.start();
	}
	@Override
	public void init() {
		driveBase.zeroSensors();
		driveBase.resetPID();
	}
	
	private State state = State.INIT;
	private State lastState = State.INIT;
	
	private enum State {
		INIT,
		DRIVE_FORWARD,
		TURN_90,
		DRIVE_ACROSS,
		TURN_TOWARD_SCALE,
		DRIVE_TO_SCALE,
		EJECT,
		PAUSE_FOR_ARM,
		DRIVE_TO_CUBE,
		DRIVE_BACK_TO_SCALE,
		EJECT_AGAIN,
		FINISHED
	}

	@Override
	public void execute() {
		switch(state){
		case INIT:
			init();
			state = State.DRIVE_FORWARD;
			//timer.reset();
			break;

		case DRIVE_FORWARD:
			if(!driveBase.distanceIsStable){
				driveBase.driveToDistancePID(-190.0/12.0, .7, 0.0);
			}
			else{
				driveBase.resetPID();
				state = State.TURN_90;
			}
			break;
			
		case TURN_90:
			if(!driveBase.angleIsStable){
				driveBase.turnToAngle(90.0);
			}
			else{
				driveBase.resetPID();
				state = State.DRIVE_ACROSS;
			}
			break;
			
		case DRIVE_ACROSS:
			if(!driveBase.distanceIsStable){
				driveBase.driveToDistancePID(-190.0/12.0, .7, 90.0);
			}
			else{
				driveBase.resetPID();
				state = State.TURN_TOWARD_SCALE;
			}
			break;
		
		case TURN_TOWARD_SCALE:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			if(!driveBase.angleIsStable){
				driveBase.turnToAngle(-15.0);
			}
			else{
				driveBase.resetPID();
				state = State.DRIVE_TO_SCALE;
			}
			break;
		
		case DRIVE_TO_SCALE:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			if(!driveBase.distanceIsStable){
				driveBase.driveToDistancePID(-29.0/12.0, .7, -15.0);//-24
			}
			else{
				timer.reset();
				driveBase.resetPID();
				state = State.EJECT;
			}
			break;
			
		case EJECT:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < .0) cubeIntake.ejectCube(0);
			else if(timer.get() < .4) cubeIntake.ejectCube(.45);// .5
			else if(startOnLeft){
				trajectoryLeft = RightFarScaleToIntakePath.trajectoryArray[1];
				trajectoryRight = RightFarScaleToIntakePath.trajectoryArray[0];
				state = State.PAUSE_FOR_ARM;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			else if(!startOnLeft){
				trajectoryLeft = RightFarScaleToIntakePath.trajectoryArray[0];
				trajectoryRight = RightFarScaleToIntakePath.trajectoryArray[1];
				state = State.PAUSE_FOR_ARM;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			else{
				state = State.FINISHED;
				System.out.println("Finshed Two Cube Scale Auto!");
			}
			break;
			
			case PAUSE_FOR_ARM:
				cubeArm.setWantedArmState(ArmState.ACQUIRING);
				cubeIntake.powerIntake(true);
				//cubeArm.setWantedArmState(ArmState.IDLE);
				if(timer.get() > 1.0){
					state = State.DRIVE_TO_CUBE;
					timer.reset();
					driveBase.zeroEncoders();
				}
				break;
			
			case DRIVE_TO_CUBE:
				cubeArm.setWantedArmState(ArmState.ACQUIRING);
				//cubeArm.setWantedArmState(ArmState.IDLE);
				if(!controller.onTarget()){// Still driving the path.
					controller.update();// does the calculations and updates the driveBase
				}
				else{ // Finished the path.
					if(startOnLeft){
						trajectoryLeft = FarIntakeToScalePath.trajectoryArray[0];
						trajectoryRight = FarIntakeToScalePath.trajectoryArray[1];
						timer.reset();
						controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
					}
					else if(!startOnLeft){
						trajectoryLeft = FarIntakeToScalePath.trajectoryArray[1];
						trajectoryRight = FarIntakeToScalePath.trajectoryArray[0];
						timer.reset();
						controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
					}
					state = State.DRIVE_BACK_TO_SCALE;
					timer.reset();
					driveBase.setLeftRightPower(0,0);
					driveBase.zeroEncoders();
				}
				break;
				
			case DRIVE_BACK_TO_SCALE:
				cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
				//cubeArm.setWantedArmState(ArmState.IDLE);
				if(!controller.onTarget()){// Still driving the path.
					controller.update();// does the calculations and updates the driveBase
				}
				else{ // Finished the path.
					state = State.EJECT_AGAIN;
					timer.reset();
					driveBase.setLeftRightPower(0,0);
					System.out.println(autoTime.get());
				}
				break;
				
			case EJECT_AGAIN:
				cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
				//cubeArm.setWantedArmState(ArmState.IDLE);
				if(timer.get() < 0) cubeIntake.ejectCube(0);
				else if(timer.get() < 1) cubeIntake.ejectCube(.75);
				else{
					state = State.FINISHED;
					cubeIntake.ejectCube(0);
				}
				break;
				
			case FINISHED:
				cubeIntake.ejectCube(0);//stop ejecting the cube
				cubeArm.setWantedArmState(ArmState.LOW_SCALE_LEVEL);
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
		// TODO Auto-generated method stub
		
	}

}
