package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.LeftScaleToIntakePath;
import org.usfirst.frc.team291.auto.arrays.NearScalePath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class NearScaleSwitchAuto extends AutoMode{


	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	
	
	private boolean startOnLeft;
	
	public NearScaleSwitchAuto(boolean startOnLeft){
		System.out.println("Starting NearScaleSwitchAuto");
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
		PAUSE_FOR_ARM,
		DRIVE_TO_CUBE,
		LIFT_ARM,
		EJECT_IN_SWITCH,
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
			if(timer.get() > .5 && cubeArm.getSystemState() == ArmState.MIDDLE_SCALE_LEVEL){
				state = State.EJECT;
				timer.reset();
			}
			break;
		case EJECT:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < 1) cubeIntake.ejectCube(.75);
			else if(startOnLeft){
				trajectoryLeft = LeftScaleToIntakePath.trajectoryArray[0];
				trajectoryRight = LeftScaleToIntakePath.trajectoryArray[1];
				state = State.PAUSE_FOR_ARM;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			else if(!startOnLeft){
				/*trajectoryLeft = RightScaleToIntakePath.trajectoryArray[1];
				trajectoryRight = RightScaleToIntakePath.trajectoryArray[0];*/
				trajectoryLeft = LeftScaleToIntakePath.trajectoryArray[1];
				trajectoryRight = LeftScaleToIntakePath.trajectoryArray[0];
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
			if(timer.get() > 1.2){
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
				/*if(startOnLeft && scaleOnLeft){
					trajectoryLeft = IntakeToScalePath.trajectoryArray[0];
					trajectoryRight = IntakeToScalePath.trajectoryArray[1];
					timer.reset();
					controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				}
				else if(!startOnLeft && !scaleOnLeft){
					trajectoryLeft = ScaleToIntakePath.trajectoryArray[1];
					trajectoryRight = ScaleToIntakePath.trajectoryArray[0];
					timer.reset();
					controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				}*/
				state = State.LIFT_ARM;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
			}
			break;
			
		case LIFT_ARM:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			/*if(!cubeArm.armIsStable()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}*/
			if(cubeArm.armIsStable() && timer.get() > 1){ // Finished the path.
				state = State.EJECT_IN_SWITCH;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
			}
			else{
				if(timer.get() < .6) driveBase.setLeftRightPower(-.3, -.3);
			}
			break;
			
		case EJECT_IN_SWITCH:
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			if(timer.get() < 1){
				cubeIntake.ejectCube(0);
				if(startOnLeft) driveBase.setLeftRightPower(.65, .5);
				else driveBase.setLeftRightPower(.5, .65);
			}
			else if(timer.get() < 2.8) cubeIntake.ejectCube(.7);
			else{
				state = State.FINISHED;
				cubeIntake.ejectCube(0);
			}
			break;
			
		case FINISHED:
			cubeIntake.ejectCube(0);//stop ejecting the cube
			cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);
			driveBase.setLeftRightPower(0,0);
			break;
		}
		
		if(state != lastState) System.out.println(state);
		lastState = state;
		
	}

	@Override
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		if(startOnLeft == scaleOnLeft && startOnLeft == switchOnLeft) return true;
		else return false;
	}


 
	@Override
	public void outputToSmartDashboard() {		
	}
}
