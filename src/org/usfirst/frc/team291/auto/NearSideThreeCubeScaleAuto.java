package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.IntakeToScalePath;
import org.usfirst.frc.team291.auto.arrays.LeftScaleToIntakePath;
import org.usfirst.frc.team291.auto.arrays.LeftScaleToSecondIntakePath;
import org.usfirst.frc.team291.auto.arrays.RightScaleToIntakePath;
import org.usfirst.frc.team291.auto.arrays.SlowNearScalePath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class NearSideThreeCubeScaleAuto extends AutoMode{


	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	private Timer runTime;
	
	
	private boolean startOnLeft;
	
	public NearSideThreeCubeScaleAuto(boolean startOnLeft){
		System.out.println("Starting NearSideThreeCubeScaleAuto");
		timer = new Timer();
		runTime = new Timer();
		this.startOnLeft = startOnLeft;
		if(startOnLeft){
			trajectoryLeft = SlowNearScalePath.trajectoryArray[0];
			trajectoryRight = SlowNearScalePath.trajectoryArray[1];
		}
		else {
			trajectoryLeft = SlowNearScalePath.trajectoryArray[1];
			trajectoryRight = SlowNearScalePath.trajectoryArray[0];
		}
		controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
		timer.start();
		runTime.start();
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
		DRIVE_BACK_TO_SCALE,
		EJECT_AGAIN,
		PAUSE_FOR_ARM_AGAIN,
		DRIVE_TO_SECOND_CUBE,
		DRIVE_BACK_TO_SCALE_AGAIN,
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
				//cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
				cubeArm.setWantedArmState(ArmState.AUTO_SCALE_PREP);
				cubeIntake.ejectCube(1);
			}
			else if(timer.get() < 1){
				cubeArm.setWantedArmState(ArmState.IDLE);
			}
			else cubeArm.setWantedArmState(ArmState.AUTO_SCALE_PREP);
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
			//cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			driveBase.holdPosition();
			timer.reset();
			state = State.EJECT;
			break;
			
			
		case EJECT:
			//cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			cubeArm.setWantedArmState(ArmState.AUTO_SCALE_PREP);
			driveBase.holdPosition();
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < .01) cubeIntake.ejectCube(1);
			else if(startOnLeft){
				trajectoryLeft = LeftScaleToIntakePath.trajectoryArray[0];
				trajectoryRight = LeftScaleToIntakePath.trajectoryArray[1];
				state = State.PAUSE_FOR_ARM;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			
			else{
				trajectoryLeft = RightScaleToIntakePath.trajectoryArray[1];
				trajectoryRight = RightScaleToIntakePath.trajectoryArray[0];
				state = State.PAUSE_FOR_ARM;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			break;
		
		case PAUSE_FOR_ARM:
			cubeArm.setWantedArmState(ArmState.ACQUIRING);
			cubeIntake.powerIntake(true);
			driveBase.holdPosition();
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
				if(startOnLeft){
					trajectoryLeft = IntakeToScalePath.trajectoryArray[0];
					trajectoryRight = IntakeToScalePath.trajectoryArray[1];
					timer.reset();
					controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				}
				else{
					trajectoryLeft = IntakeToScalePath.trajectoryArray[1];
					trajectoryRight = IntakeToScalePath.trajectoryArray[0];
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
			if(timer.get() > .5) cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			else{
				System.out.println("waitingForWrist");
				cubeArm.setWantedArmState(ArmState.STOWED);
			}
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				state = State.EJECT_AGAIN;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
			}
			break;
			
		case EJECT_AGAIN:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			driveBase.holdPosition();
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < .25) cubeIntake.ejectCube(0);
			else if(timer.get() < .75) cubeIntake.ejectCube(1);
			else if(startOnLeft){
				trajectoryLeft = LeftScaleToSecondIntakePath.trajectoryArray[0];
				trajectoryRight = LeftScaleToSecondIntakePath.trajectoryArray[1];
				state = State.PAUSE_FOR_ARM_AGAIN;
				timer.reset();
				cubeIntake.ejectCube(0);
				//driveBase.zeroEncoders();
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			else if(!startOnLeft){
				trajectoryLeft = RightScaleToIntakePath.trajectoryArray[1];
				trajectoryRight = RightScaleToIntakePath.trajectoryArray[0];
				state = State.PAUSE_FOR_ARM_AGAIN;
				timer.reset();
				cubeIntake.ejectCube(0);
				//driveBase.zeroEncoders();
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			else{
				state = State.FINISHED;
				cubeIntake.ejectCube(0);
			}
			break;
			
		case PAUSE_FOR_ARM_AGAIN:
			cubeArm.setWantedArmState(ArmState.ACQUIRING);
			cubeIntake.powerIntake(true);
			driveBase.holdPosition();
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() > 1.1){
				state = State.DRIVE_TO_SECOND_CUBE;
				timer.reset();
				//driveBase.zeroEncoders();
			}
			break;
		
		case DRIVE_TO_SECOND_CUBE:
			cubeArm.setWantedArmState(ArmState.ACQUIRING);
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				if(startOnLeft){
					trajectoryLeft = IntakeToScalePath.trajectoryArray[0];
					trajectoryRight = IntakeToScalePath.trajectoryArray[1];
					timer.reset();
					controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				}
				else if(!startOnLeft){
					trajectoryLeft = IntakeToScalePath.trajectoryArray[1];
					trajectoryRight = IntakeToScalePath.trajectoryArray[0];
					timer.reset();
					controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				}
				state = State.DRIVE_BACK_TO_SCALE_AGAIN;
				timer.reset();
				System.out.println(runTime.get());
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
			}
			break;
		
		case DRIVE_BACK_TO_SCALE_AGAIN:
			//cubeArm.setWantedArmState(ArmState.LOW_SCALE_LEVEL);
			cubeArm.setWantedArmState(ArmState.IDLE);
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				state = State.FINISHED;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
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
		if(startOnLeft == scaleOnLeft) return true;
		else return false;
	}


	@Override
	public void outputToSmartDashboard() {
	}
}
