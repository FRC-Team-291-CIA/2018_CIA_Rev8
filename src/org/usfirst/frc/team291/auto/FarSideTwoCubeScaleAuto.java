package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.auto.arrays.FarScaleForwardPath;
import org.usfirst.frc.team291.auto.arrays.FarScaleTurnPath;
import org.usfirst.frc.team291.auto.arrays.IntakeToScalePath;
import org.usfirst.frc.team291.auto.arrays.LeftFarScalePath;
import org.usfirst.frc.team291.auto.arrays.RightFarScalePath;
import org.usfirst.frc.team291.auto.arrays.RightFarScaleToIntakePath;
import org.usfirst.frc.team291.auto.arrays.RightScaleToIntakePath;
import org.usfirst.frc.team291.pathfollower.Trajectory;
import org.usfirst.frc.team291.pathfollower.TrajectoryDriveController;
import org.usfirst.frc.team291.subsystems.CubeArm.ArmState;

import edu.wpi.first.wpilibj.Timer;

public class FarSideTwoCubeScaleAuto extends AutoMode{

	private TrajectoryDriveController controller;
	private Trajectory trajectoryLeft;
	private Trajectory trajectoryRight;
	private Timer timer;
	
	
	private boolean startOnLeft;
	
	public FarSideTwoCubeScaleAuto(boolean startOnLeft){//TODO is for near side right now.
		System.out.println("Starting FarSideOneCubeScaleAuto");
		timer = new Timer();
		this.startOnLeft = startOnLeft;
		if(startOnLeft){
				//Start on the left, scale on the left
				trajectoryLeft = RightFarScalePath.trajectoryArray[0];
				trajectoryRight = RightFarScalePath.trajectoryArray[1];
		}
		else {
				//Start on the right, scale on the right
				trajectoryLeft = LeftFarScalePath.trajectoryArray[1];
				trajectoryRight = LeftFarScalePath.trajectoryArray[0];
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
		TURN_TOWARD_SCALE,
		DRIVE_FORWARD,
		RAISE_ARM,
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
			state = State.DRIVE_TO_SCALE;
			timer.reset();
			break;

		case DRIVE_TO_SCALE:
			/*if(controller.almostDone()); //cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			else if(timer.get() < 2) cubeArm.setWantedArmState(ArmState.IDLE);
			else cubeArm.setWantedArmState(ArmState.SWITCH_LEVEL);*/
			
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ // Finished the path.
				trajectoryLeft = FarScaleTurnPath.trajectoryArray[0];
				trajectoryRight = FarScaleTurnPath.trajectoryArray[1];
				if(startOnLeft) controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
				else controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
				state = State.TURN_TOWARD_SCALE;
				//state = State.FINISHED;
				timer.reset();
				driveBase.setLeftRightPower(0,0);
				driveBase.zeroEncoders();
				driveBase.resetPID();
				if(!driveBase.gyroIsConnected()) System.out.println("Gyro is not connected!!! Using encoders to execute turn.");
			}
			break;
			
		case TURN_TOWARD_SCALE:
			if(driveBase.gyroIsConnected()){
				if(!driveBase.angleIsStable) driveBase.turnToAngle(-15.0);//TODO
				else{
					trajectoryLeft = FarScaleForwardPath.trajectoryArray[0];
					trajectoryRight = FarScaleForwardPath.trajectoryArray[1];
					controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
					timer.reset();
					driveBase.setLeftRightPower(0, 0);
					driveBase.zeroSensors();
					state = State.DRIVE_FORWARD;
				}
			}
			else{
				if(!controller.onTarget()){// Still driving the path.
					controller.updateTurn();// does the calculations and updates the driveBase
				}
				else{
					trajectoryLeft = FarScaleForwardPath.trajectoryArray[0];
					trajectoryRight = FarScaleForwardPath.trajectoryArray[1];
					controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, -1.0);
					timer.reset();
					driveBase.setLeftRightPower(0, 0);
					driveBase.zeroSensors();
					state = State.DRIVE_FORWARD;	
				}
			}
			break;
			
		case DRIVE_FORWARD:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			if(!controller.onTarget()){// Still driving the path.
				controller.update();// does the calculations and updates the driveBase
			}
			else{ 
				timer.reset();
				driveBase.setLeftRightPower(0, 0);
				driveBase.zeroSensors();
				state = State.RAISE_ARM;
			}
			break;

		case RAISE_ARM:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			if(timer.get() > .7 && cubeArm.getSystemState() == ArmState.MIDDLE_SCALE_LEVEL){
				state = State.EJECT;
				timer.reset();
			}
			driveBase.holdPosition();
			break;
				
		case EJECT:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			driveBase.holdPosition();
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < 1) cubeIntake.ejectCube(.65);
			else if(startOnLeft){
				trajectoryLeft = RightFarScaleToIntakePath.trajectoryArray[1];
				trajectoryRight = RightFarScaleToIntakePath.trajectoryArray[0];
				state = State.PAUSE_FOR_ARM;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			else if(!startOnLeft){
				trajectoryLeft = RightScaleToIntakePath.trajectoryArray[1];
				trajectoryRight = RightScaleToIntakePath.trajectoryArray[0];
				state = State.PAUSE_FOR_ARM;
				timer.reset();
				cubeIntake.ejectCube(0);
				controller = new TrajectoryDriveController(trajectoryLeft, trajectoryRight, 1.0);
			}
			else{
				state = State.FINISHED;
				//System.out.println("Finshed Two Cube Scale Auto!");
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
			cubeArm.setWantedArmState(ArmState.STOWED);
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
				//state = State.DRIVE_BACK_TO_SCALE;
				state = State.FINISHED;
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
			}
			break;
			
		case EJECT_AGAIN:
			cubeArm.setWantedArmState(ArmState.MIDDLE_SCALE_LEVEL);
			//cubeArm.setWantedArmState(ArmState.IDLE);
			if(timer.get() < .75) cubeIntake.ejectCube(0);
			else if(timer.get() < 2.5) cubeIntake.ejectCube(.75);
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
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		if(startOnLeft == !scaleOnLeft) return true;
		else return false;
	}

	@Override
	public void outputToSmartDashboard() {
		 /*[webdav] putting D:\Users\Julia\workspace\2018_CIA_Rev6\dist\FRCUserProgram.jar to http://10.2.91.2/files/home/lvuser/FRCUserProgram.jar
			   [webdav] Mar 26, 2018 11:48:35 AM org.apache.http.impl.execchain.RetryExec execute
			   [webdav] INFO: I/O exception (java.net.SocketException) caught when processing request to {}->http://10.2.91.2:80: Connection reset by peer: socket write error
			   [webdav] Mar 26, 2018 11:48:35 AM org.apache.http.impl.execchain.RetryExec execute
			   [webdav] INFO: Retrying request to {}->http://10.2.91.2:80
*/
	}
	
}
