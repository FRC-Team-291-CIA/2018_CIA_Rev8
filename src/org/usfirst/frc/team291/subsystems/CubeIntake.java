package org.usfirst.frc.team291.subsystems;

import org.usfirst.frc.team291.robot.CIAConstants;
import org.usfirst.frc.team291.robot.CIAObjects;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeIntake extends Subsystem{
	
	private Spark rollers = new Spark(CIAConstants.cubeIntakePort);
	private Solenoid cubeClamp = new Solenoid(1, CIAConstants.cubeClampPort);
	private Solenoid firstStageWrist = new Solenoid(1, CIAConstants.firstStageWristPort);
	private Solenoid secondStageWrist = new Solenoid(0, CIAConstants.secondStageWristPort);
	private Ultrasonic ultrasonicSensor = new Ultrasonic(CIAConstants.ultrasonicInPort, CIAConstants.ultrasonicOutPort);
	
	private boolean hasCube = false;
	
	private double ejectPower = 0;
	private boolean adjustIn = false;
	private boolean adjustOut = false;
	
	private boolean powerIntake = false;
	
	private Timer deployTimer = new Timer();
	private Timer wristTimer = new Timer();
	
	private double highScalePistonRetractTime = 1;//seconds to wait for wrist
	private double middleScalePistonRetractTime = 0.5;// TODO .5
	private double groundPistonRetractTime = 1.0;// TODO 1.5
	
	private IntakeState extendedWristState = IntakeState.STOWED;
	
	public CubeIntake(){
		deployTimer.start();
		wristTimer.start();
		ultrasonicSensor.setAutomaticMode(true);
	}
	
	public void setRollerPower(double power){rollers.set(power);}//negative power is intaking
	public void clampCube(boolean clamp){cubeClamp.set(!clamp);}	
	public void setFirstStage(boolean out){firstStageWrist.set(out);}
	public void setSecondStage(boolean out){secondStageWrist.set(out);}
	public boolean wristExtended(){return firstStageWrist.get() && secondStageWrist.get();}
	public boolean wristHalfExtended(){return secondStageWrist.get() && !firstStageWrist.get();}
	public void powerIntake(boolean powerIntake){this.powerIntake = powerIntake;}
	public void ejectCube(double power){ejectPower = power;}
	public void adjustCubeIn(boolean adjust){adjustIn = adjust;}
	public void adjustCubeOut(boolean adjust){adjustOut = adjust;}
	
	public boolean waitForWrist(){
		if(extendedWristState == IntakeState.SCORING_HIGH && wristTimer.get() < highScalePistonRetractTime) return true;
		else if(extendedWristState == IntakeState.SCORING_MIDDLE && wristTimer.get() < middleScalePistonRetractTime) return true;
		else if(extendedWristState == IntakeState.INTAKING && wristTimer.get() < groundPistonRetractTime) return true;
		else return false;
	}
	
	public enum IntakeState{
		SCORING_FAST, SCORING_LOW, SCORING_MIDDLE_LOW, SCORING_MIDDLE, SCORING_HIGH, PREPARING_HOOK, HOOKING, SCORING_VAULT, SCORING_SWITCH, INTAKING, STOWED, IDLE
	}
	
	public void setWristState(IntakeState state){
		switch(state){
		case IDLE:
		case STOWED:
		case SCORING_SWITCH:
		case SCORING_LOW:
			firstStageWrist.set(false);
			secondStageWrist.set(false);
			deployTimer.reset();
			clampCube(true);
			break;
		case SCORING_MIDDLE_LOW:
			firstStageWrist.set(true);
			secondStageWrist.set(false);
			deployTimer.reset();
			extendedWristState = IntakeState.SCORING_MIDDLE;
			clampCube(true);
			break;
		case SCORING_MIDDLE:
			firstStageWrist.set(false);
			secondStageWrist.set(true);
			wristTimer.reset();
			deployTimer.reset();
			extendedWristState = IntakeState.SCORING_MIDDLE;
			clampCube(true);
			break;
		case SCORING_VAULT:
			secondStageWrist.set(true);
			extendedWristState = IntakeState.INTAKING;
			wristTimer.reset();
			if(deployTimer.get() > .5) firstStageWrist.set(true);
			clampCube(true);
			break;
		case SCORING_HIGH:
			extendedWristState = IntakeState.SCORING_HIGH;
			firstStageWrist.set(true);
			secondStageWrist.set(true);
			deployTimer.reset();
			wristTimer.reset();
			clampCube(true);
			break;
		case SCORING_FAST:
			firstStageWrist.set(true);
			secondStageWrist.set(false);
			deployTimer.reset();
			clampCube(true);
			break;
		case INTAKING:
			extendedWristState = IntakeState.INTAKING;
			wristTimer.reset();
			if(deployTimer.get() > .25) secondStageWrist.set(true);
			firstStageWrist.set(true);
			if(deployTimer.get() > .4) clampCube(false);
			if(powerIntake) setRollerPower(-.6);
			else setRollerPower(-.4);
			hasCube = false;
			break;
		case PREPARING_HOOK:
			firstStageWrist.set(true);
			secondStageWrist.set(true);
			if(deployTimer.get() > .25) clampCube(false);
			CIAObjects.climber.enableWinch(true);
			break;
		case HOOKING:
			firstStageWrist.set(false);
			secondStageWrist.set(true);
			clampCube(false);
			deployTimer.reset();
			CIAObjects.climber.enableWinch(true);
			extendedWristState = IntakeState.SCORING_HIGH;
			break;
		}
		
			
		//SmartDashboard.putNumber("deployTimer", deployTimer.get());
		
		if(adjustIn) setRollerPower(-.65);
		else if(adjustOut) setRollerPower(.65);
		else if(!(state == IntakeState.SCORING_SWITCH || state == IntakeState.INTAKING)) setRollerPower(ejectPower);
		else if(state == IntakeState.SCORING_SWITCH) setRollerPower(-ejectPower);
	}
	
	public boolean hasCube(){
		double distance = ultrasonicSensor.getRangeInches();
		if(distance > .1 && (distance < 16 || distance > 670)){
			CIAObjects.lights.hasCube = true;
			hasCube = true;
			return true;
		}
		else{
			CIAObjects.lights.hasCube = false;
			hasCube = false;
			return false;
		}
		
	}
	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("Cube Sensor Connected", ultrasonicSensor.getRangeInches() > 0.1);
		SmartDashboard.putBoolean("Has Cube", hasCube);
	}

	@Override
	public void stop() {
		
	}

}
