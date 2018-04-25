package org.usfirst.frc.team291.subsystems;

import org.usfirst.frc.team291.robot.CIAConstants;
import org.usfirst.frc.team291.robot.CIAObjects;
import org.usfirst.frc.team291.subsystems.CubeIntake.IntakeState;
import org.usfirst.frc.team291.util.AnalogEncoder;
import org.usfirst.frc.team291.util.CIAMath;
import org.usfirst.frc.team291.util.PID;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeArm extends Subsystem{
	
	private Spark armMotorA = new Spark(CIAConstants.armMotorA);
	private Spark armMotorB = new Spark(CIAConstants.armMotorB);
	
	public AnalogEncoder primaryEncoder = new AnalogEncoder(0);
	public AnalogEncoder secondaryEncoder = new AnalogEncoder(1);
	public Encoder tertiaryEncoder = new Encoder(CIAConstants.tertiaryArmEncoderPortA, CIAConstants.tertiaryArmEncoderPortB);
	
	private ArmState systemState = ArmState.IDLE;
	private ArmState lastSystemState = ArmState.STOWED;
	private PIDState pidState = PIDState.RELAXED;
	
	private Timer waitTimer = new Timer();
	private Timer shakeTimer = new Timer();
	private double currentCoMDist = 22.4;
	private double currentCoMAngleOffset = 10;
	private double armMass = 20;

	private final double springDist = 3.7;
	private final double springAngleOffset = 19.6;
	private final double springForce = 50;//lbs
	private final double springX = 4;
	private final double springY = 15.4;
	
	private double currentArmAngle;
	private double wantedArmAngle;
	private double armEbsilon = 4;
	private double stowedAngle = 20;
	private double groundAngle = 40;
	private double switchAngle = 70;//80 TODO
	private double frontScaleAngle = 120;
	private double scaleAngle = 170;
	private double autoPrepAngle = 170;
	private double highArmThreshold = 100;
	
	private PID armPID = new PID(.045, 0, .2, armEbsilon);
	
	public CubeArm(){
		armPID.setMaxOutput(.7);
		waitTimer.start();
		shakeTimer.start();
		primaryEncoder.setWorkingZero(CIAConstants.primaryEncoderZero);
		secondaryEncoder.setWorkingZero(CIAConstants.secondaryEncoderZero);
		primaryEncoder.setUnpluggedVoltage(CIAConstants.primaryUnpluggedVoltage);
		secondaryEncoder.setUnpluggedVoltage(CIAConstants.secondaryUnpluggedVoltage);
		secondaryEncoder.setInverted(true);
	}
	
	public static enum CubeState{
		ACQUIRING, SCORING, IDLE
	}
	
	public static enum PIDState{
		ACTIVE, HOLDING, RELAXED
	}
	
	public static enum ArmState{
		ACQUIRING, STOWED, VAULT_LEVEL, SWITCH_LEVEL, LOW_SCALE_LEVEL, MIDDLE_LOW_SCALE_LEVEL, MIDDLE_SCALE_LEVEL, FAST_SCALE_LEVEL, AUTO_SCALE_PREP,
		FRONT_SCALE_LEVEL, HIGH_SCALE_LEVEL, HOOK_LEVEL, HOOKING, MOVING, PAUSING, INTERUPTED, IDLE
	}
	
	public void setWantedArmState(ArmState wantedState){
		pidState = PIDState.ACTIVE;
		switch(wantedState){
		case ACQUIRING:
			if(systemState == ArmState.ACQUIRING || systemState == ArmState.VAULT_LEVEL || systemState == ArmState.STOWED){
				wantedArmAngle = groundAngle;
				systemState = ArmState.ACQUIRING;
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else if(!armIsStable(groundAngle)){
				systemState = ArmState.MOVING;
				wantedArmAngle = groundAngle;
			}
			else{
				wantedArmAngle = groundAngle;
				systemState = ArmState.ACQUIRING;
			}
			break;
		case STOWED:
			if(systemState == ArmState.STOWED){
				wantedArmAngle = groundAngle;
				systemState = ArmState.STOWED;
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else{
				wantedArmAngle = groundAngle;
				if(!armIsStable()){
					systemState = ArmState.MOVING;
				}
				else systemState = ArmState.STOWED;
			}
			break;
		case VAULT_LEVEL:
			if(systemState == ArmState.VAULT_LEVEL || systemState == ArmState.ACQUIRING){
				wantedArmAngle = groundAngle;
				systemState = ArmState.VAULT_LEVEL;
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else if(!armIsStable()){
				wantedArmAngle = groundAngle;
				systemState = ArmState.MOVING;
			}
			else {
				wantedArmAngle = groundAngle;
				systemState = ArmState.VAULT_LEVEL;
			}
			break;
		case SWITCH_LEVEL:
			if(systemState == ArmState.SWITCH_LEVEL){
				wantedArmAngle = switchAngle;
				systemState = ArmState.SWITCH_LEVEL;
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else if(!armIsStable()){
				wantedArmAngle = switchAngle;
				systemState = ArmState.MOVING;
			}
			else{
				wantedArmAngle = switchAngle;
				systemState = ArmState.SWITCH_LEVEL;
			}
			break;
		case FRONT_SCALE_LEVEL:
			if(systemState == ArmState.FRONT_SCALE_LEVEL){
				wantedArmAngle = frontScaleAngle + armAngleOffset;
				systemState = ArmState.FRONT_SCALE_LEVEL;
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else{
				wantedArmAngle = frontScaleAngle + armAngleOffset;
				systemState = ArmState.FRONT_SCALE_LEVEL;
			}
			break;
		case FAST_SCALE_LEVEL:
			if(systemState == ArmState.FAST_SCALE_LEVEL){
				wantedArmAngle = 140.0;
				systemState = ArmState.FAST_SCALE_LEVEL;
			}
			else if(shakeTimer.get() < .3){
				systemState = ArmState.FAST_SCALE_LEVEL;
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				//wantedArmAngle = 140.0;
				systemState = ArmState.PAUSING;
			}
			else{
				wantedArmAngle = 140;
				systemState = ArmState.FAST_SCALE_LEVEL;
			}
			/*if(systemState == ArmState.FAST_SCALE_LEVEL){
				wantedArmAngle = 140.0;
				systemState = ArmState.FAST_SCALE_LEVEL;
			}
			else if(shakeTimer.get() > 0.08 && shakeTimer.get() < .2){
				wantedArmAngle = 140.0;
				systemState = ArmState.FAST_SCALE_LEVEL;
				shakeTimer.reset();
				pidState = PIDState.RELAXED;
				System.out.println("relaxed");
			}
			else if(getCurrentArmAngle() < 100){
				wantedArmAngle = 140.0;
				systemState = ArmState.MOVING;
				shakeTimer.reset();
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				//wantedArmAngle = 140.0;
				systemState = ArmState.PAUSING;
			}
			else{
				wantedArmAngle = 140;
				systemState = ArmState.FAST_SCALE_LEVEL;
			}*/
			break;
		case LOW_SCALE_LEVEL:
		case MIDDLE_LOW_SCALE_LEVEL:
		case MIDDLE_SCALE_LEVEL:
		case HIGH_SCALE_LEVEL:
			if(systemState == ArmState.LOW_SCALE_LEVEL || systemState == ArmState.MIDDLE_SCALE_LEVEL || 
			systemState == ArmState.MIDDLE_LOW_SCALE_LEVEL || systemState == ArmState.HIGH_SCALE_LEVEL || systemState == ArmState.AUTO_SCALE_PREP){
				wantedArmAngle = scaleAngle;
				systemState = wantedState;
			}
			else if(systemState != ArmState.LOW_SCALE_LEVEL && CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else if(!armIsHigh()){
				wantedArmAngle = scaleAngle;
				systemState = ArmState.MOVING;
			}
			else{
				wantedArmAngle = scaleAngle;
				systemState = wantedState;
			}
			break;
		case HOOK_LEVEL:
			if(systemState == ArmState.LOW_SCALE_LEVEL || systemState == ArmState.MIDDLE_SCALE_LEVEL || 
			systemState == ArmState.MIDDLE_LOW_SCALE_LEVEL || systemState == ArmState.HIGH_SCALE_LEVEL || systemState == ArmState.AUTO_SCALE_PREP){
				wantedArmAngle = scaleAngle;
				systemState = wantedState;
			}
			else if(systemState != ArmState.LOW_SCALE_LEVEL && CIAObjects.cubeIntake.waitForWrist()){
				//systemState = ArmState.PAUSING;
			}
			else if(!armIsHigh()){
				wantedArmAngle = scaleAngle;
				systemState = ArmState.MOVING;
			}
			else{
				wantedArmAngle = scaleAngle;
				systemState = wantedState;
			}
			waitTimer.reset();
			break;
		case HOOKING:
			if((waitTimer.get() > 1.0)) wantedArmAngle = 170.0;
			systemState = ArmState.HOOKING;
			break;
		case AUTO_SCALE_PREP:
			
			if(getCurrentArmAngle() > 100.0){
				systemState = ArmState.AUTO_SCALE_PREP;
				wantedArmAngle = autoPrepAngle;
			}
			else if(CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else{
				wantedArmAngle = autoPrepAngle;
				systemState = ArmState.MOVING;
			}
			break;
		case MOVING:
			break;
		case IDLE:
			if(CIAObjects.cubeIntake.waitForWrist()){
				systemState = ArmState.PAUSING;
			}
			else if(getCurrentArmAngle() > stowedAngle + 3.0){
				wantedArmAngle = stowedAngle;
			}
			else pidState = PIDState.RELAXED;
			break;
		case INTERUPTED:
			systemState = ArmState.INTERUPTED;
			//wantedArmAngle = currentArmAngle;
			break;
		default:
			break;
		}
		if(wantedState != ArmState.FAST_SCALE_LEVEL) shakeTimer.reset();
		if(systemState != lastSystemState) System.out.println(systemState);
		lastSystemState = systemState;
		update();
	}
	
	public void update(){
		refreshCurrentCoM();
		currentArmAngle = getCurrentArmAngle();
		if(pidState == PIDState.ACTIVE) setArmAngle(wantedArmAngle);
		else setArmPower(0);
		
		switch(systemState){
		case ACQUIRING:
			CIAObjects.cubeIntake.setWristState(IntakeState.INTAKING);
			break;
		case STOWED:
			CIAObjects.cubeIntake.setWristState(IntakeState.STOWED);
			break;
		case VAULT_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_VAULT);
			break;
		case SWITCH_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_SWITCH);
			break;
		case FRONT_SCALE_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_SWITCH);
			break;
		case FAST_SCALE_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_FAST);
			break;
		case LOW_SCALE_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_LOW);
			break;
		case MIDDLE_LOW_SCALE_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_MIDDLE_LOW);
			break;
		case MIDDLE_SCALE_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_MIDDLE);
			break;
		case HIGH_SCALE_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_HIGH);
			break;
		case HOOK_LEVEL:
			CIAObjects.cubeIntake.setWristState(IntakeState.PREPARING_HOOK);
			break;
		case HOOKING:
			CIAObjects.cubeIntake.setWristState(IntakeState.HOOKING);
			break;
		case AUTO_SCALE_PREP:
			CIAObjects.cubeIntake.setWristState(IntakeState.SCORING_MIDDLE);
			break;
		case MOVING:
			CIAObjects.cubeIntake.setWristState(IntakeState.STOWED);
			break;
		case PAUSING:
			CIAObjects.cubeIntake.setWristState(IntakeState.STOWED);
		case INTERUPTED:
			break;
		default:
			break;
		}
	}
	
	private boolean stayExtended = false;
	private boolean lastToggle = false;
	
	public void toggleVaultState(boolean toggle){
		if(toggle){
			if(!lastToggle) stayExtended = !stayExtended;
		}
		lastToggle = toggle;
	}
	
	public boolean stayExtended(){
		return stayExtended;
	}
	
	public boolean armIsStable(){
		if(Math.abs(currentArmAngle - wantedArmAngle) < armEbsilon){
			return true;
		}
		else return false;
	}
	
	public boolean armIsStable(double wantedArmAngle){
		if(Math.abs(currentArmAngle - wantedArmAngle) < armEbsilon){
			return true;
		}
		else return false;
	}
	
	public boolean armIsHigh(){
		if(getCurrentArmAngle() > highArmThreshold) return true;
		else return false;
	}
	
	public void setArmAngle(double desiredAngle){
		armPID.setDesiredValue(desiredAngle);

			double output = armPID.calcPID(currentArmAngle) + calcHoldPosTorque(currentArmAngle);
			if(getCurrentArmAngle() != 0) setArmPower(output);
	}
	
	public void setArmPower(double power){
		armMotorA.set(-power);
		armMotorB.set(-power);
	}
	
	public void setArmMotorA(double power){ armMotorA.set(-power);}
	public void setArmMotorB(double power){ armMotorB.set(-power);}
	
	private double calcHoldPosTorque(double armAngle){
		double d = Math.sqrt(springX*springX + springY*springY);
		double gravityMoment = currentCoMDist * CIAMath.sin(180 - armAngle - currentCoMAngleOffset) * armMass;
		double springLength = Math.sqrt(d*d + springDist*springDist - 2*d*springDist*CIAMath.cos(180 - CIAMath.atan(springX/springY) - (180 - armAngle) - springAngleOffset));
		double springMoment = springForce * springDist * CIAMath.sin(CIAMath.asin(d * CIAMath.sin(180 - CIAMath.atan(springX/springY) - (180 - armAngle) - springAngleOffset)
				/ springLength));
		return (gravityMoment - springMoment)/1800 + .05;
	}
	
	private boolean primaryEncoderGood = true;
	private boolean secondaryEncoderGood = true;
	//private boolean tertiaryEncoderGood = true;
	//private double encoderTolerance = 4;//degrees
	
	private double getCurrentArmAngle(){
		double angle;
		if(!primaryEncoder.isConnected()) primaryEncoderGood = false;
		else primaryEncoderGood = true;
		if(!secondaryEncoder.isConnected()) secondaryEncoderGood = false;
		else secondaryEncoderGood = true;
		
		if(primaryEncoderGood) angle = primaryEncoder.getAngle();
		else if (secondaryEncoderGood) angle = secondaryEncoder.getAngle();
		else{
			angle = 0;
		}
		return angle;
	}
	
	private void refreshCurrentCoM(){
		//y = 9.25
		//x = 34.5
		if(CIAObjects.cubeIntake.wristHalfExtended() && getCurrentArmAngle() > 90.0){
			currentCoMDist = 35.71;
			currentCoMAngleOffset = 15.0;
		}
		else{
			currentCoMDist = 22.4;
			currentCoMAngleOffset = 10;
		}
	}
	
	public ArmState getSystemState(){
		return systemState;
	}
	
	private double armAngleOffset = 0;
	
	public void adjustArmAngle(double speed){
		if(speed > .1 || speed < -.1) armAngleOffset += speed*-2.5;
		if(armAngleOffset > scaleAngle - frontScaleAngle) armAngleOffset = scaleAngle - frontScaleAngle;
	}
	
	boolean overridden = false;
	boolean lastOverride = false;
	
	public void override(boolean override){
		if(override){
			if(!lastOverride){// Only toggle once every button press. 
				overridden = !overridden;// Reverse the Pivot State
				if(overridden) System.out.println("The driver has overriden arm control.");
				else System.out.println("The operator has regained arm control");
			}
		}
		lastOverride = override;
	}
	
	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("Arm Angle", getCurrentArmAngle());
		SmartDashboard.putBoolean("Primary Encoder Connected", primaryEncoder.isConnected());
		SmartDashboard.putBoolean("Secondary Encoder Connected", secondaryEncoder.isConnected());
		SmartDashboard.putNumber("RawPrimaryVoltage", primaryEncoder.getRawAngle());
		SmartDashboard.putNumber("RawSecondaryVoltage", secondaryEncoder.getRawAngle());
		//SmartDashboard.putNumber("tertiaryEncoder", (tertiaryEncoder.get()*CIAConstants.tertiaryEncoderAngleConversion));
		//SmartDashboard.putNumber("suggested", calcHoldPosTorque(currentArmAngle));
	}
	
	public void updateSensors(){
		
	}

	@Override
	public void stop() {
		
	}

}
