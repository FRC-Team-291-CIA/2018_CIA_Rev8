package org.usfirst.frc.team291.auto;

import org.usfirst.frc.team291.robot.CIAConstants;

import edu.wpi.first.wpilibj.Timer;

public class TestSequence extends AutoMode {
	
	/* TestSequence is called in testPeriodic in order to easily
	 * run diagnostics on the robot's motors and sensors.
	 * It will:
	 * 1. Run each drive motor individually and use the encoders to make sure each runs.
	 * 		-if the encoder doesn't read for all motors the encoder is bad.
	 * 		-if the encoder doesn't read for one motor that motor is bad.
	 * 2. Test both arm motors individually and check all three encoders.
	 * 3. Test Intake roller
	 * 4. Make sure both joysticks are plugged into the correct ports and set to the right mode.
	 * 4. Test that the gyro is connected and sample the drift rate and test ultrasonic sensor.
	 * 5. If anything is not perfect, report it to the Drivers Station and recommend how to fix it.*/
	
	Timer t;
	private int stage = 0;
	private boolean write = true;// Used to print status only once per stage.
	
	private boolean leftEncoderReads, rightEncoderReads, leftEncoderGood, rightEncoderGood,
					leftDriveAGood, leftDriveBGood, leftDriveCGood, 
					rightDriveAGood, rightDriveBGood, rightDriveCGood, gyroConnected = false;
	
	private double gyroDriftRate = 0;
	private double acceptableDriftRate = 0.5;
	private double acceptableEncoderDiff = .25;
	
	private double armEncoderTolerance = 4;
	private boolean primaryEncoderGood, secondaryEncoderGood, primaryEncoderInTolerance, secondaryEncoderInTolerance;
	
	public TestSequence(){
		t = new Timer();
		t.start();
		init();
	}

	@Override
	public void init() {
		System.out.println("\n\n---------------------------------------------------------------------");
		System.out.println("Starting test sequence...");
		System.out.println("It is recommended to put the robot up on jacks.\n");
		driveBase.zeroSensors();
	}

	@Override
	public void execute() {
		switch(stage){
		case 0:
			if(t.get() < .25){
				if(write){
					System.out.println("Testing drive motors " + CIAConstants.leftDrivePortA + " and " + CIAConstants.rightDrivePortA + "...");
					write = false;
				}
				driveBase.setleftDriveA(.4);
				driveBase.setrightDriveA(.4);
				
				
				if(driveBase.leftEncoderRate() > 0.05) {
					leftEncoderReads = leftDriveAGood = true;
				}
				if(driveBase.rightEncoderRate() > 0.05) {
					rightEncoderReads = rightDriveAGood = true;
				}
			}
			else{
				if(rightEncoderReads && leftEncoderReads){
					double encoderDiff = driveBase.rightEncoderDistance() - driveBase.leftEncoderDistance();
					if (Math.abs(encoderDiff) < acceptableEncoderDiff) leftEncoderGood = rightEncoderGood = true;
					else if(encoderDiff >= acceptableEncoderDiff) rightEncoderGood = true;
					else leftEncoderGood = true;
				}
				driveBase.setLeftRightPower(0, 0);
				driveBase.zeroSensors();
				write = true;
				t.reset();
				stage++;
			}
			break;
			
		case 1: // Pause
			if(t.get() > .4){
				t.reset();
				stage++;
			}
			break;
			
		case 2:
			if(t.get() < .25){
				if(write){
					System.out.println("Testing drive motors " + CIAConstants.leftDrivePortB + " and " + CIAConstants.rightDrivePortB + "...");
					write = false;
				}
				driveBase.setleftDriveB(-.4);
				driveBase.setrightDriveB(-.4);
				
				if(driveBase.leftEncoderRate() < -0.05) leftEncoderReads = leftDriveBGood = true;
				if(driveBase.rightEncoderRate() < -0.05) rightEncoderReads = rightDriveBGood = true;
			}
			else{
				if(rightEncoderReads && leftEncoderReads){
					double encoderDiff = driveBase.rightEncoderDistance() - driveBase.leftEncoderDistance();
					if (Math.abs(encoderDiff) < acceptableEncoderDiff) leftEncoderGood = rightEncoderGood = true;
					else if(encoderDiff >= acceptableEncoderDiff) rightEncoderGood = true;
					else leftEncoderGood = true;
				}
				driveBase.setLeftRightPower(0, 0);
				driveBase.zeroSensors();
				write = true;
				t.reset();
				stage++;
			}
			
			break;
			
		case 3: // Pause
			if(t.get() > .6){
				t.reset();
				stage++;
			}
			break;
			
		case 4:
			stage++;
			break;
			
		case 5:
			if(t.get() < .5){
				if(write){
					System.out.println("Testing Intake Roller...");
					write = false;
				}
				//cubeIntake.setRollerPower(.4);
			}
			else{
				//cubeIntake.setRollerPower(0);
				write = true;
				t.reset();
				stage++;
			}
			break;
			
		case 6:
			if(t.get() < .6){
				if(write){
					System.out.println("Testing Arm Motor, port " + CIAConstants.armMotorA + "...");
					write = false;
				}
				cubeArm.setArmMotorA(.5);
			}
			else{
				cubeArm.setArmMotorA(0);
				write = true;
				t.reset();
				stage++;
			}
			break;
		
			
		case 7: // Pause
			if(t.get() > 1){
				t.reset();
				stage++;
			}
			break;
			
		case 8:
			if(t.get() < .5){
				if(write){
					System.out.println("Testing Arm Motor, port " + CIAConstants.armMotorB + "...\n");
					write = false;
				}
				cubeArm.setArmMotorB(.5);
			}
			else{ 
				cubeArm.setArmMotorB(0);
				write = true;
				t.reset();
				stage++;
			}
			break;
		
		case 9: // Pause
			if(t.get() > .6){
				t.reset();
				stage++;
			}
			break;

		case 10:
			if(cubeArm.primaryEncoder.isConnected() && cubeArm.secondaryEncoder.isConnected()){
				if(Math.abs(cubeArm.primaryEncoder.getAngle() - cubeArm.secondaryEncoder.getAngle()) < armEncoderTolerance){
					if(Math.abs(cubeArm.primaryEncoder.getAngle() - CIAConstants.stowedArmAngle) < armEncoderTolerance){
						primaryEncoderGood = secondaryEncoderGood = primaryEncoderInTolerance = secondaryEncoderInTolerance = true;
						System.out.println("Arm Encoders Are Reading Correctly.");
					}
					else System.out.println("The pivot shaft has slipped. Tighten set screw and recalibrate encoders.");	
				}
				else{
					if(!(Math.abs(cubeArm.primaryEncoder.getAngle() - CIAConstants.stowedArmAngle) < armEncoderTolerance)){
						System.out.println("Primary Encoder needs recalibrated. Check for slip.");
						System.out.println("Recomended Zero Offset: " + cubeArm.primaryEncoder.getRawAngle());
					}
					else System.out.println("Primary Encoder is reading correctly");

					if(!(Math.abs(cubeArm.secondaryEncoder.getAngle() - 15) < armEncoderTolerance)){
						System.out.println("Secondary Encoder needs recalibrated. Check for slip");
						System.out.println("Recomended Zero Offset: " + (cubeArm.secondaryEncoder.getRawAngle() + 2.0*CIAConstants.stowedArmAngle));
					}
					else System.out.println("Secondary Encoder is reading correctly");
				}
			}
			else{
				if(!cubeArm.primaryEncoder.isConnected()) System.out.println("Primary Encoder is not connected!");
				if(!cubeArm.secondaryEncoder.isConnected()) System.out.println("Secondary Encoder is not connected");
			}
			stage++;
			break;

		case 11:
			System.out.println("\nDiagnostic sequence complete.\n");
			if(leftEncoderGood && rightEncoderGood && leftDriveAGood && leftDriveBGood && leftDriveCGood && rightDriveAGood && 
					rightDriveBGood && rightDriveCGood && driveBase.gyroIsConnected()){
				System.out.println("The rest of the Robot is in perfect working order.");
			}
			else if(!leftEncoderGood && !rightEncoderGood && !leftDriveAGood && !leftDriveBGood && !leftDriveCGood && !rightDriveAGood && 
					!rightDriveBGood && !rightDriveCGood && !gyroConnected){
						System.out.println("The robot is completely broken. Nothing works.");
						System.out.println("It is recommended that you crawl into a corner, give up and blame Mr. Fleming");
					}
			else{
				if(!leftEncoderGood){
					System.out.println("Left Encoder is not reading as it should.");
					if(!leftEncoderReads) System.out.println("It is reading zero. Check wiring for loose connections.");
					else System.out.println("It still works, but it reads significantly less than the right encoder.");
				}

				if(!rightEncoderGood){
					System.out.println("Right Encoder is not reading as it should.");
					if(!rightEncoderReads) System.out.println("It is reading zero. Check wiring for loose connections.");
					else System.out.println("It still works, but it reads significantly less than the left encoder.");
				}
				if(leftEncoderReads){
					if(!leftDriveAGood) System.out.println("Left Motor at port " + CIAConstants.leftDrivePortA + " is not working!!!!");
					if(!leftDriveBGood) System.out.println("Left Motor at port " + CIAConstants.leftDrivePortB + " is not working!!!!");
				}
				if(rightEncoderReads){
					if(!rightDriveAGood) System.out.println("Right Motor at port " + CIAConstants.rightDrivePortA + " is not working!!!!");
					if(!rightDriveBGood) System.out.println("Right Motor at port " + CIAConstants.rightDrivePortB + " is not working!!!!");
				}
				
				if(!driveBase.gyroIsConnected()) System.out.println("The Gyro is not Connected!");
			}
			System.out.println("---------------------------------------------------------------\n");
			stage++;
			write = true;
			break;
			
		case 12:
			break;
		}
	
			
	}

	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean isValid(boolean startOnLeft, boolean switchOnLeft, boolean scaleOnLeft) {
		return false;
	}
	
}
