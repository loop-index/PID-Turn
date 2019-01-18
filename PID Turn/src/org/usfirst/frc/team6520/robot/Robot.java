/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6520.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot implements PIDOutput {
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	Spark left = new Spark(0);
	Spark right = new Spark(1);
	
	public void turnPID(double angle, double Kp, double Ki, double Kd){
    	gyro.reset();
    	PIDController turnController = new PIDController(Kp, Ki, Kd, gyro, this);
    	
		turnController.setInputRange(0, 360);
		turnController.setOutputRange(-0.4, 0.4);
		turnController.setAbsoluteTolerance(2);
		turnController.setContinuous(true);
		turnController.reset();
		turnController.setSetpoint(angle);
		turnController.enable();
		
		while (!turnController.onTarget()){
			left.set(turnController.get());
			right.set(turnController.get());
			SmartDashboard.putNumber("heading", gyro.getAngle());
		}
		
		left.stopMotor();
		right.stopMotor();
		SmartDashboard.putNumber("heading", gyro.getAngle());
    }
	
	@Override
	public void teleopInit() {
		turnPID(90, 0.013, 0, 0.027);
	}

	@Override
	public void pidWrite(double output) {
	}
}
