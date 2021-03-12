/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

	// define variables
	private final WPI_VictorSPX intakeMotor;

	// constructor
	public Intake() {

		// instantiate motor controllers
		intakeMotor = new WPI_VictorSPX(Constants.Intake.MOTOR_ID);

		// configuration
		intakeMotor.setSafetyEnabled(false);

	}

	public void startVore(){
		intakeMotor.set(Constants.Intake.MAX_SPEED);
	}
	public void stopVore(){
		intakeMotor.set(0);
	}
	public void unVore() { intakeMotor.set(-Constants.Intake.MAX_SPEED); }

	@Override
	public void periodic() {

	}
}
