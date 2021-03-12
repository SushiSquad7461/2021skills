/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class Flywheel extends PIDSubsystem {

	// define variables
	private final CANSparkMax flywheelMain;
	private final CANSparkMax flywheelSecondary;
	private final SimpleMotorFeedforward flywheelFeedforward;
	private final CANCoder encoderMain;

	public Flywheel() {
		super(new PIDController(Constants.Flywheel.kP, Constants.Flywheel.kI, Constants.Flywheel.kD));

		// instantiate motor and encoders
		flywheelMain = new CANSparkMax(Constants.Flywheel.MAIN_ID, brushless);
		flywheelSecondary = new CANSparkMax(Constants.Flywheel.SECONDARY_ID, brushless);

		flywheelFeedforward = new SimpleMotorFeedforward(
				Constants.Flywheel.kS,
				Constants.Flywheel.kV,
				Constants.Flywheel.kA
		);

		// configure motor controllers
		// flywheelMain.configFactoryDefault(); this does not exist for sparks as far as i can tell
		flywheelMain.setInverted(Constants.Flywheel.MAIN_INVERTED);
		flywheelSecondary.setInverted(Constants.Flywheel.SECONDARY_INVERTED);
		flywheelSecondary.follow(flywheelMain);

		// config the peak and nominal outputs ([-1, 1] represents [-100, 100]%)
		/*
		flywheelMain.configNominalOutputForward(0, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configNominalOutputReverse(0, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configPeakOutputForward(1, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configPeakOutputReverse(-1, Constants.Flywheel.CONFIG_TIMEOUT); 
		pretty sure this doesn't exist either, although i'm not quite sure what it did anyways */

		// the first number here is a 0 for position tolerance, we want
		// it to be zero
		this.getController().setTolerance(0, Constants.Flywheel.ERROR_TOLERANCE);
	}

	public void stop() {
		flywheelMain.set(ControlMode.Velocity, 0);
	}

	@Override
	public void periodic() {
		this.getController().setSetpoint(Constants.Flywheel.SPEED);

		// put rpm on dashboard
		SmartDashboard.putNumber("flywheel rpm", this.getMeasurement());

		// put revved up boolean on dashboard
		SmartDashboard.putBoolean("flywheel at speed", isAtSpeed());

		SmartDashboard.putNumber("flywheel rpm 2", this.getMeasurement());

		//RobotContainer.operatorController.setRumble(GenericHID.RumbleType.kRightRumble, Math.pow(encoderMain.getVelocity() / 12000, 3));
	}

	@Override
	protected void useOutput(double output, double setpoint) { }

	public void enableFlywheel() {
		double output = m_controller.calculate(this.getMeasurement(), Constants.Flywheel.SPEED);
		double feedForward = flywheelFeedforward.calculate(Constants.Flywheel.SPEED);

		flywheelMain.setVoltage(output + feedForward);
	}

	// return current flywheel speed
	@Override
	protected double getMeasurement() {
		return flywheelMain.getVelocity();
	}

	// check if flywheel is at speed
	public boolean isAtSpeed() {
		return this.getMeasurement() >= Constants.Flywheel.SPEED - Constants.Flywheel.SPEED_TOLERANCE;
	}

}
