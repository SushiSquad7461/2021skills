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
import com.revrobotics.CANSparkMaxLowLevel;

public class Flywheel extends PIDSubsystem {

    // fields
    private final CANSparkMax flywheelMain;
    private final CANSparkMax flywheelSecondary;
    private final SimpleMotorFeedforward flywheelFeedforward;

    public Flywheel() {
        super(new PIDController(Constants.Flywheel.kP, Constants.Flywheel.kI, Constants.Flywheel.kD));

        // instantiate and configure motors
        flywheelMain = new CANSparkMax(Constants.Flywheel.MAIN_ID, Constants.Flywheel.MOTOR_TYPE);
        flywheelSecondary = new CANSparkMax(Constants.Flywheel.SECONDARY_ID, Constants.Flywheel.MOTOR_TYPE);
        flywheelFeedforward = new SimpleMotorFeedforward(
                Constants.Flywheel.kS,
                Constants.Flywheel.kV,
                Constants.Flywheel.kA
        );

        flywheelMain.restoreFactoryDefaults();
        flywheelMain.setSmartCurrentLimit(Constants.Flywheel.CURRENT_LIMIT);
        flywheelSecondary.restoreFactoryDefaults();
        flywheelSecondary.setSmartCurrentLimit(Constants.Flywheel.CURRENT_LIMIT);
        flywheelSecondary.restoreFactoryDefaults();
        flywheelMain.setInverted(Constants.Flywheel.MAIN_INVERTED);
        flywheelSecondary.setInverted(Constants.Flywheel.SECONDARY_INVERTED);
        flywheelSecondary.follow(flywheelMain, true);

        // config the peak and nominal outputs ([-1, 1] represents [-100, 100]%)
		/*flywheelMain.configNominalOutputForward(0, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configNominalOutputReverse(0, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configPeakOutputForward(1, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configPeakOutputReverse(-1, Constants.Flywheel.CONFIG_TIMEOUT); 
		pretty sure this doesn't exist either, although i'm not quite sure what it did anyways */

        // the first number here is a 0 for position tolerance, we want it to be zero
        this.getController().setTolerance(0, Constants.Flywheel.ERROR_TOLERANCE);
        
    }

    public void stop() {
        flywheelMain.set(0);
    }

    @Override
    public void periodic() {
        //this.getController().setSetpoint(Constants.Flywheel.SPEED);
        SmartDashboard.putNumber("flywheel rpm", this.getMeasurement()); // rpm
        SmartDashboard.putNumber("flywheel rps", this.getMeasurement()/60); // rpm
        SmartDashboard.putBoolean("flywheel at speed", isAtSpeed()); // revved up boolean
    }

    @Override
    protected void useOutput(double output, double setpoint) {
    }

    public void enableFlywheel() {
        double controlOutput = m_controller.calculate(this.getMeasurement() / 60, Constants.Flywheel.SPEED);
        double feedForward = flywheelFeedforward.calculate(Constants.Flywheel.SPEED) / 12; 
        double output = controlOutput + feedForward;
        flywheelMain.set(output);
        SmartDashboard.putNumber("Flywheel primary current", flywheelMain.getOutputCurrent());
        SmartDashboard.putNumber("Flywheel secondary current", flywheelSecondary.getOutputCurrent());
        //flywheelMain.set(0.05);
        SmartDashboard.putNumber("Flywheel feedforward", feedForward);
        SmartDashboard.putNumber("Flywheel control loop output", controlOutput);
        SmartDashboard.putNumber("Flywheel expected kP", controlOutput / m_controller.getPositionError());
        SmartDashboard.putNumber("Flywheel position error", m_controller.getPositionError());
    }

    // return current flywheel speed in RPM
    @Override
    protected double getMeasurement() {
        return flywheelMain.getEncoder().getVelocity();
    }

    // check if flywheel is at speed
    public boolean isAtSpeed() {
        return this.getMeasurement() >= Constants.Flywheel.SPEED - Constants.Flywheel.SPEED_TOLERANCE;
    }

}
