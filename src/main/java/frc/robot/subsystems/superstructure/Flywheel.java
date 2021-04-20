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
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel;

public class Flywheel extends SubsystemBase {

    // fields
    private final CANSparkMax flywheelMain;
    private final CANSparkMax flywheelSecondary;
    private final SimpleMotorFeedforward feedforward;
    private final ProfiledPIDController controller;
    private boolean enabled = false;
    private double last = 0;

    public Flywheel() {
        controller = new ProfiledPIDController(
            Constants.Flywheel.kP,
            Constants.Flywheel.kI,
            Constants.Flywheel.kD,
            new Constraints(
                Constants.Flywheel.MAX_ACCELERATION,
                Constants.Flywheel.MAX_JERK // I max jerked your mom
            ));

        // instantiate and configure motors
        flywheelMain = new CANSparkMax(Constants.Flywheel.MAIN_ID, Constants.Flywheel.MOTOR_TYPE);
        flywheelSecondary = new CANSparkMax(Constants.Flywheel.SECONDARY_ID, Constants.Flywheel.MOTOR_TYPE);
        feedforward = new SimpleMotorFeedforward(
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
        controller.setTolerance(0, Constants.Flywheel.ERROR_TOLERANCE);
        
    }

    @Override
    public void periodic() {
        //this.getController().setSetpoint(Constants.Flywheel.SPEED);
        SmartDashboard.putNumber("flywheel rpm", this.getMeasurement()); // rpm
        SmartDashboard.putNumber("flywheel rps", this.getMeasurement()/60); 
        double velocitySet = controller.getSetpoint().position;
        double accelerationSet = controller.getSetpoint().velocity;
        SmartDashboard.putNumber("flywheel profile setpoint pos", velocitySet);
        SmartDashboard.putNumber("flywheel profile setpoint vel", accelerationSet);
        double velocity = flywheelMain.getEncoder().getVelocity() / 60; //rps
        double acceleration = velocity - last;
        last = velocity;
        SmartDashboard.putNumber("flywheel accel", acceleration);
        
        double controllerVal = controller.calculate(velocity);
        /*
         * GUESS WHAT BITCH
         * IF YOU'RE READING THIS YOU'RE PROBABLY RE-WRITING FLYWHEEL CODE OR 
         * SOMETHING
         * 
         * REMEMBER TO DIVIDE FEEDFORWARD BY 12 HERE SO IT'S IN VOLTS 
        */
        double feedforwardVal = feedforward.calculate(velocitySet) / 12.0;
        SmartDashboard.putNumber("controller val", controllerVal);
        SmartDashboard.putNumber("feedforward val", feedforwardVal);
        if (enabled) {
            flywheelMain.set(controllerVal + feedforwardVal);
        } else {
            flywheelMain.set(0);
        }
    }
    // return current flywheel speed in RPM
    protected double getMeasurement() {
        return flywheelMain.getEncoder().getVelocity();
    }
    
    public void setGoal(double goal) {
        SmartDashboard.putNumber("flywheel goal", goal);
        controller.setGoal(goal);
        if (!enabled) {
            this.enabled = true;
        }
    }
    
    public void stop() {
        this.enabled = false;
    }
    
    public boolean isEnabled() {
        return this.enabled;
    }
    
}
