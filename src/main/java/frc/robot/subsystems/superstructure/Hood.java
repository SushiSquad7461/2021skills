/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Hood extends PIDSubsystem {
    private CANSparkMax hoodMain;
    private CANEncoder hoodEncoder;

    public Hood() {
        super(new PIDController(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD));

        this.hoodMain = new CANSparkMax(Constants.Hood.MOTOR_ID, Constants.Hood.MOTOR_TYPE);
        hoodMain.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.hoodEncoder = this.hoodMain.getEncoder();
        this.zeroHood();
        this.setSetpoint(Constants.Hood.SETPOINT);

    }

    @Override
    public void useOutput(double output, double setpoint) {
        this.hoodMain.set(output);
        SmartDashboard.putNumber("output", output);
        SmartDashboard.putNumber("bruh position", hoodEncoder.getPosition());
        SmartDashboard.putNumber("bruh set", this.getSetpoint() );
    }

    @Override
    public double getMeasurement() {
        return this.hoodEncoder.getPosition();
    }

    public void zeroHood() {
        this.hoodEncoder.setPosition(0.0);
    }

}
