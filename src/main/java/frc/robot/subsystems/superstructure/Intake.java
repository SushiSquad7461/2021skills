/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    // fields
    private final WPI_TalonSRX intakeMotor;
    public DoubleSolenoid intakeSolenoid;

    public Intake() {
        // instantiate and configure motors
        intakeMotor = new WPI_TalonSRX(Constants.Intake.MOTOR_ID);
        intakeMotor.setSafetyEnabled(false);

        // instantiate and configure solenoid
        //intakeSolenoid = new DoubleSolenoid(Constants.Intake.SOLENOID_IN, Constants.Intake.SOLENOID_OUT);
    }

    // begin intaking
    public void startVore() {
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.MAX_SPEED);
        //intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    // reverse intake movement
    public void unVore() {
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.MAX_SPEED);
        //intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    // stop intake movement
    public void stopVore() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
        //intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {

    }
}
