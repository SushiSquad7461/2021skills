/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import javax.naming.ldap.Control;

public class Hopper extends SubsystemBase {

    // fields
    private final WPI_TalonSRX hopperFast;
    private final WPI_VictorSPX hopperSlow;
    private final WPI_TalonSRX hopperFloor;
    private final WPI_TalonSRX kicker;
    public Hopper() {
        // instantiate motors
        hopperFast = new WPI_TalonSRX(Constants.Hopper.FAST_ID);
        hopperSlow = new WPI_VictorSPX(Constants.Hopper.SLOW_ID);
        hopperFloor = new WPI_TalonSRX(Constants.Hopper.FLOOR_ID);
        kicker = new WPI_TalonSRX(Constants.Hopper.KICKER_ID);
        kicker.configFactoryDefault();
        kicker.setInverted(true);
        // config the peak and the minimum outputs to tell if there was a spike
        // [-1,1] represents [-100%, 100%]
        hopperFast.configNominalOutputForward(0, Constants.Hopper.CONFIG_TIMEOUT);
        hopperFast.configNominalOutputReverse(0, Constants.Hopper.CONFIG_TIMEOUT);
        hopperFast.configPeakOutputForward(1, Constants.Hopper.CONFIG_TIMEOUT);
        hopperFast.configPeakOutputReverse(-1, Constants.Hopper.CONFIG_TIMEOUT);
        kicker.configPeakOutputForward(1, Constants.Hopper.CONFIG_TIMEOUT);
        kicker.configPeakOutputReverse(-1, Constants.Hopper.CONFIG_TIMEOUT);
        hopperFloor.setInverted(true);
        // sets the same configs to hopperSlow
        hopperSlow.follow(hopperFast);
    }

    // run hopper inward
    public void startSpit() {
        hopperFast.set(ControlMode.PercentOutput, Constants.Hopper.MAX_SPEED);
        hopperSlow.set(ControlMode.PercentOutput, Constants.Hopper.SLOW_SPEED);
        hopperFloor.set(ControlMode.PercentOutput, Constants.Hopper.FLOOR_SPEED);
        kicker.set(ControlMode.PercentOutput, -1.0);
        SmartDashboard.putNumber("Kicker applied output", kicker.getMotorOutputPercent());
        SmartDashboard.putBoolean("Hopper floor inverted", hopperFloor.getInverted());
    }

    // reverse for anti-jam
    public void reverseSpit() {
        hopperFast.set(ControlMode.PercentOutput, Constants.Hopper.REVERSE_SPEED);
        hopperSlow.set(ControlMode.PercentOutput, Constants.Hopper.REVERSE_SPEED);
    }

    // stop hopper movement
    public void endSpit() {
        hopperFast.set(ControlMode.PercentOutput, 0);
        hopperSlow.set(ControlMode.PercentOutput, 0);
        hopperFloor.set(ControlMode.PercentOutput, 0);
        kicker.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
    }
}