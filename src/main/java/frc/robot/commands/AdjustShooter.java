/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Hood;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team254.lib.util.InterpolatingDouble;

import org.photonvision.*;

/**
 * An example command that uses an example subsystem.
 */
public class AdjustShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Hood m_hood;
  private final Flywheel m_flywheel;
  private final PhotonCamera m_camera;
  public double hoodSetpoint = Constants.Hood.INITIAL_SETPOINT;
  public double flywheelSetpoint = 60;
  public final double setpointIncrement = 1;
  private int setpointIndex = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AdjustShooter(Hood hood, Flywheel flywheel, PhotonCamera camera) {
    m_hood = hood;
    m_flywheel = flywheel;
    m_camera = camera;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);
    addRequirements(m_flywheel);
    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flywheel.setGoal(flywheelSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //refreshSetpoints();
    SmartDashboard.putNumber("Hood setpoint", hoodSetpoint);
    SmartDashboard.putNumber("Flywheel setpoint right here", flywheelSetpoint);
    double setpoint = getHoodSetpointFromCamera();
    if (setpoint != -1) {
      m_hood.setSetpoint(setpoint);
    }
    m_flywheel.setGoal(flywheelSetpoint);
    //m_flywheel.setGoal(0);
    SmartDashboard.putNumber("Current zone", setpointIndex);
  }
  
  // Returns -1 if it doesn't see the target
  public double getHoodSetpointFromCamera() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      InterpolatingDouble pitch = new InterpolatingDouble(target.getPitch());
      SmartDashboard.putNumber("Camera pitch", pitch.value);
      InterpolatingDouble distance = solveForDistance(pitch.value);
      SmartDashboard.putNumber("Interpolated distance", distance.value);
      double setpoint = Constants.Hood.hoodAngleTreeMap.getInterpolated(distance).value;
      SmartDashboard.putNumber("Interpolated setpoint", setpoint);
      return setpoint;
    } else {
      return -1;
    }
  }

  public InterpolatingDouble solveForDistance(double pitch) {
    double cot = 1.0/Math.tan(pitch + Constants.Camera.CAMERA_PITCH_RADIANS);
    double diff = Constants.Camera.TARGET_HEIGHT_METERS - Constants.Camera.CAMERA_HEIGHT_METERS;
    return new InterpolatingDouble(cot * diff);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void cycleSetpoints() { 
    if (hoodSetpoint < Constants.Hood.INITIAL_SETPOINT) {
      hoodSetpoint += 1.0;
    }
    /*setpointIndex++;
    if (setpointIndex >= Constants.Hood.ZONE_SETPOINTS.length) {
      setpointIndex = 0;
    }*/
  }
  public void unCycleSetpoints() { 
    if (hoodSetpoint > 0) {
      hoodSetpoint -= 1.0;
    }
    /*setpointIndex--;
    if (setpointIndex < 0) {
      setpointIndex = Constants.Hood.ZONE_SETPOINTS.length-1;
    }*/
  }
  private void refreshSetpoints() {
    hoodSetpoint = Constants.Hood.ZONE_SETPOINTS[setpointIndex];
    flywheelSetpoint = Constants.Flywheel.ZONE_SETPOINTS[setpointIndex];
  }
}
