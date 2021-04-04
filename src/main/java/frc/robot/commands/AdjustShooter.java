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
  public double hoodSetpoint;
  public final double setpointIncrement = 0.01;

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
    m_flywheel.enableFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    SmartDashboard.putString("We are at line", "60");
    if (result.hasTargets()) {
      SmartDashboard.putString("Oh geez", "we have reached the inside of the if");
      PhotonTrackedTarget target = result.getBestTarget();
      double distance = 
        PhotonUtils.calculateDistanceToTargetMeters(
          Constants.Camera.CAMERA_HEIGHT_METERS,
          Constants.Camera.TARGET_HEIGHT_METERS,
          Constants.Camera.CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(result.getBestTarget().getPitch())
        );

      InterpolatingDouble id_distance = new InterpolatingDouble(distance);
      //double hoodSetpoint = Constants.Hood.angleTreeMap.get(id_distance).value;
      //double flywheelSetpoint = Constants.Flywheel.rpmTreeMap.get(id_distance).value;
      
      m_hood.setSetpoint(hoodSetpoint);
    }
    double flywheelSetpoint = 5100;
    m_flywheel.setSetpoint(flywheelSetpoint);
    SmartDashboard.putNumber("Hood setpoint", hoodSetpoint);
    m_hood.setSetpoint(hoodSetpoint);
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

  public void increaseSetpoint() {
    hoodSetpoint += setpointIncrement;
  }

  public void decreaseSetpoint() {
    hoodSetpoint -= setpointIncrement;
  }
}
