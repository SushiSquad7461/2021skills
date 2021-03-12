package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.util.List;

public class Paths {
	private final RobotContainer m_container;
	private TrajectoryConfig config;
	public Trajectory example = null;
	public Paths(RobotContainer container, DifferentialDriveVoltageConstraint voltageConstraint) {
		m_container = container;
		config =
				new TrajectoryConfig(Constants.Ramsete.MAX_METERS_PER_SECOND,
						Constants.Ramsete.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
				.setKinematics(container.s_drive.driveKinematics)
				.addConstraint(voltageConstraint);

		example = TrajectoryGenerator.generateTrajectory(
				new Pose2d(0, 0, new Rotation2d(0)),
				List.of(

				),
				new Pose2d(3, 0, new Rotation2d(0)),
				config
		);
	}
}
