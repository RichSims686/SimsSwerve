// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.trajectory.Waypoint;

public class AutoCommands {
  // Subsystems
  private final Drive drive;

  // Constants

  // Waypoints

  public AutoCommands(
      Drive drive) {
    this.drive = drive;
  }

  /** Reset the odometry to the specified pose. */
  private Command reset(Pose2d pose) {
    return runOnce(() -> drive.setPose(AllianceFlipUtil.apply(pose)));
  }

  /** Returns a waypoint for a holonomic pose. */
  private Waypoint holonomic(Pose2d pose) {
    return Waypoint.fromHolonomicPose(pose);
  }

  /** Drives along the specified trajectory. */
  private Command path(Waypoint... waypoints) {
    return new DriveTrajectory(drive, Arrays.asList(waypoints), List.of());
  }

  /** Drives along the specified trajectory. */
  private Command path(List<TrajectoryConstraint> constraints, Waypoint... waypoints) {
    return new DriveTrajectory(drive, Arrays.asList(waypoints), constraints);
  }
}
