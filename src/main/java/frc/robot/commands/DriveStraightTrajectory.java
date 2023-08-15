// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// hello

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.trajectory.Waypoint;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraightTrajectory extends SequentialCommandGroup {

  // public static double xDistanceMeters = Units.feetToMeters(5.0); 
  // public static double yDistanceMeters = Units.feetToMeters(5.0); 
  public static double xDistanceMeters = Units.feetToMeters(20.0); 
  public static double yDistanceMeters = Units.feetToMeters(0.0); 
  public static int numRepeat = 1;

  // public static Pose2d position1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  // public static Pose2d position2 = new Pose2d(xDistanceMeters, 0.0, Rotation2d.fromDegrees(-90));
  // public static Pose2d position3 = new Pose2d(xDistanceMeters, yDistanceMeters, Rotation2d.fromDegrees(180));
  // public static Pose2d position4 = new Pose2d(0.0, yDistanceMeters, Rotation2d.fromDegrees(90));
  public static Pose2d position1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static Pose2d position2 = new Pose2d(xDistanceMeters, 0.0, Rotation2d.fromDegrees(0));
  // public static Pose2d position3 = new Pose2d(xDistanceMeters, yDistanceMeters, Rotation2d.fromDegrees(0));
  // public static Pose2d position4 = new Pose2d(0.0, yDistanceMeters, Rotation2d.fromDegrees(0));

  public DriveStraightTrajectory(Drive drive) {

    for (int k=0; k<numRepeat; k++) {

        addCommands(
            new DriveTrajectory(drive,
            List.of(
                Waypoint.fromHolonomicPose(position1),
                Waypoint.fromHolonomicPose(position2))));
    }
  }
}
