// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class FollowAprilTag extends SequentialCommandGroup {
    private static final LoggedTunableNumber targetDistance = new LoggedTunableNumber("FollowAprilTag/TargetDistance",
            2.0);

    public FollowAprilTag(Drive drive, AprilTagVision aprilTagVision) {
        addCommands(
            Commands.runOnce(() -> aprilTagVision.setDemoTagMode(true))
                .andThen(            
                    new DriveToPose(
                        drive,
                        true,
                        () -> {
                            Pose3d robotPose = new Pose3d(drive.getPose());
                            Optional<Transform3d> robotToTag = aprilTagVision.getRobotToDemoTag();
                            if (robotToTag.isPresent()) {
                                Pose2d demoTagPose = robotPose.transformBy(robotToTag.get()).toPose2d();
                                Pose2d driveToPose = demoTagPose.transformBy(
                                    new Transform2d(
                                        new Translation2d(targetDistance.get(), 0.0), 
                                        new Rotation2d(Math.PI)));
                                Logger.getInstance().recordOutput("AprilTagVision/DemoTagPose", demoTagPose);
                                Logger.getInstance().recordOutput("AprilTagVision/DriveToPose", driveToPose);
                                return driveToPose;
                            } else {
                                // stay where you are if you don't see the demo tag
                                Logger.getInstance().recordOutput("AprilTagVision/DemoTagPose", new double[] {});
                                Logger.getInstance().recordOutput("AprilTagVision/DriveToPose", new double[] {});
                                return drive.getPose();
                            }
                        }
                    )
                )
                .finallyDo((boolean interrupted) -> aprilTagVision.setDemoTagMode(true)
            )
        );
    }
}