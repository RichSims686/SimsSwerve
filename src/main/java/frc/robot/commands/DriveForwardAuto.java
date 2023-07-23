package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.drive.Drive;

public class DriveForwardAuto extends SequentialCommandGroup {
  ChassisSpeeds speeds = new ChassisSpeeds(0.5, 0, 0);
  private static final double driveDuration = 3.0;

  /**
   * Creates a new DriveForwardAuto, which drives forward for three seconds
   */
  public DriveForwardAuto(Drive drive) {
    addCommands(
        new StartEndCommand(() -> drive.drivePercent(speeds), drive::stop, drive)
            .withTimeout(driveDuration));
  }
}
