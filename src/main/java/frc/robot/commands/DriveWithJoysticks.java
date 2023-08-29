// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveJoystickInputs;

public class DriveWithJoysticks extends CommandBase {

  private final Drive drive;
  private final DoubleSupplier xSupplier; // x-axis translation
  private final DoubleSupplier ySupplier; // y-axis translation
  private final DoubleSupplier turnSupplier; // rotation
  private final BooleanSupplier precisionSupplier; // slow-down for precision positioning
  private final BooleanSupplier robotRelativeOverride; // robot-relative instead of field-relative

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier turnSupplier,
                            BooleanSupplier robotRelativeOverride, BooleanSupplier precisionSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;
    this.robotRelativeOverride = robotRelativeOverride;
    this.precisionSupplier = precisionSupplier;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    boolean squareInputs = true;

    // process joystick inputs
    SwerveJoystickInputs inputs = new SwerveJoystickInputs(xSupplier.getAsDouble(), 
                                               ySupplier.getAsDouble(),
                                               turnSupplier.getAsDouble(),
                                               squareInputs,
                                               squareInputs,
                                               precisionSupplier.getAsBoolean());

    // Convert to meters/sec and radians/sec
    double vxMetersPerSecond = inputs.getX() * drive.getMaxLinearSpeedMetersPerSec();
    double vyMetersPerSecond = inputs.getY() * drive.getMaxLinearSpeedMetersPerSec();
    double omegaRadiansPerSecond = inputs.getTurn() * drive.getMaxAngularSpeedRadiansPerSec();

    // field relative controls
    ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

    if (robotRelativeOverride.getAsBoolean()) {
      // robot relative controls
      var driveRotation = drive.getRotation(); // angle from alliance wall normal
      if (DriverStation.getAlliance() == Alliance.Red) {
        driveRotation = driveRotation.rotateBy(new Rotation2d(Math.PI));
      }
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driveRotation);
    }

    drive.driveVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }


}
