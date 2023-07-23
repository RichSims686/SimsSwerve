// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveWithJoysticks extends CommandBase {

  private final Drive drive;
  private final Supplier<Double> xSupplier; // x-axis translation
  private final Supplier<Double> ySupplier; // y-axis translation
  private final Supplier<Double> turnSupplier; // rotation
  private final Supplier<Boolean> precisionSupplier; // slow-down for precision positioning
  private final Supplier<Boolean> robotRelativeOverride; // robot-relative instead of field-relative

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> turnSupplier,
                            Supplier<Boolean> robotRelativeOverride, Supplier<Boolean> precisionSupplier) {
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

    // Get values from double suppliers
    double xTranslationInput = processJoystickInputs(xSupplier.get(), squareInputs);
    double yTranslationInput = processJoystickInputs(ySupplier.get(), squareInputs);
    double turnInput = processJoystickInputs(turnSupplier.get(), squareInputs);

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(xTranslationInput, yTranslationInput);
    Rotation2d linearDirection = new Rotation2d(xTranslationInput, yTranslationInput);

    // Apply speed limits
    if (precisionSupplier.get()) {
      linearMagnitude *= DriveConstants.precisionLinearMultiplier;
      turnInput *= DriveConstants.precisionTurnMulitiplier;
    }

    // Calcaulate new linear components
    Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);

    // Convert to meters/sec and radians/sec
    double vxMetersPerSecond = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
    double vyMetersPerSecond = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();
    double omegaRadiansPerSecond = turnInput * drive.getMaxAngularSpeedRadPerSec();

    // field relative controls
    ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

    if (robotRelativeOverride.get()) {
      // robot relative controls
      var driveRotation = drive.getRotation(); // angle from alliance wall normal
      if (DriverStation.getAlliance() == Alliance.Red) {
        driveRotation = driveRotation.rotateBy(new Rotation2d(Math.PI));
      }
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driveRotation);
    }

    drive.driveVelocity(speeds);
  }


  private double processJoystickInputs(double value, boolean squareInputs) {
    double scaledValue = 0.0;
    double deadband = DriveConstants.driveJoystickDeadbandPercent;
    if (Math.abs(value) > deadband) {
      scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
      if (squareInputs) {
        scaledValue = Math.copySign(scaledValue * scaledValue, value);
      } else {
        scaledValue = Math.copySign(scaledValue, value);
      }
    }
    return scaledValue;
  }


  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
