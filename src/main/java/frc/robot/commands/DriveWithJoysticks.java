// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveJoysticks.ProcessedJoysticks;

public class DriveWithJoysticks extends CommandBase {

    private final Drive drive;
    private final Supplier<ProcessedJoysticks> joystickSupplier;
    private final BooleanSupplier fieldRelativeSupplier;    // field-relative instead of robot-relative
    private final BooleanSupplier precisionSupplier;        // slower speeds for precision tasks

    /** Creates a new DriveWithJoysticks. */
    public DriveWithJoysticks(Drive drive, Supplier<ProcessedJoysticks> joystickSupplier, 
            BooleanSupplier fieldRelativeSupplier, BooleanSupplier precisionSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.joystickSupplier = joystickSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.precisionSupplier = precisionSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // process joystick inputs
        ProcessedJoysticks processedJoysticks = joystickSupplier.get();
        boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();
        boolean precision = precisionSupplier.getAsBoolean();

        // Convert to meters/sec and radians/sec
        double vxMetersPerSecond = processedJoysticks.getX() * drive.getMaxLinearSpeedMetersPerSec();
        double vyMetersPerSecond = processedJoysticks.getY() * drive.getMaxLinearSpeedMetersPerSec();
        double omegaRadiansPerSecond = processedJoysticks.getTurn() * drive.getMaxAngularSpeedRadiansPerSec();

        if (precision) {
            vxMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            vyMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            omegaRadiansPerSecond *= DriveConstants.precisionTurnMulitiplier;
        }

        // robot relative controls
        ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

        if (fieldRelative) {
            // field relative controls
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
