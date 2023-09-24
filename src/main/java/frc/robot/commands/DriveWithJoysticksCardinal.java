// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.CardinalDirection;
import frc.robot.subsystems.drive.SwerveJoysticks.ProcessedJoysticks;

public class DriveWithJoysticksCardinal extends CommandBase {

    private final Drive drive;
    private final Supplier<ProcessedJoysticks> joystickSupplier;
    private final Supplier<Optional<CardinalDirection>> cardinalDirectionSupplier; // rotation
    private final BooleanSupplier precisionSupplier; // slow-down for precision positioning

    private double desiredHeadingRadians;
    private final double headingKp = 4 / DriveConstants.maxTurnRateRadiansPerSec;
    private final double headingKd = 0;
    private final double headingKi = 0;
    private final double headingTolerance = Units.degreesToRadians(5.0);
    private final PIDController headingPID;

    /** Creates a new DriveWithJoysticks. */
    public DriveWithJoysticksCardinal(Drive drive, Supplier<ProcessedJoysticks> joystickSupplier,
            Supplier<Optional<CardinalDirection>> cardinalDirectionSupplier,
            BooleanSupplier precisionSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.joystickSupplier = joystickSupplier;
        this.cardinalDirectionSupplier = cardinalDirectionSupplier;
        this.precisionSupplier = precisionSupplier;

        headingPID = new PIDController(headingKp, headingKd, headingKi);
        headingPID.enableContinuousInput(-Math.PI, Math.PI); // since gyro angle is not limited to [-pi, pi]
        headingPID.setTolerance(headingTolerance);
    }

    @Override
    public void initialize() {
        desiredHeadingRadians = drive.getPose().getRotation().getRadians();
    }

    @Override
    public void execute() {

        // note that processedJoysticks.getTurn() is ignored, overridden by cardinalInput
        // also, field relative controls are always enabled

        ProcessedJoysticks processedJoysticks = joystickSupplier.get();
        boolean precision = precisionSupplier.getAsBoolean();
         
        // update desired direction
        Optional<CardinalDirection> cardinalInput = cardinalDirectionSupplier.get();
        if (cardinalInput.isPresent()) {
            desiredHeadingRadians = cardinalInput.get().getAngleRadians();
        }

        // PID control of turn
        double turnInput = headingPID.calculate(drive.getPose().getRotation().getRadians(), desiredHeadingRadians);
        turnInput = headingPID.atSetpoint() ? 0 : turnInput;
        turnInput = MathUtil.clamp(turnInput, -1.0, +1.0);

        // Convert to meters/sec and radians/sec
        double vxMetersPerSecond = processedJoysticks.getX() * drive.getMaxLinearSpeedMetersPerSec();
        double vyMetersPerSecond = processedJoysticks.getY() * drive.getMaxLinearSpeedMetersPerSec();
        double omegaRadiansPerSecond = turnInput * drive.getMaxAngularSpeedRadiansPerSec();

        if (precision) {
            vxMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            vyMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            omegaRadiansPerSecond *= DriveConstants.precisionTurnMulitiplier;
        }

        // robot relative controls
        ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

        // field relative controls
        var driveRotation = drive.getRotation(); // angle from alliance wall normal
        if (DriverStation.getAlliance() == Alliance.Red) {
            driveRotation = driveRotation.rotateBy(new Rotation2d(Math.PI));
        }
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driveRotation);

        drive.driveVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
