package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagCameraIO {
    @AutoLog
    public static class AprilTagCameraIOInputs {
        public boolean isConnected;
        public Optional<Pose3d> visionPose;
        public double timestamp;
    }

    public default void updateInputs(AprilTagCameraIOInputs inputs) {}
}
