package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagCameraIOLimelight implements AprilTagCameraIO {

    private final String cameraName; 

    public AprilTagCameraIOLimelight(String cameraName) {
        // Important: need to configure robotToCamera pose using Limelight webUI
        // Important: need to configure AprilTag field map using Limelight webUI
        // https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#robot-localization-botpose-and-megatag
        this.cameraName = cameraName;
        LimelightHelpers.setPipelineIndex(cameraName, 1);
    }

    public void updateInputs(AprilTagCameraIOInputs inputs) {
        // set default values
        inputs.isConnected = false;
        inputs.visionPose = Optional.empty();
        inputs.timestamp = Timer.getFPGATimestamp();

        // get parsed results from JSON on NetworkTables.  
        // Use this JSON results to make sure all values are from the same snapshot
        LimelightHelpers.Results result = LimelightHelpers.getLatestResults(cameraName).targetingResults;

        // TODO: figure out how to determine if Limelight is disconnected
        inputs.isConnected = true;
        if (!inputs.isConnected) 
            return;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            inputs.visionPose = Optional.of(result.getBotPose3d_wpiBlue());
        } else {
            inputs.visionPose = Optional.of(result.getBotPose3d_wpiRed());
        }
        double latencySeconds = (result.latency_capture + result.latency_pipeline + result.latency_jsonParse) / 1000.0;
        inputs.timestamp = Timer.getFPGATimestamp() - latencySeconds;
    }
}
