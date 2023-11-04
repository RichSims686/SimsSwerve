package frc.robot.subsystems.apriltagvision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.apriltagvision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.GeomUtil;

public class AprilTagCameraIOLimelight implements AprilTagCameraIO {

    private final String cameraName; 
    private final Transform3d robotToCamera;

    public AprilTagCameraIOLimelight(String cameraName, Transform3d robotToCamera) {
        // Important: need to configure robotToCamera pose using Limelight webUI
        // Important: need to configure AprilTag field map using Limelight webUI
        // https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#robot-localization-botpose-and-megatag
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        LimelightHelpers.setPipelineIndex(cameraName, 1);
    }

    public void updateInputs(AprilTagCameraIOInputs inputs) {
        // set default values
        inputs.robotToCamera = robotToCamera;
        inputs.isConnected = false;
        inputs.visionPose = Optional.empty();
        inputs.timestamp = Timer.getFPGATimestamp();
        inputs.cameraToTargets.clear();

        // get parsed results from JSON on NetworkTables.  
        // Use this JSON results to make sure all values are from the same snapshot
        LimelightHelpers.Results result = LimelightHelpers.getLatestResults(cameraName).targetingResults;

        // TODO: figure out how to determine if Limelight is disconnected
        inputs.isConnected = true;
        if (!inputs.isConnected) 
            return;

        if (!result.valid) {
            return;
        }

        // TODO: result.valid always returns true.  Use 'tv' from NetworkTables.  There is a small chance that the JSON and NT will be out of sync
        if (!LimelightHelpers.getTV(cameraName)) {
            return;
        }

        inputs.visionPose = Optional.ofNullable(getBotPose3d(result));
        double latencySeconds = (result.latency_capture + result.latency_pipeline + result.latency_jsonParse) / 1000.0;
        inputs.timestamp = Timer.getFPGATimestamp() - latencySeconds;


        // populate targets array
        double limelightAmbiguity = 1.0;    // limelight doesn't give ambiguity, so set this as a large number
        for (LimelightTarget_Fiducial target : result.targets_Fiducials) {
            Pose3d pose = convertLimelightEDNtoNWU(target.getTargetPose_CameraSpace());
            inputs.cameraToTargets.add(
                new AprilTagTarget(
                    (int)target.fiducialID, 
                    GeomUtil.pose3dToTransform3d(pose),
                    limelightAmbiguity));
        }
    }

    public Pose3d getBotPose3d(LimelightHelpers.Results result) {
        Pose3d botPose3d = null;

        //2024 if (DriverStation.getAlliance().isPresent()) {
        //2024     Alliance alliance = DriverStation.getAlliance().get();
            Alliance alliance = DriverStation.getAlliance();
            if (alliance == Alliance.Blue) {
                botPose3d = result.getBotPose3d_wpiBlue();
            } else {
                botPose3d = result.getBotPose3d_wpiRed();
            }
            botPose3d = convertLimelightEDNtoNWU(botPose3d);
        //2024 }
        return botPose3d;
    }

    // convert from EDN to NWU coordinate system
    public Pose3d convertLimelightEDNtoNWU(Pose3d llPose3d) {
        Translation3d Tll = llPose3d.getTranslation();
        Rotation3d Rll = llPose3d.getRotation();
        return new Pose3d(
            new Translation3d(Tll.getZ(), -Tll.getX(), -Tll.getY()),
            new Rotation3d(Rll.getZ(), -Rll.getX(), Rll.getY() - Math.PI/2.0));
    }
}
