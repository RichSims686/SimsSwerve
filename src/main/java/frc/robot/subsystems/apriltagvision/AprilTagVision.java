package frc.robot.subsystems.apriltagvision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.GeomUtil;

public class AprilTagVision extends SubsystemBase {

    private final AprilTagCamera[] cameras;

    private static Optional<Transform3d> robotToTarget = Optional.empty();
    private static double lastDemoTagTimestamp = 0.0;
    private static final double demoTagPersistenceSeconds = 0.5;

    public AprilTagVision() {

        cameras = new AprilTagCamera[] {
            //DEBUG
            // new AprilTagCamera(VisionConstants.cameraNames[0], new AprilTagCameraIOPhotonVision(VisionConstants.cameraNames[0], VisionConstants.robotToCameras[0])),
            // new AprilTagCamera(VisionConstants.cameraNames[1], new AprilTagCameraIOPhotonVision(VisionConstants.cameraNames[1], VisionConstants.robotToCameras[1])),
            // new AprilTagCamera(VisionConstants.cameraNames[2], new AprilTagCameraIOPhotonVision(VisionConstants.cameraNames[2], VisionConstants.robotToCameras[2])),
            // new AprilTagCamera(VisionConstants.cameraNames[3], new AprilTagCameraIOPhotonVision(VisionConstants.cameraNames[3], VisionConstants.robotToCameras[3])),
            new AprilTagCamera(VisionConstants.cameraNames[4], new AprilTagCameraIOLimelight(VisionConstants.cameraNames[4], VisionConstants.robotToCameras[4]))
        };
    }

    @Override
    public void periodic() {
        for (var camera : cameras) {
            camera.periodic();
        }

        //DEBUG
        setDemoTagMode(true);
        Optional<Transform3d> robotToDemoTag = getRobotToDemoTag();
    }


    public void setDemoTagMode(boolean enable) {
        for (var camera : cameras) {
            camera.setVisionUpdatesEnabled(!enable);
        }
    }


    /** Get pose associated with Demo Tag.  If multiple cameras see the tag, pick the one with the lowest ambiguity */
    // TODO: maybe average the poses for the same tag?  weighted by ambiguity?
    public Optional<Transform3d> getRobotToDemoTag() {

        // clear robotToTarget if we haven't seen the tag in a while
        if (Timer.getFPGATimestamp() - lastDemoTagTimestamp > demoTagPersistenceSeconds) {
            robotToTarget = Optional.empty();
        }

        double minAmbiguity = Double.POSITIVE_INFINITY; 
        for (var camera : cameras) {
            for (var target : camera.getTargetList()) {
                if (target.getFiducialId() == Constants.VisionConstants.demoTagId 
                        && target.getAmbiguity() > -0.1 
                        && target.getAmbiguity() < minAmbiguity) {
                    minAmbiguity = target.getAmbiguity();
                    robotToTarget = Optional.of(camera.getRobotToCamera().plus(target.getCameraToTarget()));
                    lastDemoTagTimestamp = Timer.getFPGATimestamp();
                }
            }
        }

        if (robotToTarget.isPresent()) {
            Logger.getInstance().recordOutput("AprilTagVision/RobotToDemoTag", GeomUtil.transform3dToPose3d(robotToTarget.get()));
        } else {
            Logger.getInstance().recordOutput("AprilTagVision/RobotToDemoTag", new double[] {});
        }

        return robotToTarget;
    } 

}