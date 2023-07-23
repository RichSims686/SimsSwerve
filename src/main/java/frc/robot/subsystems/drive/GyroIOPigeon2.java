// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(Constants.CANDevices.pigeonCanID, Constants.CANDevices.driveCanBusName);
  
  public GyroIOPigeon2() {
    var config = new Pigeon2Configuration(); 
    // change factory defaults here
    config.MountPose.MountPoseYaw = 0;    // pigeon2 oriented with x forward, y left, z up
    config.MountPose.MountPosePitch = 0;
    config.MountPose.MountPoseRoll = 0;
    pigeon.getConfigurator().apply(config);

    // set signals to an appropriate rate
    pigeon.getYaw().setUpdateFrequency(Constants.loopFrequencyHz);

    // TODO: Phoenix6 replacement for Phoenix5s zeroGyroBiasNow?
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPositionRad =     Units.degreesToRadians( pigeon.getYaw().getValue());    // ccw+
    inputs.pitchPositionRad =   Units.degreesToRadians(-pigeon.getPitch().getValue());  // up+
    inputs.rollPositionRad =    Units.degreesToRadians(-pigeon.getRoll().getValue());   // ccw+
    // TODO: confirm Pigeon2 XYZ orientation    
    inputs.yawVelocityRadPerSec =   Units.degreesToRadians( pigeon.getAngularVelocityZ().getValue());   // ccw+
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-pigeon.getAngularVelocityY().getValue());   // up+
    inputs.rollVelocityRadPerSec =  Units.degreesToRadians(-pigeon.getAngularVelocityX().getValue());   // ccw+

    inputs.connected = pigeon.getYaw().getError().isOK();
  }
}
