// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(0);
    pigeon.configFactoryDefault();
    pigeon.zeroGyroBiasNow();
    pigeon.setYaw(0.0);
  }

  public void updateInputs(GyroIOInputs inputs) {
    pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDps);
    inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
    inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);     // cw+
    inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);   // up+
    inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);      // ccw+
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);   // cw+
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]); // up+
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);    // ccw+
  }
}
