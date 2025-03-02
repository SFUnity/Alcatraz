package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;
import static frc.robot.util.SparkUtil.sparkConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbIOSparkMax implements ClimbIO {
  private final SparkMax climbMotor = new SparkMax(0, null);
  private final RelativeEncoder encoder = climbMotor.getEncoder();

  private final SparkMaxConfig config = sparkConfig(inverted, positionFactor);

  public ClimbIOSparkMax() {
    // TODO persist Parameters
    configureSpark(climbMotor, config, true);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.velocityRotsPerSec = encoder.getVelocity();
    inputs.appliedVolts = climbMotor.getAppliedOutput() * climbMotor.getBusVoltage();
    inputs.currentAmps = climbMotor.getOutputCurrent();
  }
}
