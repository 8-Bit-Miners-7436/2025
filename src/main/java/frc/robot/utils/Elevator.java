package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
  // Initialize Motors, Encoder, and Limit Switches
  private final SparkMax coralMotor = new SparkMax(16, MotorType.kBrushless);
  private final SparkMax elevatorMotor = new SparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final DigitalInput upperLimitSwitch = new DigitalInput(9);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(0);

  private Double targetRot = null;
  public void moveToLevel(String level) {
    switch (level) {
      case "BASE"   -> {setElevatorSpeed(-1); targetRot = 0.0;}                            //  A: Move to Bottom
      case "INTAKE" -> targetRot = (targetRot != null && targetRot == 11.0) ? 27.0 : 11.0; //  B: Move to Coral Intake
      case "L2"     -> targetRot = 50.0;                                                   //  X: Move to L2
      case "L3"     -> {setElevatorSpeed(1); targetRot = 192.0;}                           //  Y: Move to L3 (Top)
      case "DOWN"   -> {setElevatorSpeed(-0.5); targetRot = null;}                         // LB: Manually Move Down
      case "UP"     -> {setElevatorSpeed(0.5); targetRot = null;}                          // RB: Manually Move Up
      case "STOP"   -> {setElevatorSpeed(0); targetRot = null;}
    }
  }

  public void updateElevator() {
    if (targetRot != null) {
      double error = targetRot - elevatorEncoder.getPosition();
      double speed = Math.signum(error) * (1 / (1 + Math.exp(-Math.abs(error) / 10)));
      if (Math.abs(error) < 1) setElevatorSpeed(0);
      else setElevatorSpeed(MathUtil.clamp(speed, -1, 1));
    }
  }

  // Moves Elevator Only if The Respective Limit Switch is Not Pressed
  public void setElevatorSpeed(double speed) {
    boolean limitHit = (speed > 0 ? upperLimitSwitch.get() : lowerLimitSwitch.get());
    if (lowerLimitSwitch.get()) resetEncoder();
    Logger.recordOutput("elevator rots", elevatorEncoder.getPosition());
    if (limitHit || speed == 0) elevatorMotor.stopMotor();
    else elevatorMotor.set(speed);
  }

  // Spins Coral Launcher Only if Speed Above 0.1 to Prevent Missfire
  public void setCoralSpeed(double speed) {
    coralMotor.set(Math.abs(speed) > 0.1 ? speed : 0);
  }

  // Resets Encoder to Ensure Correct Height Detection
  public void resetEncoder() {
    elevatorEncoder.setPosition(0);
  }
}
