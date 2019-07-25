/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SwerveDrive extends Command {

  private double[] turnAngle = {0,0,0,0};
  private double[] drivePower = {0,0,0,0};

  private double[] error = {0,0,0,0};
  private double[] sumError = {0,0,0,0};
  private double[] diffError = {0,0,0,0};
  private double[] prevError = {0,0,0,0};
  private double[] turnPower = {0,0,0,0};

  private final double kP = 0.5;
  private final double kI = 0.001;
  private final double kD = 0.1;

  public SwerveDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.robotDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double[] inputs = Robot.m_oi.GetJoyValues(); //vx,vy,w
    double vy = inputs[0];
    double vx = inputs[1];
    double w = inputs[2];

    double a = vx - w*(RobotMap.l/RobotMap.r);
    double b = vx + w*(RobotMap.l/RobotMap.r);
    double c = vy - w*(RobotMap.w/RobotMap.r);
    double d = vy + w*(RobotMap.w/RobotMap.r);

    drivePower[0] = Math.sqrt((b*b)+(c*c));
    drivePower[1] = Math.sqrt((b*b)+(d*d));
    drivePower[2] = Math.sqrt((a*a)+(d*d));
    drivePower[3] = Math.sqrt((a*a)+(c*c));
    
    double max = drivePower[0];
    max = Math.max(max,drivePower[1]);
    max = Math.max(max,drivePower[2]);
    max = Math.max(max,drivePower[3]);

    if (max > 1){
      drivePower[0] /= max;
      drivePower[1] /= max;
      drivePower[2] /= max;
      drivePower[3] /= max;
    }

    turnAngle[0] = Math.atan2(b,c)*180/Math.PI;
    turnAngle[1] = Math.atan2(b,d)*180/Math.PI;
    turnAngle[2] = Math.atan2(a,d)*180/Math.PI;
    turnAngle[3] = Math.atan2(a,c)*180/Math.PI;

    for(int i=0; i<4; i++){
      
      error[i] = turnAngle[i] - Robot.robotDrivetrain.getEncoderDegrees()[i];
      sumError[i] += error[i];
      diffError[i] = error[i] - prevError[i];
      prevError[i] = error[i];

      turnPower[i] = error[i]*kP + sumError[i]*kI + diffError[i]*kD;
    }

    Robot.robotDrivetrain.setDrivePower(drivePower);
    Robot.robotDrivetrain.setTurnPower(turnPower);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
