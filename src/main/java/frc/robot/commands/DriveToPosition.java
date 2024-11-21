// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveToPosition extends Command {
    private static final double POSITION_TOLERANCE = 0.025;  // 2.5 centimeters

    final DriveTrain m_driveTrain;

    Translation2d m_positionChangeBlue;
    Translation2d m_endPosition;

    public DriveToPosition(DriveTrain driveTrain, Translation2d positionChangeBlue) {
        m_driveTrain = driveTrain;
        m_positionChangeBlue = positionChangeBlue;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Compute endPosition. Work as if we are on the Blue side
        Translation2d startPosBlue = FieldConstants.flipTranslation(m_driveTrain.getPose().getTranslation());

        // Now that we know our start position, compute end
        Translation2d endPosBlue = startPosBlue.plus(m_positionChangeBlue);
        
        // have Blue position. Make it field absolute
        m_endPosition = FieldConstants.flipTranslation(endPosBlue);

        // so now, m_endPosition is the destination, in absolute field coordinates
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d offset = getOffset();

        // set the magnitude to the desired speed
        double scale = 0.5 * DriveTrain.MAX_SPEED / offset.getNorm();
        Translation2d velocity = offset.times(scale);

        // this is field-centric, using absolute coordinate system (not driver view)
        m_driveTrain.driveWithSpeeds(velocity.getX(), velocity.getY(), 0.0, false);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.drive(0, 0, 0, false);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Translation2d offset = getOffset();
        return offset.getNorm() < POSITION_TOLERANCE;
    }

    private Translation2d getOffset() {
        Translation2d currPos = m_driveTrain.getPose().getTranslation();
        return m_endPosition.minus(currPos);
    }
}
