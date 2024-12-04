// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.FieldConstants;
// import frc.robot.subsystems.DriveTrain;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ChoreoAuto extends AutoCommandInterface {
//     private DriveTrain m_driveTrain;
//     private Pose2d m_initPose;

//     /** Creates a new NoteAuto. */
//     public ChoreoAuto(DriveTrain driveTrain) {
//         // Add your commands in the addCommands() call, e.g.
//         // addCommands(new FooCommand(), new BarCommand());
//         m_driveTrain = driveTrain;

//         try {
//             PathPlannerPath startPath = PathPlannerPath.fromPathFile("Note_C_1 to Shoot_1");
//             m_initPose = startPath.getStartingDifferentialPose();
//             addCommands(m_driveTrain.followPath(startPath));

//         } catch (Exception e) {
//             DriverStation.reportError("Unable to load PP path Test", true);
//             m_initPose = new Pose2d();
//         }
//     }

//     @Override
//     public Pose2d getInitialPose() {
//         return FieldConstants.flipPose(m_initPose);
//     }
// }


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ChoreoAuto extends Command {
    private static final double POSITION_TOLERANCE = 0.025;  // 2.5 centimeters

    final DriveTrain m_driveTrain;
    private Pose2d m_initPose;

    public ChoreoAuto(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        try {
            PathPlannerPath startPath = PathPlannerPath.fromChoreoTrajectory("Test path");
            m_initPose = startPath.getStartingDifferentialPose();
            m_driveTrain.followPath(startPath);

        } catch (Exception e) {
            DriverStation.reportError("Unable to load PP path Test", true);
            m_initPose = new Pose2d();
        }
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}
