// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;

public class SimpleAuto extends SequentialCommandGroup {
    private Pose2d m_initPose;

    /** Creates a new SimpleAuto. */
    public SimpleAuto(DriveTrain driveTrain) {

        // PathPlannerPath C1 to shoot
        // ChoreoPath C1 to shoot
        // PathPlannerPath startPath = PathPlannerPath.fromChoreoTrajectory("ChoreoPath C1 to shoot");

        PathPlannerPath startPath = DriveTrain.loadPath("PathPlannerPath C1 to shoot");
        m_initPose = startPath.getStartingDifferentialPose();
        addCommands(driveTrain.followPath(startPath));
        
        // addCommands(
        //         new DriveToPosition(driveTrain, new Translation2d(3.0, 0.0)),
        //         new DriveToPosition(driveTrain, new Translation2d(0.0, 3.0)),
        //         new DriveToPosition(driveTrain, new Translation2d(-3.0, 0.0)),
        //         new DriveToPosition(driveTrain, new Translation2d(.0, -3.0))
        // );
    }
}
