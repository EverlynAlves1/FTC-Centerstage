
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class ArmSuspend extends SequentialCommandGroup {

    public ArmSuspend(DestemidosBot robot){

        super(
                new InstantCommand(()->robot.simpleArm.goToPosition2(RobotConstants.ARM_SUSPEND_GOAL))
        );
    }
}
