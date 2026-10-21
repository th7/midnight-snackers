package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import org.firstinspires.ftc.teamcode.simcore.Deployment;

public interface Deploy {
    Deployment from(Path worktree);
}
