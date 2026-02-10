package org.firstinspires.ftc.teamcode.util;

// import com.pedropathing.localization.Pose;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class StateTransfer {
    // public static Pose pose = new Pose();

    public static States.Obelisk motif = States.Obelisk.unread;

    public static States.Alliance alliance;
    public static Pose posePedro = null;
}
