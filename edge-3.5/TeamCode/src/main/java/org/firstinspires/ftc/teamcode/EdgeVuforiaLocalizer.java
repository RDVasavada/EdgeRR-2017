package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class EdgeVuforiaLocalizer extends VuforiaLocalizerImpl {
    boolean vuforiaInstanceOpen = true;

    public EdgeVuforiaLocalizer(Parameters parameters) {
        super(parameters);
    }

    @Override
    public void close() {
        if (vuforiaInstanceOpen) {
            super.close();

            vuforiaInstanceOpen = false;
        }
    }
}