package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PositionProcessor {

    public static class Point{
        VectorF translation;
        Orientation orientation;
        String ref;

        public Point(VectorF translation, Orientation orientation, String ref) {
            this.translation = translation;
            this.orientation = orientation;
            this.ref = ref;
        }

    }

    Point lastPoint;

    double CENTER = 0.47;

    public void vuforiaInput(VectorF translation, Orientation orientation, String value){
        if(!value.equals("Front-Craters") && !value.equals("Red-Footprint"))
        {
            orientation.thirdAngle -= 180;
        }
        lastPoint = new Point(translation, orientation, value);
    }

    public void phoneMountAngle(){
        switch (lastPoint.ref){

        }
    }
}
