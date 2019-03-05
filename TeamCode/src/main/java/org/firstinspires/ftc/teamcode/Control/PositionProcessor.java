package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PositionProcessor {

    VuforiaHandler vuforiaHandler;
    boolean vuforiaPowered = false;
    public static class Point{
        VectorF translation;
        Orientation orientation;
        String ref;
        boolean mappingType = false;

        public Point(VectorF translation, Orientation orientation, String ref) {
            this.translation = translation;
            this.orientation = orientation;
            this.ref = ref;
        }

    }

    Point lastPoint;

    final double CENTER = 0.47;

    public Point vuforiaInput(VectorF translation, Orientation orientation, String value){
        if(!value.equals("Front-Craters") && !value.equals("Red-Footprint"))
        {
            orientation.thirdAngle -= 180;
        }
        lastPoint = new Point(translation, orientation, value);
        return lastPoint;
    }

    public PositionProcessor(VuforiaHandler vuforiaHandler) {
        this.vuforiaHandler = vuforiaHandler;
        vuforiaPowered = true;
    }

    public static Point posToMap(Point p){
        p.translation.put(0, 72 - p.translation.get(0));
        p.translation.put(1, 72 + p.translation.get(1));
        p.mappingType = true;
        return p;

    }
    public PositionProcessor() {

    }

    public double phoneMountAngle(){
        double angle = CENTER;
        if (lastPoint.ref.equals(vuforiaHandler.frontCraters.getName())){
            angle = Math.atan(lastPoint.translation.get(1) / (72-Math.abs(lastPoint.translation.get(0)))) + (Orientation.getOrientation(vuforiaHandler.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

        }
        else if (lastPoint.ref.equals(vuforiaHandler.backSpace.getName())){
            angle = -Math.atan(lastPoint.translation.get(1) / (72-Math.abs(lastPoint.translation.get(0)))) + (Orientation.getOrientation(vuforiaHandler.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

        }
        else if (lastPoint.ref.equals(vuforiaHandler.blueRover.getName())){
            angle = Math.atan(lastPoint.translation.get(0) / (72-Math.abs(lastPoint.translation.get(1)))) + 90 - (Orientation.getOrientation(vuforiaHandler.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

        }
        else if (lastPoint.ref.equals(vuforiaHandler.redFootprint.getName())){
            angle = -Math.atan(lastPoint.translation.get(0) / (72-Math.abs(lastPoint.translation.get(1)))) + 90 - (Orientation.getOrientation(vuforiaHandler.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

        }
        return (CENTER + angle / Math.toRadians(300));
    }
}
