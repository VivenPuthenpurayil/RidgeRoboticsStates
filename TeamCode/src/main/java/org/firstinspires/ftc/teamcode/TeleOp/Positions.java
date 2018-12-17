package org.firstinspires.ftc.teamcode.TeleOp;


public class Positions  {

    public static class Position{
        double[] vector;
        double orient;

        public Position(double[] vector, double orient) {
            this.vector = vector;
            this.orient = orient;
        }
        double[] returnv(){
            return vector;

        }
        double returno(){

            return orient;
        }

void updateOrient(double o){
         orient = o;

}


    }
    public static Position currentabsaposition(int xtrans, int ytrans, int ztrans, int orientation, String id) {
        double v[] = new double[3];
        if (id.equals("Back-Space")) {
            v[0] = -xtrans;
            v[1] = -ytrans;
            v[2] = ztrans;
            return new Position(v, orientation + 180);

        } else if (id.equals("Red-Footprint")) {
            v[0] = -ytrans;
            v[1] = xtrans;
            v[2] = ztrans;
            return new Position(v, orientation + 90);

        } else if (id.equals("Front-Craters")) {
            v[0] = xtrans;
            v[1] = ytrans;
            v[2] = ztrans;
            return new Position(v, orientation);

        } else if (id.equals("Blue-Rober")) {
            v[0] = ytrans;
            v[1] = -xtrans;
            v[2] = ztrans;
            return new Position(v, -orientation - 180);

        }
        else {
            return null;
        }
    }

    public Position motortoabs(Position p){
        double xval = p.returnv()[0]* Math.cos(p.returno()) + p.returnv()[1]*Math.sin(p.returno());
        double yval = p.returnv()[1]*Math.cos(p.returno()) -  p.returnv()[0]* Math.sin(p.returno());
        double[] a = {xval,yval,p.returnv()[2]};
        return new Position(a,p.returno());

    }
    public Position abstomotorCoord(Position p){
        double xval = p.returnv()[0]* Math.cos(p.returno()) - p.returnv()[1]*Math.sin(p.returno());
        double yval = p.returnv()[1]*Math.cos(p.returno()) +  p.returnv()[0]* Math.sin(p.returno());
        double[] a = {xval,yval,p.returnv()[2]};
        return new Position(a,p.returno());

    }
 public Position move(Position startpos, Position endpos){
        double orientMotorcoord = 0;
        Position start = abstomotorCoord(startpos);
        Position end = abstomotorCoord(endpos);
         double dify = end.returnv()[1]- start.returnv()[1];
         double difx = end.returnv()[0]- start.returnv()[0];

         if(difx>=0){

         }
        endpos.updateOrient(endpos.returno() + orientMotorcoord);
        return motortoabs(endpos); //returns abs pos
 }

}
