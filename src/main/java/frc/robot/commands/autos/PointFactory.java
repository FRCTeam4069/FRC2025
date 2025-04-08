package frc.robot.commands.autos;

import java.util.ArrayList;

public class PointFactory {
    // Creates a list of Points
    public static ArrayList<VelocityPoint> createPointList(VelocityPoint... points) {
        ArrayList<VelocityPoint> list = new ArrayList<>();
        for (VelocityPoint p : points) {
            list.add(p);
        }
        return list;
    }
}
