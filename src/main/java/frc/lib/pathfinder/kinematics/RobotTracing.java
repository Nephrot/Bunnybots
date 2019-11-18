package frc.lib.pathfinder.kinematics;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Optional;

import frc.lib.util.Point;

public class RobotTracing {

	public ArrayList<Point> path = new ArrayList<Point>();
	public ArrayList<Point> leftPath = new ArrayList<Point>();
	public ArrayList<Point> rightPath = new ArrayList<Point>();
	public ArrayList<Double> heading = new ArrayList<Double>();
	public double robotWidth;

	public RobotTracing(ArrayList<Point> path, double trackWidth) {
		this.path = path;
		this.robotWidth = trackWidth;
	}

	public void leftRight(ArrayList<Point> path, double trackWidth) {
		ArrayList<Double> gradient = new ArrayList<Double>();

		for (int i = 0; i < path.size() - 1; i++)
			gradient.add(Math.atan2(path.get(i + 1).getY() - path.get(i).getY(),
					path.get(i + 1).getX() - path.get(i).getX()));

		gradient.set(gradient.size() - 1, gradient.get(gradient.size() - 2));

		for (int i = 0; i < gradient.size(); i++) {
			leftPath.add(new Point(
					(((trackWidth / 2) * Math.cos(gradient.get(i) + Math.PI / 2)) + path.get(i).getX()),
					(((trackWidth / 2) * Math.sin(gradient.get(i) + Math.PI / 2)) + path.get(i).getY())));

			rightPath.add(new Point(
					(((trackWidth / 2) * Math.cos(gradient.get(i) - Math.PI / 2)) + path.get(i).getX()),
					(((trackWidth / 2) * Math.sin(gradient.get(i) - Math.PI / 2)) + path.get(i).getY())));
			
			
			double deg = Math.toDegrees(gradient.get(i));
			gradient.set(i, deg);
			if (i > 0) {
				if ((deg - gradient.get(i - 1)) > 180)
					gradient.set(i, -360 + deg);
				if ((deg - gradient.get(i - 1)) < -180)
					gradient.set(i, 360 + deg);
			}
		}
		this.heading = gradient;
	}
}
