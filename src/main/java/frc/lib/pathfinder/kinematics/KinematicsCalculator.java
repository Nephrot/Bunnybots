package frc.lib.pathfinder.kinematics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import frc.lib.pathfinder.pathCreator.SmoothPosition;
import frc.lib.pathfinder.pathCreator.PathGenerator;
import frc.lib.util.Point;

public class KinematicsCalculator {

	public static ArrayList<Double> distance = new ArrayList<Double>();
	public static ArrayList<Double> curvature = new ArrayList<Double>();
	public static ArrayList<Double> velocity = new ArrayList<Double>();
	public static ArrayList<Double> leftVelocity = new ArrayList<Double>();
	public static ArrayList<Double> rightVelocity = new ArrayList<Double>();
	public static ArrayList<Double> outputs = new ArrayList<Double>();

	static double k1 = 0;
	static double k2 = 0;
	static double centerA = 0;
	static double centerB = 0;
	static double radius = 0;
	static double maxVelocity = 7;
	static double maxAcceleration = 10;
	static double maxChange = 1;
	static double vf = 0;
	static double vi = 0;
	static double deltaTime;
	static double maxRate = 0.7;
	static double output;
	static double input;
	static double lastOutput;
	public static ArrayList<Double> leftDistance = new ArrayList<Double>();
	public static ArrayList<Double> rightDistance = new ArrayList<Double>();

	public static void calculateDistance() {
		SmoothPosition.smoothPath(PathGenerator.finalPoints, SmoothPosition.dataWeightA,
				SmoothPosition.smoothWeightB, SmoothPosition.tolerance);
		for (int i = 1; i < SmoothPosition.newPathPoints.size(); i++) {
			if (i == 1) {
				distance.add(new Double(Math.sqrt(Math.pow(
						SmoothPosition.newPathPoints.get(i).getX() - SmoothPosition.newPathPoints.get(i - 1).getX(), 2)
						+ Math.pow(SmoothPosition.newPathPoints.get(i).getY()
								- SmoothPosition.newPathPoints.get(i - 1).getY(), 2))));
			} else {
				distance.add(new Double(distance.get(i - 2) + Math.sqrt(Math.pow(
						SmoothPosition.newPathPoints.get(i).getX() - SmoothPosition.newPathPoints.get(i - 1).getX(), 2)
						+ Math.pow(SmoothPosition.newPathPoints.get(i).getY()
								- SmoothPosition.newPathPoints.get(i - 1).getY(), 2))));
			}
		}
	}

	public static void calculateLeftDistance(ArrayList<Point> leftPath) {

		for (int i = 1; i < leftPath.size(); i++) {
			if (i == 1) {
				leftDistance.add(new Double(Math.sqrt(Math.pow(leftPath.get(i).getX() - leftPath.get(i - 1).getX(), 2)
						+ Math.pow(leftPath.get(i).getY() - leftPath.get(i - 1).getY(), 2))));
			} else {
				leftDistance.add(new Double(leftDistance.get(i - 2)
						+ Math.sqrt(Math.pow(leftPath.get(i).getX() - leftPath.get(i - 1).getX(), 2)
								+ Math.pow(leftPath.get(i).getY() - leftPath.get(i - 1).getY(), 2))));
			}
		}
	}

	public static void calculateRightDistance(ArrayList<Point> rightPath) {

		for (int i = 1; i < rightPath.size(); i++) {
			if (i == 1) {
				rightDistance
						.add(new Double(Math.sqrt(Math.pow(rightPath.get(i).getX() - rightPath.get(i - 1).getX(), 2)
								+ Math.pow(rightPath.get(i).getY() - rightPath.get(i - 1).getY(), 2))));
			} else {
				rightDistance.add(new Double(rightDistance.get(i - 2)
						+ Math.sqrt(Math.pow(rightPath.get(i).getX() - rightPath.get(i - 1).getX(), 2)
								+ Math.pow(rightPath.get(i).getY() - rightPath.get(i - 1).getY(), 2))));
			}
		}
	}

	public static void calculuateCurvature() {
		curvature.add((double) 0);
		for (int i = 1; i < SmoothPosition.newPathPoints.size() - 1; i++) {
			k1 =
					// 0.5 * (x^2+y^2-x2^2-y2^2)/(x^1-x2^2)
					0.5 * (Math.pow(SmoothPosition.newPathPoints.get(i).getX(), 2)
							+ Math.pow(SmoothPosition.newPathPoints.get(i).getY(), 2)
							- Math.pow(SmoothPosition.newPathPoints.get(i - 1).getX(), 2)
							- Math.pow(SmoothPosition.newPathPoints.get(i - 1).getY(), 2))
							/ ((SmoothPosition.newPathPoints.get(i).getX() + 0.001)
									- SmoothPosition.newPathPoints.get(i - 1).getX());

			k2 =
					// (y-y2)/(x-x2)
					(SmoothPosition.newPathPoints.get(i).getY() - SmoothPosition.newPathPoints.get(i - 1).getY())
							/ ((SmoothPosition.newPathPoints.get(i).getX() + 0.001)
									- SmoothPosition.newPathPoints.get(i - 1).getX());
			centerB =
					// 0.5 * (x2^2-2*x2*k1 + y2^2 - x3^2+2*x3*k1-y3^2
					0.5 * (Math.pow(SmoothPosition.newPathPoints.get(i - 1).getX(), 2)
							- 2 * (SmoothPosition.newPathPoints.get(i - 1).getX()) * k1
							+ Math.pow(SmoothPosition.newPathPoints.get(i - 1).getY(), 2)
							- Math.pow(SmoothPosition.newPathPoints.get(i + 1).getX(), 2)
							+ 2 * (SmoothPosition.newPathPoints.get(i + 1).getX()) * k1
							- Math.pow(SmoothPosition.newPathPoints.get(i + 1).getY(), 2))
							/ (SmoothPosition.newPathPoints.get(i + 1).getX() * k2
									- SmoothPosition.newPathPoints.get(i + 1).getY()
									+ SmoothPosition.newPathPoints.get(i - 1).getY()
									- SmoothPosition.newPathPoints.get(i - 1).getX() * k2);
			centerA = k1 - k2 * centerB;
			radius = Math.sqrt(Math.pow(SmoothPosition.newPathPoints.get(i).getX() - centerA, 2)
					+ Math.pow(SmoothPosition.newPathPoints.get(i).getY() - centerB, 2));
			if (radius > 0)
				curvature.add(1 / radius);
			else
				curvature.add((double) 0);
		}
		curvature.add((double) 0);
	}

	public static void calculateVelocities() {
		velocity.add((double) vf);
		for (int i = SmoothPosition.newPathPoints.size() - 2; i >= 0; i--) {
			double distances = SmoothPosition.newPathPoints.get(i).distance(SmoothPosition.newPathPoints.get(i+1));
			velocity.add(Math.min(Math.min(maxVelocity, 2 / (curvature.get(i) * 3)),
					Math.sqrt(Math.pow(velocity.get(SmoothPosition.newPathPoints.size() - i - 2), 2)
							+ (2 * maxAcceleration * distances))));
            System.out.println(velocity.get(SmoothPosition.newPathPoints.size() - i - 2));
		}
		Collections.reverse(velocity);
		velocity.set(0, (double) 0);
		for(int i = 1; i < SmoothPosition.newPathPoints.size() - SmoothPosition.newPathPoints.size() / 3; i++) {
			velocity.set(i, Math.min(Math.min(maxVelocity, 2 / (curvature.get(i) * 3)), (velocity.get(i+1) + velocity.get(i-1)) / 2));
		}
	}

	public static void calculateLeftVelocities(ArrayList<Point> leftPath) {
		leftVelocity.add((double) 0);
		for (int i = leftPath.size() - 2; i >= 0; i--) {
			double distances = Math.sqrt(Math.pow(leftPath.get(i + 1).getX() - leftPath.get(i).getX(), 2)
					+ Math.pow(leftPath.get(i + 1).getY() - leftPath.get(i).getY(), 2));
			leftVelocity.add(Math.min(Math.min(maxVelocity, 2 / (curvature.get(i) * 3)), Math
					.sqrt(Math.pow(leftVelocity.get(leftPath.size() - i - 2), 2) + 2 * maxAcceleration * distances)));

		}
		Collections.reverse(leftVelocity);
	}

	public static void calculateRightVelocities(ArrayList<Point> rightPath) {
		rightVelocity.add((double) 0);
		for (int i = rightPath.size() - 2; i >= 0; i--) {
			double distances = Math.sqrt(Math.pow(rightPath.get(i + 1).getX() - rightPath.get(i).getX(), 2)
					+ Math.pow(rightPath.get(i + 1).getY() - rightPath.get(i).getY(), 2));
			rightVelocity.add(Math.min(Math.min(maxVelocity, 2 / (curvature.get(i) * 3)), Math
					.sqrt(Math.pow(rightVelocity.get(rightPath.size() - i - 2), 2) + 2 * maxAcceleration * distances)));

		}
		Collections.reverse(rightVelocity);
    }
    
	public static void rateLimiter() {
		outputs.add((double) 0);
		for (int i = 0; i < SmoothPosition.newPathPoints.size() - 1; i++) {
			double distances = Math.sqrt(Math
					.pow(SmoothPosition.newPathPoints.get(i + 1).getX() - SmoothPosition.newPathPoints.get(i).getX(), 2)
					+ Math.pow(
							SmoothPosition.newPathPoints.get(i + 1).getY() - SmoothPosition.newPathPoints.get(i).getY(),
							2));
			maxChange = 7;
//			if(velocity.get(i+1) - outputs.get(i) < -maxChange) {
//				outputs.add((outputs.get(i) -maxChange)/16);
//			}
//			else if(velocity.get(i+1) - outputs.get(i) > maxChange) {
//				outputs.add((outputs.get(i) + maxChange)/16);
//			}
//			else 
			if (((double) i) / ((double) velocity.size()) > 0.9) {
				outputs.add(velocity.get(i + 1) / 40);
			} else {
				outputs.add(Math.max(0.3, (velocity.get(i + 1) / 40)));
			}
		}
	}
}
