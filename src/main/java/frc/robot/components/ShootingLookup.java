// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components;

import java.util.TreeMap;

import edu.wpi.first.math.Pair;

/** An interpolating lookup table for the shooter. */
public class ShootingLookup {
    // Tree Map datastructure stores (Flywheel RPM, Hood Angle)s associated with a distance in feet.
    private final TreeMap<Double, Pair<Double, Double>> treemap;

    public ShootingLookup(){
        treemap = new TreeMap<>();
    }

    /**Adds a point to the treemap */
    public void insert(double distance, Pair<Double,Double> shootingOutputs){
        treemap.put(distance, shootingOutputs);
    }

    /**Multiplies a pair by a scalar */
    private static Pair<Double, Double> mult(Pair<Double, Double> pairIn, double scalarIn){
        return new Pair<Double, Double>(pairIn.getFirst() * scalarIn, pairIn.getSecond() * scalarIn);
    }

    /**Adds two pairs together, piecewise */
    private static Pair<Double, Double> add(Pair<Double, Double> a, Pair<Double, Double> b){
        return new Pair<> (a.getFirst() + b.getFirst(), a.getSecond() + b.getSecond());
    }

    /**Gets a value from the treemap given a distance, interpolating between values for distances not explicitly in the map.
     * Clamped between the min and max distances added to the treemap.
     */
    public Pair<Double, Double> get(double distance){
        // If the map has a distance, use it.
        if (treemap.containsKey(distance)){
            return treemap.get(distance);
        }

        // Find the key immediately below and above the given distance.
        Double floorKey = treemap.floorKey(distance);
        Double ceilingKey = treemap.ceilingKey(distance);

        // If either key is null (ie we are above or below the max or min value), use the max or min value
        if (floorKey == null){
            return treemap.get(ceilingKey);
        }
        if (ceilingKey == null){
            return treemap.get(floorKey);
        }

        // Find the interpolation amount
        double distanceInCurrentRange = (distance - floorKey)/(ceilingKey - floorKey);

        // Interpolate and return
        return add(mult(treemap.get(ceilingKey), distanceInCurrentRange), mult(treemap.get(floorKey), (1 - distanceInCurrentRange)));
    }
}
