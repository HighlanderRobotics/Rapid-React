// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components;

import java.util.TreeMap;

import edu.wpi.first.math.Pair;

/** Add your docs here. */
public class ShootingLookup {
    private final TreeMap<Double, Pair<Double, Double>> treemap;

    public ShootingLookup(){
        treemap = new TreeMap<>();
    }

    public void insert(double distance, Pair<Double,Double> shootingOutputs){
        treemap.put(distance, shootingOutputs);
    }

    private static Pair<Double, Double> mult(Pair<Double, Double> pairIn, double scalarIn){
        return new Pair<Double, Double>(pairIn.getFirst() * scalarIn, pairIn.getSecond() * scalarIn);
    }

    private static Pair<Double, Double> add(Pair<Double, Double> a, Pair<Double, Double> b){
        return new Pair<> (a.getFirst() + b.getFirst(), a.getSecond() + b.getSecond());
    }

    public Pair<Double, Double> get(double distance){
        if (treemap.containsKey(distance)){
            return treemap.get(distance);
        }

        Double floorKey = treemap.floorKey(distance);
        Double ceilingKey = treemap.ceilingKey(distance);

        if (floorKey == null){
            return treemap.get(ceilingKey);
        }
        if (ceilingKey == null){
            return treemap.get(floorKey);
        }

        double distanceInCurrentRange = (distance - floorKey)/(ceilingKey - floorKey);

        return add(mult(treemap.get(ceilingKey), distanceInCurrentRange), mult(treemap.get(floorKey), (1 - distanceInCurrentRange)));
    }
}
