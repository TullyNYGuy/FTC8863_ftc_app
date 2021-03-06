package org.firstinspires.ftc.teamcode.Lib.FTCLib;

import java.util.ArrayList;

public class StatTrackerGB {

    //*********************************************************************************************
    //          ENUMERATED TYPES
    //
    // user defined types
    //
    //*********************************************************************************************


    //*********************************************************************************************
    //          PRIVATE DATA FIELDS
    //
    // can be accessed only by this class, or by using the public
    // getter and setter methods
    //*********************************************************************************************
    private double maximum;
    private double minimum;
    private double average;
    private int count;

    // running sum
    private double sum;

    // place to store all of the values. Arrays cannot be extended in Java. They are a fixed length.
    // ArrayList can be extended.
    // Note that an Arraylist is probably a lot slower than a true array.
    private ArrayList<Double> list;

    // flag to indicate if the values in the ArrayList have been processed to find max, min, average
    // sum and count
    boolean isProcessed = false;

    //*********************************************************************************************
    //          GETTER and SETTER Methods
    //
    // allow access to private data fields in a controlled way
    //
    //*********************************************************************************************

    // These are all readonly properties so we only want a getter method
    public double getMaximum() {
        if (!list.isEmpty()) {
            // since the list was not empty I'm assuming the user chose not to use updateStats() or
            // updateStatsUnified(). So I have to process all of the values that were stored in the
            // Arraylist.
            processArrayList();
        }
        return maximum;
    }

    public double getMinimum() {
        if (!list.isEmpty()) {
            // since the list was not empty I'm assuming the user chose not to use updateStats() or
            // updateStatsUnified(). So I have to process all of the values that were stored in the
            // Arraylist.
            processArrayList();
        }
        return minimum;
    }

    public double getAverage() {
        if (!list.isEmpty()) {
            // since the list was not empty I'm assuming the user chose not to use updateStats() or
            // updateStatsUnified(). So I have to process all of the values that were stored in the
            // Arraylist.
            processArrayList();
        }
        return average;
    }

    public double getCount() {
        if (!list.isEmpty()) {
            // since the list was not empty I'm assuming the user chose not to use updateStats() or
            // updateStatsUnified(). So I have to process all of the values that were stored in the
            // Arraylist.
            processArrayList();
        }
        return count;
    }

    public double getSum() {
        if (!list.isEmpty()) {
            // since the list was not empty I'm assuming the user chose not to use updateStats() or
            // updateStatsUnified(). So I have to process all of the values that were stored in the
            // Arraylist.
            processArrayList();
        }
        return sum;
    }

    //*********************************************************************************************
    //          Constructors
    //
    // the function that builds the class when an object is created
    // from it
    //*********************************************************************************************
    public StatTrackerGB() {
        // initialize the properties
        reset();
        // create an arraylist that is empty
        list = new ArrayList<Double>();
    }

    //*********************************************************************************************
    //          Helper Methods
    //
    // methods that aid or support the major functions in the class
    //*********************************************************************************************

    /**
     * Use the value passed in and the previous maximum to determine if the new value is a new maximum.
     *
     * @param newValue
     */
    private void updateMax(double newValue) {
        if (newValue > maximum) {
            maximum = newValue;
        }
    }

    /**
     * Use the value passed in and the previous minimum to determine if the new value is a new minimum.
     *
     * @param newValue
     */
    private void updateMin(double newValue) {
        if (newValue < minimum) {
            minimum = newValue;
        }
    }

    /**
     * Use the value passed in and the previously processed values to calculate a new average and
     * update the number of times the stat tracker has been called.
     *
     * @param newValue
     */
    private void updateAverage(double newValue) {
        sum = sum + newValue;
        count++;
        average = sum / count;
    }

    /**
     * Iterate across the stored values and calculate min, max and average
     */
    private void processArrayList() {
        // only process the Arraylist once
        if (!isProcessed) {
            for (Double number : list) {
                updateMax(number);
                updateMin(number);
                updateAverage(number);
            }
        }
        // indicate the ArrayList has already been processed
        isProcessed = true;
    }

    //*********************************************************************************************
    //          MAJOR METHODS
    //
    // public methods that give the class its functionality
    //*********************************************************************************************

    /**
     * This is public method that gets repeatedly called from a loop. Each time the lastest in a
     * series of values is passed in. This method updates all of the stats kept for the series of
     * values.
     *
     * @param newValue
     */
    public void updateStats(double newValue) {
        // The first time this is called there are no previous values so just set all of the
        // stats to the new value.
        if (count == 0) {
            maximum = newValue;
            minimum = newValue;
            sum = newValue;
            average = newValue;
            count = 1;
            // This is not the first time called so update the stats the normal way.
        } else {
            updateMax(newValue);
            updateMax(newValue);
            updateAverage(newValue);
        }
    }

    /**
     * This method does the same thing as updateStats but does not call other methods to do it. It
     * will be interesting to see how much faster this method runs.
     * @param newValue
     */
    public void updateStatsUnified(double newValue) {
        // The first time this is called there are no previous values so just set all of the
        // stats to the new value.
        if (count == 0) {
            maximum = newValue;
            minimum = newValue;
            sum = newValue;
            average = newValue;
            count = 1;
            // This is not the first time called so update the stats the normal way.
        } else {
            // update the max
            if (newValue > maximum) {
                maximum = newValue;
            }

            // update the min
            if (newValue < minimum) {
                minimum = newValue;
            }

            // update the average and the count
            sum = sum + newValue;
            count++;
            average = sum / count;
        }
    }

    public void updateList(double newValue) {
        list.add(newValue);
    }

    /**
     * Reset everything so that a fresh set of numbers can be tracked.
     */
    public void reset() {
        maximum = 0;
        minimum = 0;
        average = 0;
        sum = 0;
        count = 0;
        isProcessed = false;
    }

}
