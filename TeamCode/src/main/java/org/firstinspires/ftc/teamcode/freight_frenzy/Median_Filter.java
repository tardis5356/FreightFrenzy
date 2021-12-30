//package org.firstinspires.ftc.teamcode.freight_frenzy;
//import java.util.Arrays;
//
//public class Median_Filter {
//
////        public static void main(String[] args) {
////
////            // initialize array with odd number of element
////            double[] values = { 1,2.12,1.5,3.14,1,1,2,2,2,3};
////            printArray(values);
////            // calculate median
////            double median = median(values);
////            System.out.println("Median is : " + median);
////
////        // do a test in adding a new element to the end of the array "values"
////            values =  popValueIntoArray(values, 1000);
////            values =  popValueIntoArray(values, 2000);
////            values =  popValueIntoArray(values, 4000);
////            values =  popValueIntoArray(values, 2);
////            values =  popValueIntoArray(values, 2);
////            values =  popValueIntoArray(values, 3);
////            values =  popValueIntoArray(values, 3.2);
////            printArray(values);
////            // calculate median
////            median = median(values);
////            System.out.println("Median is : " + median);
////
////
////        }
//
//        static double median(double[] values) {
//            // get array length
//            int totalElements = values.length;
//             // make temporary array that gets the sorted/manipulated
//            double[] newArray = new double[totalElements];
//            // now make the actual copy
//            for (int i = 0; i < totalElements; i++) {
//                newArray[i] = values[i];
//            }
//
//            // sort array
//            Arrays.sort(newArray);
//
//            double median; // now get the median by finding the "halfway" element in the sorted array
//      //      System.out.println("# elements is : " + totalElements);
//            // check if total number of scores is even
//            if (totalElements % 2 == 0) {
//                double sumOfMiddleElements = newArray[totalElements / 2] +
//                        newArray[totalElements / 2 - 1];
//                // calculate average of middle elements
//                median = ((double) sumOfMiddleElements) / 2;
//            } else {
//                // get the middle element
//                median = (double) newArray[newArray.length / 2];
//            }
//            return median;
//        }
//
//        static double[] popValueIntoArray(double[] previousArray, double latestValue) {
//            // add element to end of array, drop out first element in array;
//            int totalElements = previousArray.length;
//            double[] newArray = new double[totalElements];
//
//            for (int i = 0; i < totalElements-1; i++) {
//                newArray[i] = previousArray[i+1];
//            }
//            newArray[totalElements-1]=latestValue; // append latest value to the end of the array
//
//            // now placed updates in returned array
//            for (int i = 0; i < totalElements; i++) {
//                previousArray[i] = newArray[i];
//            }
//
//            return previousArray;
//        }
//
//        static void printArray(double[] values) {
//            for (double i : values) {
//                System.out.print(" " +i);
//            }
//            System.out.println("");
//            int totalElements = values.length;
//            System.out.println("# elements is : " + totalElements);
//        }
//
//}
