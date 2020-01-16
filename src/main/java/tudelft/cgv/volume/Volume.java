/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tudelft.cgv.volume;

import java.io.File;
import java.io.IOException;

/**
 * This class stores the volume and contains functions to retrieve values of the volume.
 *
 * @author michel modified by Anna
 * Edit by Andrea Alfieri, Reinier Koops & Aditya Kunar
 */

//////////////////////////////////////////////////////////////////////
///////////////// CONTAINS FUNCTIONS TO BE IMPLEMENTED ///////////////
//////////////////////////////////////////////////////////////////////

public class Volume {
	//Do NOT modify these attributes
    private int dimX, dimY, dimZ;
    private short[] data;
    private int[] histogram;

    // Do NOT modify this function
    // This function returns the nearest neighbour given a position in the volume given by coord.
    public short getVoxelNN(double[] coord) {
        if (coord[0] < 0 || coord[0] > (dimX - 1) || coord[1] < 0 || coord[1] > (dimY - 1)
                || coord[2] < 0 || coord[2] > (dimZ - 1)) {
            return 0;
        }
        /* notice that in this framework we assume that the distance between neighbouring voxels is 1 in all directions */
        int x = (int) Math.round(coord[0]);
        int y = (int) Math.round(coord[1]);
        int z = (int) Math.round(coord[2]);

        return getVoxel(x, y, z);
    }

    //Do NOT modify this function
    //This function linearly interpolates the value g0 and g1 given the factor (t) 
    //the result is returned. It is used for the tri-linearly interpolation the values 
    private float interpolate(float g0, float g1, float factor) {
        float result = (1 - factor) * g0 + factor * g1;
        return result;
    }

    //Do NOT modify this function
    // This function returns the trilinear interpolated value of the position given by position coord. 
    
    public float getVoxelLinearInterpolate(double[] coord) {
        if (coord[0] < 0 || coord[0] > (dimX - 2) || coord[1] < 0 || coord[1] > (dimY - 2)
                || coord[2] < 0 || coord[2] > (dimZ - 2)) {
            return 0;
        }
        /* notice that in this framework we assume that the distance between neighbouring voxels is 1 in all directions*/
        int x = (int) Math.floor(coord[0]);
        int y = (int) Math.floor(coord[1]);
        int z = (int) Math.floor(coord[2]);

        float fac_x = (float) coord[0] - x;
        float fac_y = (float) coord[1] - y;
        float fac_z = (float) coord[2] - z;

        float t0 = interpolate(getVoxel(x, y, z), getVoxel(x + 1, y, z), fac_x);
        float t1 = interpolate(getVoxel(x, y + 1, z), getVoxel(x + 1, y + 1, z), fac_x);
        float t2 = interpolate(getVoxel(x, y, z + 1), getVoxel(x + 1, y, z + 1), fac_x);
        float t3 = interpolate(getVoxel(x, y + 1, z + 1), getVoxel(x + 1, y + 1, z + 1), fac_x);
        float t4 = interpolate(t0, t1, fac_y);
        float t5 = interpolate(t2, t3, fac_y);
        float t6 = interpolate(t4, t5, fac_z);

        return t6;
    }

    // User defined; recommended values are: -1, -0.75, -0.5
    float a = -0.75f; // global variable <a> used in cubic interpolation.

    /**
     * Computes the weights for one of the four samples used in cubic interpolation
     * weight(x) is similar to the h(x); the Cubic interpolation kernel function.
     *
     * @param x                 one out of four sample points.
     * @param absXMaxOneElseTwo True when |x| < 1, False when |x| < 2
     * @return weight of sample point.
     */
    public float weight(float x, Boolean absXMaxOneElseTwo) {
        float abs = Math.abs(x);

        // Case 1: 0 ≤ |𝑥| < 1
        if (absXMaxOneElseTwo)
            return (float) (((a + 2) * Math.pow(abs, 3)) - ((a + 3) * Math.pow(abs, 2)) + 1);
        
        // Case 2: 1 ≤ |x| < 2
        return (float) ((a * Math.pow(abs, 3)) - (5 * a * Math.pow(abs, 2)) + (8 * a * abs) - (4 * a));
        // Case 3: Impossible due to assumption that distance between neighbouring voxels is 1.
    }

    /**
     * Computes the 1D cubic interpolation using four (4^1) reference points (g0, g1, g2, g3) with
     * their corresponding weights. The result of each reference point with the corresponding
     * weight is summed up, whereby the total is the interpolation of the point.
     *
     * @param g0     reference point
     * @param g1     reference point
     * @param g2     reference point
     * @param g3     reference point
     * @param factor x + factor = distance from actual point to position wanted to interpolate.
     * @return interpolation of point based on four reference points.
     */
    public float cubicinterpolate(float g0, float g1, float g2, float g3, float factor) {

        float result = 0.0f;

        // interpolation takes into account 4 voxels in each direction
        result += g0 * weight(1 + factor, false);
        result += g1 * weight(factor, true);
        result += g2 * weight(1 - factor, true);
        result += g3 * weight(2 - factor, false);

        return result;
    }

    /**
     * Computes the 2D cubic interpolation using sixteen (4^2) reference points with their
     * corresponding weights. The result of each reference point with the corresponding weight is
     * summed up, whereby the total is the interpolation of the point.
     *
     * @param coord list with sixteen reference points
     * @param z     the third axis of (x, y, z) / 3D
     * @return interpolation of point based on sixteen reference points.
     */
    public float bicubicinterpolateXY(double[] coord, int z) {
        //Coord is an array in the form (x_, y_)
        int x = (int) Math.floor(coord[0]);
        int y = (int) Math.floor(coord[1]);

        // (x-1; y+2)       (x; y+2)        (x+1; y+2)      (x+2; y+2)
        // (x-1; y+1)       (x; y+1)        (x+1; y+1)      (x+2; y+1)
        // (x-1; y)         (x; y)          (x+1; y)        (x+2; y)
        // (x-1; y-1)       (x; y-1)        (x+1; y-1)      (x+2; y-1)

        float t0 = cubicinterpolate(
                getVoxel(x - 1, y - 1, z),
                getVoxel(x, y - 1, z),
                getVoxel(x + 1, y - 1, z),
                getVoxel(x + 2, y - 1, z),
                (float) (coord[0] - x)
        );
        float t1 = cubicinterpolate(
                getVoxel(x - 1, y, z),
                getVoxel(x, y, z),
                getVoxel(x + 1, y, z),
                getVoxel(x + 2, y, z),
                (float)(coord[0] - x)
        );
        float t2 = cubicinterpolate(
                getVoxel(x - 1, y + 1, z),
                getVoxel(x, y + 1, z),
                getVoxel(x + 1, y + 1, z),
                getVoxel(x + 2, y + 1, z),
                (float) (coord[0] - x)
        );
        float t3 = cubicinterpolate(
                getVoxel(x - 1, y + 2, z),
                getVoxel(x, y + 2, z),
                getVoxel(x + 1, y + 2, z),
                getVoxel(x + 2, y + 2, z),
                (float) (coord[0] - x)
        );
        return cubicinterpolate(t0, t1, t2, t3, (float) (coord[1] - y));
    }

    /**
     * Computes the 3D cubic interpolation using sixty-four (4^3) reference points with their
     * corresponding weights. The result of each reference point with the corresponding weight is
     * summed up, whereby the total is the interpolation of the point.
     *
     * @param coord list with sixty-four reference points
     * @return interpolation of point based on sixty-four reference points.
     */
    public float getVoxelTriCubicInterpolate(double[] coord) {
        // Outside of the volume, the values is 0 by default
        if (coord[0] < 1 || coord[0] > (dimX - 3) || coord[1] < 1 || coord[1] > (dimY - 3)
                || coord[2] < 1 || coord[2] > (dimZ - 3)) {
            return 0;
        }
        // coord is defined as [x_, y_, z_]
        int z = (int) Math.floor(coord[2]);

        float t0 = bicubicinterpolateXY(coord, z - 1);
        float t1 = bicubicinterpolateXY(coord, z);
        float t2 = bicubicinterpolateXY(coord, z + 1);
        float t3 = bicubicinterpolateXY(coord, z + 2);

        float result = cubicinterpolate(t0, t1, t2, t3, (float) Math.abs(coord[2] - z));

        // Colorspace limitation: it is from value 0 to 255
        if (result < 0) return 0;
        if (result > 255) return 255;

        return result;
    }

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

    //Do NOT modify this function
    public Volume(int xd, int yd, int zd) {
        data = new short[xd * yd * zd];
        dimX = xd;
        dimY = yd;
        dimZ = zd;
    }

    //Do NOT modify this function
    public Volume(File file) {
        try {
            VolumeIO reader = new VolumeIO(file);
            dimX = reader.getXDim();
            dimY = reader.getYDim();
            dimZ = reader.getZDim();
            data = reader.getData().clone();
            computeHistogram();
        } catch (IOException ex) {
            System.out.println("IO exception");
        }
    }

    //Do NOT modify this function
    public short getVoxel(int x, int y, int z) {
        int i = x + dimX * (y + dimY * z);
        return data[i];
    }

    //Do NOT modify this function
    public void setVoxel(int x, int y, int z, short value) {
        int i = x + dimX * (y + dimY * z);
        data[i] = value;
    }

    //Do NOT modify this function
    public void setVoxel(int i, short value) {
        data[i] = value;
    }

    //Do NOT modify this function
    public short getVoxel(int i) {
        return data[i];
    }

    //Do NOT modify this function
    public int getDimX() {
        return dimX;
    }

    //Do NOT modify this function
    public int getDimY() {
        return dimY;
    }

    //Do NOT modify this function
    public int getDimZ() {
        return dimZ;
    }

    //Do NOT modify this function
    public short getMinimum() {
        short minimum = data[0];
        for (int i = 0; i < data.length; i++) {
            minimum = data[i] < minimum ? data[i] : minimum;
        }
        return minimum;
    }

    //Do NOT modify this function
    public short getMaximum() {
        short maximum = data[0];
        for (int i = 0; i < data.length; i++) {
            maximum = data[i] > maximum ? data[i] : maximum;
        }
        return maximum;
    }

    //Do NOT modify this function
    public int[] getHistogram() {
        return histogram;
    }

    //Do NOT modify this function
    private void computeHistogram() {
        histogram = new int[getMaximum() + 1];
        for (int i = 0; i < data.length; i++) {
            histogram[data[i]]++;
        }
    }
}
