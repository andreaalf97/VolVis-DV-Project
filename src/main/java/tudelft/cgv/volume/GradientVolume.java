/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tudelft.cgv.volume;

/**
 * It stores and provides methods to compute and retrieve gradient vector
 * In math, gradient vectors are vectors that point to the greatest increase of a function
 *
 * @author michel and modified by Anna Vilanova
 * Edit by Andrea Alfieri, Reinier Koops & Aditya Kunar
 */

//////////////////////////////////////////////////////////////////////
///////////////// CONTAINS FUNCTIONS TO BE IMPLEMENTED ///////////////
//////////////////////////////////////////////////////////////////////
public class GradientVolume {
    //Do NOT modify this attributes
    private int dimX, dimY, dimZ;
    private VoxelGradient zero = new VoxelGradient();
    VoxelGradient[] data;
    Volume volume;
    double maxmag;

    //Do NOT modify this function
    // 
    // Computes the gradient of the volume attribute and save it into the data attribute
    // This is a lengthy computation and is performed only once (have a look at the constructor GradientVolume) 
    //
    private void compute() {
        for (int i = 0; i < data.length; i++) {
            data[i] = zero;
        }

        for (int z = 1; z < dimZ - 1; z++) {
            for (int y = 1; y < dimY - 1; y++) {
                for (int x = 1; x < dimX - 1; x++) {
                    float gx = (volume.getVoxel(x + 1, y, z) - volume.getVoxel(x - 1, y, z)) / 2.0f;
                    float gy = (volume.getVoxel(x, y + 1, z) - volume.getVoxel(x, y - 1, z)) / 2.0f;
                    float gz = (volume.getVoxel(x, y, z + 1) - volume.getVoxel(x, y, z - 1)) / 2.0f;
                    VoxelGradient grad = new VoxelGradient(gx, gy, gz);
                    setGradient(x, y, z, grad);
                }
            }
        }
        maxmag = calculateMaxGradientMagnitude();
    }

    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
    //////////////////////////////////////////////////////////////////////
    //This function linearly interpolates gradient vector g0 and g1 given the factor (t)
    //the result is given at result. You can use it to tri-linearly interpolate the gradient
    public void interpolate(VoxelGradient g0, VoxelGradient g1, float factor, VoxelGradient result) {
        // to be implemented
        result.x = g0.x * (1 - factor) + g1.x * factor;
        result.y = g0.y * (1 - factor) + g1.y * factor;
        result.z = g0.z * (1 - factor) + g1.z * factor;
        result.mag = (float) Math.sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
    }

    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
    //////////////////////////////////////////////////////////////////////
    // This function should return linearly interpolated gradient for position coord[]
    // right now it returns the nearest neighbour.
    // This function returns the trilinear interpolated value of the position given by
    // position coord based on the value of the gradients around it.
    public VoxelGradient getGradient(double[] coord) {
        // to be implemented
        if (coord[0] < 0 || coord[0] > (dimX - 2) || coord[1] < 0 || coord[1] > (dimY - 2)
                || coord[2] < 0 || coord[2] > (dimZ - 2)) {
            return zero;
        }

        //Find the value for x,y,z coordinates just before our position for which we need to interpolate.
        int x = (int) Math.floor(coord[0]);
        int y = (int) Math.floor(coord[1]);
        int z = (int) Math.floor(coord[2]);

        //Find the factor by which we need to interpolate by using the values calculated above
        float fac_x = (float) coord[0] - x;
        float fac_y = (float) coord[1] - y;
        float fac_z = (float) coord[2] - z;

        //Need to initialise VoxelGradient objects to store resulting interpolations.
        VoxelGradient t0 = new VoxelGradient(0, 0, 0);
        VoxelGradient t1 = new VoxelGradient(0, 0, 0);
        VoxelGradient t2 = new VoxelGradient(0, 0, 0);
        VoxelGradient t3 = new VoxelGradient(0, 0, 0);
        VoxelGradient t4 = new VoxelGradient(0, 0, 0);
        VoxelGradient t5 = new VoxelGradient(0, 0, 0);
        VoxelGradient t6 = new VoxelGradient(0, 0, 0);

        // Need to interpolate first in x direction, then y and finally z to get the trilinear interpolation as done in the Volume.java file.
        interpolate(getGradient(x, y, z), getGradient(x + 1, y, z), fac_x, t0);
        interpolate(getGradient(x, y + 1, z), getGradient(x + 1, y + 1, z), fac_x, t1);
        interpolate(getGradient(x, y, z + 1), getGradient(x + 1, y, z + 1), fac_x, t2);
        interpolate(getGradient(x, y + 1, z + 1), getGradient(x + 1, y + 1, z + 1), fac_x, t3);
        interpolate(t0, t1, fac_y, t4);
        interpolate(t2, t3, fac_y, t5);
        interpolate(t4, t5, fac_z, t6);

        //returning the final interpolated value.
        return t6;
    }

    //Do NOT modify this function
    public VoxelGradient getGradientNN(double[] coord) {
        if (coord[0] < 0 || coord[0] > (dimX - 2) || coord[1] < 0 || coord[1] > (dimY - 2)
                || coord[2] < 0 || coord[2] > (dimZ - 2)) {
            return zero;
        }

        int x = (int) Math.round(coord[0]);
        int y = (int) Math.round(coord[1]);
        int z = (int) Math.round(coord[2]);
        return getGradient(x, y, z);
    }

    //Returns the maximum gradient magnitude
    //The data array contains all the gradients, in this function you have to return the maximum magnitude of the vectors in data[] 
    //Do NOT modify this function
    private double calculateMaxGradientMagnitude() {
        if (maxmag >= 0) {
            return maxmag;
        } else {
            double magnitude = data[0].mag;
            for (int i = 0; i < data.length; i++) {
                magnitude = data[i].mag > magnitude ? data[i].mag : magnitude;
            }
            maxmag = magnitude;
            return magnitude;
        }
    }

    //Do NOT modify this function
    public double getMaxGradientMagnitude() {
        return this.maxmag;
    }
    //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////

    //Do NOT modify this function
    public GradientVolume(Volume vol) {
        volume = vol;
        dimX = vol.getDimX();
        dimY = vol.getDimY();
        dimZ = vol.getDimZ();
        data = new VoxelGradient[dimX * dimY * dimZ];
        maxmag = -1.0;
        compute();
    }

    //Do NOT modify this function
    public VoxelGradient getGradient(int x, int y, int z) {
        return data[x + dimX * (y + dimY * z)];
    }

    //Do NOT modify this function
    public void setGradient(int x, int y, int z, VoxelGradient value) {
        data[x + dimX * (y + dimY * z)] = value;
    }

    //Do NOT modify this function
    public void setVoxel(int i, VoxelGradient value) {
        data[i] = value;
    }

    //Do NOT modify this function
    public VoxelGradient getVoxel(int i) {
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

}
