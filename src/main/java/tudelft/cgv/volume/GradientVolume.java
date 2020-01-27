/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tudelft.cgv.volume;

/**
 * It stores and provides methods to compute and retrieve gradient vector
 * In math, gradient vectors are vectors that point to the greatest increase of a function
 *
 * @author Michel Westenberg
 * @author Anna Vilanova
 * @author Andrea Alfieri
 * @author Reinier Koops
 * @author Aditya Kunar
 */
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

    /**
     * Function that linearly interpolates gradient vector g0 and g1 given the factor
     * (t). Used to tri-linearly interpolate the gradient
     *
     * @param g0     gradient vector
     * @param g1     gradient vector
     * @param factor ratio of importance
     * @param result the interpolation based on the two gradients
     */
    public void interpolate(VoxelGradient g0, VoxelGradient g1, float factor, VoxelGradient result) {
        result.x = g0.x * (1 - factor) + g1.x * factor;
        result.y = g0.y * (1 - factor) + g1.y * factor;
        result.z = g0.z * (1 - factor) + g1.z * factor;
        result.mag = (float) Math.sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
    }

    /**
     * Function that returns tri-linearly interpolated value of the position given by
     * position coord, based on the value of the gradients around it.
     *
     * @param coord the coordinate (x, y, z)
     * @return return tri-linearly interpolated value of the position
     */
    public VoxelGradient getGradient(double[] coord) {
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

        //This array contains all the temporary values and the final value to be returned for the gradients
        VoxelGradient[] voxels = new VoxelGradient[7];
        for(int i = 0; i < 7; i++)
            voxels[i] = new VoxelGradient();

        // Need to interpolate first in x direction, then y and finally z to get
        // the tri-linear interpolation as done in the Volume.java file.

        //Interpolate in 3 Dimensions
        interpolate(getGradient(x, y, z), getGradient(x + 1, y, z), fac_x, voxels[0]);
        interpolate(getGradient(x, y + 1, z), getGradient(x + 1, y + 1, z), fac_x, voxels[1]);
        interpolate(getGradient(x, y, z + 1), getGradient(x + 1, y, z + 1), fac_x, voxels[2]);
        interpolate(getGradient(x, y + 1, z + 1), getGradient(x + 1, y + 1, z + 1), fac_x, voxels[3]);

        //Interpolate in 2 dimensions
        interpolate(voxels[0], voxels[1], fac_y, voxels[4]);
        interpolate(voxels[2], voxels[3], fac_y, voxels[5]);

        //Interpolate on the last dimension
        interpolate(voxels[4], voxels[5], fac_z, voxels[6]);

        //returning the final interpolated value.
        return voxels[6];
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
