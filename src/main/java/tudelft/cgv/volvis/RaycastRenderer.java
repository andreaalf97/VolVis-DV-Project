// Version for students

package tudelft.cgv.volvis;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.util.texture.Texture;
import com.jogamp.opengl.util.texture.awt.AWTTextureIO;
import tudelft.cgv.gui.RaycastRendererPanel;
import tudelft.cgv.gui.TransferFunction2DEditor;
import tudelft.cgv.gui.TransferFunctionEditor;
import java.awt.image.BufferedImage;
import tudelft.cgv.util.TFChangeListener;
import tudelft.cgv.util.VectorMath;
import tudelft.cgv.volume.GradientVolume;
import tudelft.cgv.volume.Volume;
import tudelft.cgv.volume.VoxelGradient;

import java.awt.Color;


/**
 *
 * @author michel
 *  Edit by AVilanova & Nicola Pezzotti
 * 
 * 
 * Main functions to implement the volume rendering
 */


//////////////////////////////////////////////////////////////////////
///////////////// CONTAINS FUNCTIONS TO BE IMPLEMENTED ///////////////
//////////////////////////////////////////////////////////////////////

public class RaycastRenderer extends Renderer implements TFChangeListener {


// attributes
    
    private Volume volume = null;
    private GradientVolume gradients = null;
    RaycastRendererPanel panel;
    TransferFunction tFunc;
    TransferFunction2D tFunc2D;
    TransferFunctionEditor tfEditor;
    TransferFunction2DEditor tfEditor2D;
    private boolean mipMode = false;
    private boolean slicerMode = true;
    private boolean compositingMode = false;
    private boolean tf2dMode = false;
    private boolean shadingMode = false;
    private boolean isoMode = false;
    private float iso_value=95; 
    // This is a work around
    private float res_factor = 1.0f;
    private float max_res_factor=0.25f;
    private TFColor isoColor; 

    
    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE MODIFIED    /////////////////////////
    ////////////////////////////////////////////////////////////////////// 
    //Function that updates the "image" attribute (result of renderings)
    // using the slicing technique. 
    
    public void slicer(double[] viewMatrix) {
	
        // we start by clearing the image
        resetImage();

        // vector uVec and vVec define the view plane, 
        // perpendicular to the view vector viewVec which is going from the view point towards the object
        // uVec contains the up vector of the camera in world coordinates (image vertical)
        // vVec contains the horizontal vector in world coordinates (image horizontal)
        double[] viewVec = new double[3];
        double[] uVec = new double[3];
        double[] vVec = new double[3];
        getViewPlaneVectors(viewMatrix,viewVec,uVec,vVec);

        // The result of the visualization is saved in an image(texture)
        // we update the vector according to the resolution factor
        // If the resolution is 0.25 we will sample 4 times more points. 
        for(int k=0;k<3;k++)
        {
            uVec[k]=res_factor*uVec[k];
            vVec[k]=res_factor*vVec[k];
        }

        // compute the volume center
        double[] volumeCenter = new double[3];
        computeVolumeCenter(volumeCenter);

        // Here will be stored the 3D coordinates of every pixel in the plane 
        double[] pixelCoord = new double[3];

        // We get the size of the image/texture we will be puting the result of the 
        // volume rendering operation.
        int imageW=image.getWidth();
        int imageH=image.getHeight();

        int[] imageCenter = new int[2];
        // Center of the image/texture 
        imageCenter[0]= imageW/2;
        imageCenter[1]= imageH/2;
        
        // imageW/ image H contains the real width of the image we will use given the resolution. 
        //The resolution is generated once based on the maximum resolution.
        imageW = (int) (imageW*((max_res_factor/res_factor)));
        imageH = (int) (imageH*((max_res_factor/res_factor)));

        // sample on a plane through the origin of the volume data
        double max = volume.getMaximum();

        // Color that will be used as a result 
        TFColor pixelColor = new TFColor();
        // Auxiliar color
        TFColor colorAux;

        // Contains the voxel value of interest
        int val;
        
        //Iterate on every pixel
        for (int j = imageCenter[1] - imageH/2; j < imageCenter[1] + imageH/2; j++) {
            for (int i =  imageCenter[0] - imageW/2; i <imageCenter[0] + imageW/2; i++) {
                
                // computes the pixelCoord which contains the 3D coordinates of the pixels (i,j)
                computePixelCoordinatesFloat(pixelCoord,volumeCenter,uVec,vVec,i,j);
                
                //we now have to get the value for the in the 3D volume for the pixel
                //we can use a nearest neighbor implementation like this:
                //val = volume.getVoxelNN(pixelCoord);
                
                //you have also the function getVoxelLinearInterpolated in Volume.java          
                val = (int) volume.getVoxelLinearInterpolate(pixelCoord);
                
                //you have to implement this function below to get the cubic interpolation
                //val = (int) volume.getVoxelTriCubicInterpolate(pixelCoord);
                
                
                // Map the intensity to a grey value by linear scaling
                 pixelColor.r = (val/max);
                 pixelColor.g = pixelColor.r;
                 pixelColor.b = pixelColor.r;

                // the following instruction makes intensity 0 completely transparent and the rest opaque
                //pixelColor.a = val > 0 ? 1.0 : 0.0;
                
                // Alternatively, apply the transfer function to obtain a color using the tFunc attribute
                //colorAux= tFunc.getColor(val);
                //pixelColor.r=colorAux.r;pixelColor.g=colorAux.g;pixelColor.b=colorAux.b;pixelColor.a=colorAux.a;
              
                // IMPORTANT: You can also simply use pixelColor = tFunc.getColor(val); However then you copy by reference and this means that if you change 
                // pixelColor you will be actually changing the transfer function So BE CAREFUL when you do this kind of assignments

                //BufferedImage/image/texture expects a pixel color packed as ARGB in an int
                //use the function computeImageColor to convert your double color in the range 0-1 to the format need by the image
                int pixelColor_i = computeImageColor(pixelColor.r,pixelColor.g,pixelColor.b,pixelColor.a);
                image.setRGB(i, j, pixelColor_i);
            }
        }
}
    

    
    //Do NOT modify this function
    //
    //Function that updates the "image" attribute using the MIP raycasting
    //It returns the color assigned to a ray/pixel given it's starting point (entryPoint) and the direction of the ray(rayVector).
    // exitPoint is the last point.
    //ray must be sampled with a distance defined by the sampleStep
   
    int traceRayMIP(double[] entryPoint, double[] exitPoint, double[] rayVector, double sampleStep) {
    	//compute the increment and the number of samples
        double[] increments = new double[3];
        VectorMath.setVector(increments, rayVector[0] * sampleStep, rayVector[1] * sampleStep, rayVector[2] * sampleStep);
        
        // Compute the number of times we need to sample
        double distance = VectorMath.distance(entryPoint, exitPoint);
        int nrSamples = 1 + (int) Math.floor(VectorMath.distance(entryPoint, exitPoint) / sampleStep);

        //the current position is initialized as the entry point
        double[] currentPos = new double[3];
        VectorMath.setVector(currentPos, entryPoint[0], entryPoint[1], entryPoint[2]);
       
        double maximum = 0;
        do {
            double value = volume.getVoxelLinearInterpolate(currentPos)/255.; 
            if (value > maximum) {
                maximum = value;
            }
            for (int i = 0; i < 3; i++) {
                currentPos[i] += increments[i];
            }
            nrSamples--;
        } while (nrSamples > 0);

        double alpha;
        double r, g, b;
        if (maximum > 0.0) { // if the maximum = 0 make the voxel transparent
            alpha = 1.0;
        } else {
            alpha = 0.0;
        }
        r = g = b = maximum;
        int color = computeImageColor(r,g,b,alpha);
        return color;
    }
    
          
    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
    ////////////////////////////////////////////////////////////////////// 
    //Function that updates the "image" attribute using the Isosurface raycasting
    //It returns the color assigned to a ray/pixel given it's starting point (entryPoint) and the direction of the ray(rayVector).
    // exitPoint is the last point.
    //ray must be sampled with a distance defined by the sampleStep
    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
    ////////////////////////////////////////////////////////////////////// 
    //Function that updates the "image" attribute using the Isosurface raycasting
    //It returns the color assigned to a ray/pixel given it's starting point (entryPoint) and the direction of the ray(rayVector).
    // exitPoint is the last point.
    //ray must be sampled with a distance defined by the sampleStep
   
   public int traceRayIso(double[] entryPoint, double[] exitPoint, double[] rayVector, double sampleStep) {
       
       
       //In case we are moving the image, we increase sampling steps to reduce the number of points being sampled, reducing computation when in interactive mode.
       if (interactiveMode == true){
            sampleStep = sampleStep * 3;
        }
        //We define the light vector as directed toward the view point (which is the source of the light)
        // another light vector would be possible
        double[] lightVector = new double[3];
        VectorMath.setVector(lightVector, rayVector[0]* sampleStep, rayVector[1]* sampleStep, rayVector[2]* sampleStep);
                
        //compute the increment 
        double[] increments = new double[3];
        VectorMath.setVector(increments, rayVector[0] * sampleStep, rayVector[1] * sampleStep, rayVector[2] * sampleStep);
        
        // Compute the number of times we need to sample
        int nrSamples = 1 + (int) Math.floor( VectorMath.distance(entryPoint, exitPoint)/ sampleStep);

        //the current position is initialized as the entry point
        double[] currentPos = new double[3];
        VectorMath.setVector(currentPos, entryPoint[0], entryPoint[1], entryPoint[2]);
       
        // initialise the alpha.
        double alpha = 0.0;

        //Same loop as in traceRayMIP to iterate over the ray.
        do {
            //Get the interpolated value for the current position in the ray.
            float value = volume.getVoxelLinearInterpolate(currentPos);
            //If we hit the isoValue then we break, everything from this point on is opaque. 
            if (value == getIsoValue()) {
                alpha = 1.0;
                break;
            }
            //If we want to be more precise, we use a binary search within two sample steps to get a more accurate position for our isoValue.
            // If the value for our current position is greater than isoValue we are looking for, the actual position must be inbetween the current position and the previous position. 
            else if (value > getIsoValue() && currentPos!=entryPoint) {
                currentPos = bisection_accuracy(currentPos, increments, iso_value);
                alpha = 1.0;
                break;
            }
            //Updating our current position in the ray.
            for (int i = 0; i < 3; i++) {
                currentPos[i] += increments[i];
            }
            nrSamples--;
        } while (nrSamples > 0);
        
        //initialise the color by assigning it the isoColor values.
        int color;
        
        double r, g, b;
        r = isoColor.r;
        g = isoColor.g;
        b = isoColor.b;
        
        //If we enable shading, we compute the phong shading and return that as the color else we return the unshaded color. 
        if (shadingMode == true) {
            TFColor normal = new TFColor(r, g, b, alpha);
            VoxelGradient gradient = gradients.getGradient(currentPos);
            //Pass the gradient and the normal color to the function to get the shaded color.
            TFColor phong_color = computePhongShading(normal, gradient, lightVector, rayVector);
            // using the shaded color to compute the final image color.
            color = computeImageColor(phong_color.r, phong_color.g, phong_color.b, phong_color.a);
        } else {
            color = computeImageColor(r, g, b, alpha);
        }
        return color;
    }
   
    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
    ////////////////////////////////////////////////////////////////////// 
    
    // Given the current sample position, increment vector of the sample (vector from previous sample to current sample) and sample Step. 
   // Previous sample value and current sample value, isovalue value
    // The function should search for a position where the iso_value passes that it is more precise.
    public double[]  bisection_accuracy (double[] currentPos, double[] increments, float iso_value) {

        // to be implemented
        //Initialise variables for whether we found the value(v_f), the mid point between two sampled points in the ray and the value associated with it.
        float mid_val=0;
        boolean v_f = false;
        double[] prevPos = new double[3];
        double[] mid_point = new double[3];
        //Get the previous position.
        for(int i=0;i<3 ;i++)
        {
           prevPos[i]= currentPos[i] - increments[i];
        };
        // Doing a binary search to find the accurate position within a threshold of 0.001. Need this condition because it's possible we will never find the actual position.   
        // Running the loop as long as we dont find the value or the threshold is greater than 0.001.
        while (v_f==false && (mid_val - iso_value)>0.001) {
            //Compute the mid point between previous and current position.
            for(int i=0; i< 3;i++) {
                mid_point[i]=(currentPos[i] + prevPos[i])/ 2;
                          };
            //get the value through interpolation for the mid point.              
            mid_val = volume.getVoxelLinearInterpolate(mid_point);
           
            //In the case we found it, great, lets stop and assign this mid point as the more accurate position for our iso value.
            if (mid_val == iso_value) {
                currentPos = mid_point;
                v_f = true;
            } 
            // Shucks, we didn't find it, but it looks like we might find it in the region between the prev position and the current mid_point.
            else if (iso_value < mid_val) {
                currentPos = mid_point;
            }
              // Shucks, we didn't find it, but it looks like we might find it in the region between the current position and the current mid_point.
            else if (mid_val < iso_value) {
                prevPos = mid_point;
            }
   
        }  
       //returning the more accurate position. 
        return currentPos;
    }
    
    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
    ////////////////////////////////////////////////////////////////////// 
    //Function that updates the "image" attribute using the compositing// accumulatted raycasting
    //It returns the color assigned to a ray/pixel given it's starting point (entryPoint) and the direction of the ray(rayVector).
    // exitPoint is the last point.
    //ray must be sampled with a distance defined by the sampleStep
   
    public int traceRayComposite(double[] entryPoint, double[] exitPoint, double[] rayVector, double sampleStep) {
        double[] lightVector = new double[3];
        
        //the light vector is directed toward the view point (which is the source of the light)
        // another light vector would be possible 
        VectorMath.setVector(lightVector, rayVector[0]* sampleStep, rayVector[1]* sampleStep, rayVector[2]* sampleStep);

        //In case we are moving the image, we increase sampling steps to reduce the number of points being sampled, reducing computation when in interactive mode.
        if (interactiveMode == true){
            sampleStep = sampleStep * 3;
        }
        
        //Initialization of the colors as floating point values
        double r, g, b;
        r = g = b = 0.0;
        double alpha = 0.0;
        double alph = 0;
        TFColor voxel_color = new TFColor();
        
        
        // To be Implemented this function right now just gives back a constant color depending on the mode
        
        if (compositingMode) {
            // 1D transfer function
            TFColor colorAux = rayCompositing(entryPoint,exitPoint, lightVector, rayVector, sampleStep);

            voxel_color.r = colorAux.r;
            voxel_color.g = colorAux.g;
            voxel_color.b = colorAux.b;

            //If any of the colors are positive, assign the opacity as 1.
            if (colorAux.r > 0 || colorAux.g > 0 || colorAux.b > 0)
                alph = 1;

        }    
        if (tf2dMode) {
             // 2D transfer function 
          // Computes the ray compositing based on the 2d transfer function and returns the final accumalated color.
           TFColor colorAux;
           colorAux = rayCompositing2D(entryPoint,exitPoint,lightVector, rayVector,sampleStep);
           //assign the local colors to the global colors.
           voxel_color.r = colorAux.r;
           voxel_color.g = colorAux.g;
           voxel_color.b = colorAux.b;
           
            //If any of the colors are positive, assign the opacity as 1.
           if (colorAux.r > 0 || colorAux.g > 0 || colorAux.b > 0) {
               alph = colorAux.a;
            }
             }
        
        r = voxel_color.r;
        g = voxel_color.g;
        b = voxel_color.b;
        alpha = alph;
        //computes the color
        int color = computeImageColor(r,g,b,alpha);
        return color;
    }

    ////////////////////////
    // We do back to front ray compositing in this function.
     TFColor rayCompositing(double[] entryPoint, double[] exitPoint, double[] lightVector, double[] rayVector, double sampleStep) {

       
        // Compute the number of times we need to sample
        int nrSamples = 1 + (int) Math.floor(VectorMath.distance(entryPoint, exitPoint) / sampleStep);

        //the current position is initialized as the exit point
        double[] currentPos = new double[3];
        VectorMath.setVector(currentPos, exitPoint[0], exitPoint[1], exitPoint[2]);

        //Computes the increment.
        double[] increments = new double[3];
        VectorMath.setVector(increments, rayVector[0] * sampleStep, rayVector[1] * sampleStep, rayVector[2] * sampleStep);

        //Compute the interpolated at the current position
        int value = (int) volume.getVoxelLinearInterpolate(currentPos);

        //Get the color for the value computed above.
        TFColor prevColor = this.tFunc.getColor(value);
        
        //We use the phong shaded color if shading is enabled.
        if (shadingMode){
        // If we dont have a value for red or green or blue we dont need to do the phong shading.
        if(prevColor.r > 0 || prevColor.g > 0 || prevColor.b > 0){
        
            prevColor = computePhongShading(prevColor, gradients.getGradient(currentPos), lightVector, rayVector);
        
        }
        
        }
        
        //Initialise the color to be accumalated. 
        TFColor accColor = new TFColor(0,0,0,0);
        
        //Loop through the ray back to front.
        do {
            //Update the position.
            for (int i = 0; i < 3; i++){
                currentPos[i] -= increments[i];
            }
            //Get the value and the color based on the value for this position, if phong shading is enabled, then get the shaded color.
            value = (int) volume.getVoxelLinearInterpolate(currentPos);
            TFColor currColor = this.tFunc.getColor(value);
            
            if (shadingMode){
            // If we dont have a value for red or green or blue we dont need to do the phong shading.
            if(currColor.r > 0 || currColor.g > 0 || currColor.b > 0){

            currColor = computePhongShading(currColor, gradients.getGradient(currentPos), lightVector, rayVector);
   
            }
            
            }
            
           //Calculate the accumalated color based on the current color and the previously accumalated color. 
           accColor.r = currColor.a * currColor.r + (1 - currColor.a) * prevColor.r;

           accColor.g = currColor.a * currColor.g + (1 - currColor.a) * prevColor.g;

           accColor.b = currColor.a * currColor.b + (1 - currColor.a) * prevColor.b;
            
            //Here we assign the prevColor in our loop to the new accumalated color to ensure the recursion is executed as we want it to.
            prevColor=accColor;

            nrSamples--;

        } while (nrSamples > 1);

        return accColor;

     }
     
     // In this function we go front to back.
    TFColor rayCompositing2D(double[] entryPoint, double[] exitPoint,double[] lightVector ,double[] rayVector, double sampleStep) {
        // Compute the number of times we need to sample
        int nrSamples = 1 + (int) Math.floor(VectorMath.distance(entryPoint, exitPoint) / sampleStep);
        
        //the current position is set as the entry point
        double[] currentPos = new double[3];
        VectorMath.setVector(currentPos, entryPoint[0], entryPoint[1], entryPoint[2]);
        
        //Compute the increment
        double[] increments = new double[3];
        VectorMath.setVector(increments, rayVector[0] * sampleStep, rayVector[1] * sampleStep, rayVector[2] * sampleStep);
        
        // Initialse the color variables.
        TFColor accColor = new TFColor(0,0,0.0,0.0);
        TFColor colorAux = new TFColor(0.0,0.0,0.0,0.0);
        
        double alpha = 0.0;
        
        for (int i = 0; i < nrSamples; i++) {
          // if the color accumalated so far has opacity 0.9 or above, we can stop as no new color sampled will be visible.
            if(accColor.a >= 0.90){
                break;
            }
            // Get the value for the current position.
            double value = volume.getVoxelLinearInterpolate(currentPos);
            // Get the gradient for the current position.
            VoxelGradient gradient = gradients.getGradient(currentPos);
            //Compute the color based on 2d Transfer function. 
            colorAux = tFunc2D.color;
            //Compute the opacity
            alpha = this.computeOpacity2DTF(tFunc2D.baseIntensity, tFunc2D.radius,value, gradient.mag);
            // If shading is enabled, we compute the phong shaded color.
            if (shadingMode) {
                if (alpha > 0.0f && colorAux.r > 0.0f && colorAux.g > 0.0f && colorAux.b > 0.0f) {
                    TFColor color = new TFColor(colorAux.r, colorAux.g, colorAux.b, alpha);
                    colorAux = computePhongShading(color, gradient, lightVector, rayVector);
                    alpha = colorAux.a;
                }
             }
            //Calculate the accumated color as we iterate through the ray based on front to back compositing. 
            accColor.r += (1.0 - accColor.a) * alpha * colorAux.r;
            accColor.g += (1.0 - accColor.a) * alpha * colorAux.g;
            accColor.b += (1.0 - accColor.a) * alpha * colorAux.b;
            //We also take account of the opacity in this case.
            accColor.a += (1.0 - accColor.a) * alpha;
            //Update the current position 
            for(int j = 0; j < 3; j++){
                currentPos[j] += increments[j];
            }
        }
        
        return accColor;
        
     }
    
    //////////////////////////////////////////////////////////////////////
    ///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
    ////////////////////////////////////////////////////////////////////// 
    // Compute Phong Shading given the voxel color (material color), the gradient, the light vector and view vector 
    public TFColor computePhongShading(TFColor voxel_color, VoxelGradient gradient, double[] lightVector,
            double[] rayVector) {

        // To be implemented 
        // Set the values for the phong shading model.
        float alpha_p = 100.0f;
        float la = 1.0f;
        float ld = 1.0f;
        float ls = 1.0f;
        float ka = 0.1f;
        float kd = 0.7f;
        float ks = 0.2f;
        
        
        //Initialse the color variable. 
        TFColor color = new TFColor(0, 0, 0, voxel_color.a);

        // Compute the ambient color.
        TFColor ambient = new TFColor (0,0,0,0);
        ambient.r = la*ka*voxel_color.r;
        ambient.g = la*ka*voxel_color.g;
        ambient.b = la*ka*voxel_color.b;
        
        
        // diffuse
        
        //Normalise the light vector.
        double lightVectorNorm = VectorMath.length(lightVector);
        for( int i =0; i<3;i++){
            lightVector[i] = lightVector[i]/(lightVectorNorm+.000000001);
        }
        
        //Normalise the ray vector.
        double rayVectorNorm = VectorMath.length(rayVector);
        for( int i =0; i<3;i++){
            rayVector[i] = rayVector[i]/(rayVectorNorm+.000000001);
        }
        
        //Normlaise the normal vector. 
        double[] normalVector = new double[3];
        normalVector[0] = gradient.x / (gradient.mag+.000000001);
        normalVector[1] = gradient.y / (gradient.mag+.000000001);
        normalVector[2] = gradient.z / (gradient.mag+.000000001);
        
        //Compute cos of the angle between normal vector(Vector perpendicular to the surface) and light vector(Direction of light source).
        // All vectors are normalised so the dot product just gives the value of the numerator ie cos(angle)
        double cos_theta = VectorMath.dotproduct(normalVector, lightVector);
        
        //Compute the diffuse color.
        TFColor diffuse = new TFColor (0,0,0,0);
        diffuse.r = (ld*kd*voxel_color.r*cos_theta);
        diffuse.g = (ld*kd*voxel_color.g*cos_theta);
        diffuse.b = (ld*kd*voxel_color.b*cos_theta);
        
        //Specular
        //Calculate twice of the normal vector.
        double twice_normalVec[] = new double[3];
        
        for (int i=0; i< 3; i++)
        {
        twice_normalVec[i] = normalVector[i]*2;
        }
        
        //Compute cos of the angle between twice the normal vector(Vector perpendicular to the surface) and light vector(Direction of light source).
        // All vectors are normalised so the dot product just gives the value of the numerator ie cos(angle)
        double cos_phi = VectorMath.dotproduct(twice_normalVec, lightVector);
        
        //Computing R vector.
        double R[] = new double[3];
        
        for (int i =0;i<3;i++) {
            R[i] = cos_phi*normalVector[i] - lightVector[i];
        }
        // Computing cos of the angle between ray vector and R. All vectors are normalised so the dot product just gives the value of the numerator ie cos(angle).
        double cos_psy = VectorMath.dotproduct(rayVector, R);
        
        //Compute the specular color.
        TFColor specular = new TFColor(0,0,0,0);
        specular.r = (ls*ks*Math.pow(cos_psy, alpha_p));
        specular.g = (ls*ks*Math.pow(cos_psy, alpha_p));
        specular.b = (ls*ks*Math.pow(cos_psy, alpha_p));
        
        //Compute the final color.
        color.r = ambient.r + diffuse.r + specular.r;
        color.g = ambient.g + diffuse.g + specular.g;
        color.b = ambient.b + diffuse.b + specular.b;
        
        // Limit the color range between 0 and 1. 
        if (color.r < 0) {

            color.r = 0;

        }
        
        if (color.r > 1) {

            color.r = 1;

        }

        if (color.g < 0) {

            color.g = 0;

        }

          if (color.g > 1) {

            color.g = 1;

        }
        
        if (color.b < 0) {

            color.b = 0;

        }

        if (color.b > 1) {

            color.b = 1;

        }
        

        return color;
        
       
    }
    
    
    //////////////////////////////////////////////////////////////////////
    ///////////////// LIMITED MODIFICATION IS NEEDED /////////////////////
    ////////////////////////////////////////////////////////////////////// 
    // Implements the basic tracing of rays trough the image and given the
    // camera transformation
    // It calls the functions depending on the raycasting mode
  
    public void raycast(double[] viewMatrix) {

    	//data allocation
        double[] viewVec = new double[3];
        double[] uVec = new double[3];
        double[] vVec = new double[3];
        double[] pixelCoord = new double[3];
        double[] entryPoint = new double[3];
        double[] exitPoint = new double[3];
        
        // increment in the pixel domain in pixel units
        int increment = 1;
        // sample step in voxel units
        int sampleStep = 1;
        // reset the image to black
        resetImage();
        
        // vector uVec and vVec define the view plane, 
        // perpendicular to the view vector viewVec which is going from the view point towards the object
        // uVec contains the up vector of the camera in world coordinates (image vertical)
        // vVec contains the horizontal vector in world coordinates (image horizontal)
        getViewPlaneVectors(viewMatrix,viewVec,uVec,vVec);
        
        
        // The result of the visualization is saved in an image(texture)
        // we update the vector according to the resolution factor
        // If the resolution is 0.25 we will sample 4 times more points. 
        for(int k=0;k<3;k++)
        {
            uVec[k]=res_factor*uVec[k];
            vVec[k]=res_factor*vVec[k];
        }
        
       // We get the size of the image/texture we will be puting the result of the 
        // volume rendering operation.
        int imageW=image.getWidth();
        int imageH=image.getHeight();

        int[] imageCenter = new int[2];
        // Center of the image/texture 
        imageCenter[0]= imageW/2;
        imageCenter[1]= imageH/2;
        
        // imageW/ image H contains the real width of the image we will use given the resolution. 
        //The resolution is generated once based on the maximum resolution.
        imageW = (int) (imageW*((max_res_factor/res_factor)));
        imageH = (int) (imageH*((max_res_factor/res_factor)));
        
        //The rayVector is pointing towards the scene
        double[] rayVector = new double[3];
        rayVector[0]=-viewVec[0];rayVector[1]=-viewVec[1];rayVector[2]=-viewVec[2];
             
        // compute the volume center
        double[] volumeCenter = new double[3];
        computeVolumeCenter(volumeCenter);

        
        // ray computation for each pixel
        for (int j = imageCenter[1] - imageH/2; j < imageCenter[1] + imageH/2; j += increment) {
            for (int i =  imageCenter[0] - imageW/2; i <imageCenter[0] + imageW/2; i += increment) {
                // compute starting points of rays in a plane shifted backwards to a position behind the data set
            	computePixelCoordinatesBehindFloat(pixelCoord,viewVec,uVec,vVec,i,j);
            	// compute the entry and exit point of the ray
                computeEntryAndExit(pixelCoord, rayVector, entryPoint, exitPoint);
                if ((entryPoint[0] > -1.0) && (exitPoint[0] > -1.0)) {
                    int val = 0;
                    if (compositingMode || tf2dMode) {
                        val = traceRayComposite(entryPoint, exitPoint, rayVector, sampleStep);
                    } else if (mipMode) {
                        val = traceRayMIP(entryPoint, exitPoint, rayVector, sampleStep);
                    } else if (isoMode){
                        val= traceRayIso(entryPoint,exitPoint,rayVector, sampleStep);
                    }
                    for (int ii = i; ii < i + increment; ii++) {
                        for (int jj = j; jj < j + increment; jj++) {
                            image.setRGB(ii, jj, val);
                        }
                    }
                }

            }
        }
    }


//////////////////////////////////////////////////////////////////////
///////////////// FUNCTION TO BE IMPLEMENTED /////////////////////////
////////////////////////////////////////////////////////////////////// 
// Compute the opacity based on the value of the pixel and the values of the
// triangle widget tFunc2D contains the values of the baseintensity and radius
// tFunc2D.baseIntensity, tFunc2D.radius they are in image intensity units
 public double computeOpacity2DTF(short intensity,double radius, double voxelValue, double gradMagnitude) {

    //init opacity with 0

    double opacity = 0.0;

    //angle of the widget

    double angle = Math.atan(radius/gradients.getMaxGradientMagnitude());
  
    //angle of current voxel with respect to base intensity center

    double voxelRad = Math.abs(voxelValue-intensity);

    double voxelGradMag = gradMagnitude;

    double voxelAngle = Math.atan(voxelRad/voxelGradMag);

    //if the voxel is inside the widget, give it an opacity

    if(voxelAngle < angle){

        //the factor between the angles is used as a ramp

        opacity = (1 - (voxelAngle/angle))*tFunc2D.color.a;  
        
    }

    return opacity;

} 

/*
    public double computeOpacity2DTF(double material_value, double material_r, double voxelValue, double gradMagnitude) {

    double opacity = 0.0;
    if (gradMagnitude == 0.0 && voxelValue == material_value) {
        opacity = 1.0;
    } else if (gradMagnitude > 0.0 && voxelValue - material_r * gradMagnitude <= material_value
            && material_value <= voxelValue + material_r * gradMagnitude) {
        opacity = 1.0 - (1/material_r)*Math.abs((material_value - voxelValue)/gradMagnitude);
    }

    return opacity;
} 

*/
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  
    
    //Do NOT modify this function
    int computeImageColor(double r, double g, double b, double a){
		int c_alpha = 	a <= 1.0 ? (int) Math.floor(a * 255) : 255;
        int c_red = 	r <= 1.0 ? (int) Math.floor(r * 255) : 255;
        int c_green = 	g <= 1.0 ? (int) Math.floor(g * 255) : 255;
        int c_blue = 	b <= 1.0 ? (int) Math.floor(b * 255) : 255;
        int pixelColor = getColorInteger(c_red,c_green,c_blue,c_alpha);
        return pixelColor;
	}
    //Do NOT modify this function    
    public void resetImage(){
    	for (int j = 0; j < image.getHeight(); j++) {
	        for (int i = 0; i < image.getWidth(); i++) {
	            image.setRGB(i, j, 0);
	        }
	    }
    }
   
    //Do NOT modify this function
    void computeIncrementsB2F(double[] increments, double[] rayVector, double sampleStep) {
        // we compute a back to front compositing so we start increments in the oposite direction than the pixel ray
    	VectorMath.setVector(increments, -rayVector[0] * sampleStep, -rayVector[1] * sampleStep, -rayVector[2] * sampleStep);
    }
    
    //used by the slicer
    //Do NOT modify this function
    void getViewPlaneVectors(double[] viewMatrix, double viewVec[], double uVec[], double vVec[]) {
            VectorMath.setVector(viewVec, viewMatrix[2], viewMatrix[6], viewMatrix[10]);
	    VectorMath.setVector(uVec, viewMatrix[0], viewMatrix[4], viewMatrix[8]);
	    VectorMath.setVector(vVec, viewMatrix[1], viewMatrix[5], viewMatrix[9]);
	}
    
    //used by the slicer	
    //Do NOT modify this function
    void computeVolumeCenter(double volumeCenter[]) {
	VectorMath.setVector(volumeCenter, volume.getDimX() / 2, volume.getDimY() / 2, volume.getDimZ() / 2);
    }
	
    //used by the slicer
    //Do NOT modify this function
    void computePixelCoordinatesFloat(double pixelCoord[], double volumeCenter[], double uVec[], double vVec[], float i, float j) {
        // Coordinates of a plane centered at the center of the volume (volumeCenter and oriented according to the plane defined by uVec and vVec
            float imageCenter = image.getWidth()/2;
            pixelCoord[0] = uVec[0] * (i - imageCenter) + vVec[0] * (j - imageCenter) + volumeCenter[0];
            pixelCoord[1] = uVec[1] * (i - imageCenter) + vVec[1] * (j - imageCenter) + volumeCenter[1];
            pixelCoord[2] = uVec[2] * (i - imageCenter) + vVec[2] * (j - imageCenter) + volumeCenter[2];
    }
        
    //Do NOT modify this function
    void computePixelCoordinates(double pixelCoord[], double volumeCenter[], double uVec[], double vVec[], int i, int j) {
        // Coordinates of a plane centered at the center of the volume (volumeCenter and oriented according to the plane defined by uVec and vVec
            int imageCenter = image.getWidth()/2;
            pixelCoord[0] = uVec[0] * (i - imageCenter) + vVec[0] * (j - imageCenter) + volumeCenter[0];
            pixelCoord[1] = uVec[1] * (i - imageCenter) + vVec[1] * (j - imageCenter) + volumeCenter[1];
            pixelCoord[2] = uVec[2] * (i - imageCenter) + vVec[2] * (j - imageCenter) + volumeCenter[2];
    }
    //Do NOT modify this function
    void computePixelCoordinatesBehindFloat(double pixelCoord[], double viewVec[], double uVec[], double vVec[], float i, float j) {
            int imageCenter = image.getWidth()/2;
            // Pixel coordinate is calculate having the center (0,0) of the view plane aligned with the center of the volume and moved a distance equivalent
            // to the diaganal to make sure I am far away enough.

            double diagonal = Math.sqrt((volume.getDimX()*volume.getDimX())+(volume.getDimY()*volume.getDimY())+ (volume.getDimZ()*volume.getDimZ()))/2;               
            pixelCoord[0] = uVec[0] * (i - imageCenter) + vVec[0] * (j - imageCenter) + viewVec[0] * diagonal + volume.getDimX() / 2.0;
            pixelCoord[1] = uVec[1] * (i - imageCenter) + vVec[1] * (j - imageCenter) + viewVec[1] * diagonal + volume.getDimY() / 2.0;
            pixelCoord[2] = uVec[2] * (i - imageCenter) + vVec[2] * (j - imageCenter) + viewVec[2] * diagonal + volume.getDimZ() / 2.0;
    }
    //Do NOT modify this function
    void computePixelCoordinatesBehind(double pixelCoord[], double viewVec[], double uVec[], double vVec[], int i, int j) {
            int imageCenter = image.getWidth()/2;
            // Pixel coordinate is calculate having the center (0,0) of the view plane aligned with the center of the volume and moved a distance equivalent
            // to the diaganal to make sure I am far away enough.

            double diagonal = Math.sqrt((volume.getDimX()*volume.getDimX())+(volume.getDimY()*volume.getDimY())+ (volume.getDimZ()*volume.getDimZ()))/2;               
            pixelCoord[0] = uVec[0] * (i - imageCenter) + vVec[0] * (j - imageCenter) + viewVec[0] * diagonal + volume.getDimX() / 2.0;
            pixelCoord[1] = uVec[1] * (i - imageCenter) + vVec[1] * (j - imageCenter) + viewVec[1] * diagonal + volume.getDimY() / 2.0;
            pixelCoord[2] = uVec[2] * (i - imageCenter) + vVec[2] * (j - imageCenter) + viewVec[2] * diagonal + volume.getDimZ() / 2.0;
    }
	
    //Do NOT modify this function
    public int getColorInteger(int c_red, int c_green, int c_blue, int c_alpha) {
    	int pixelColor = (c_alpha << 24) | (c_red << 16) | (c_green << 8) | c_blue;
    	return pixelColor;
    } 
    //Do NOT modify this function
    public RaycastRenderer() {
        panel = new RaycastRendererPanel(this);
        panel.setSpeedLabel("0");
        isoColor = new TFColor();
        isoColor.r=1.0;isoColor.g=1.0;isoColor.b=0.0;isoColor.a=1.0;
    }
    
     
    //Do NOT modify this function
    public void setVolume(Volume vol) {
        System.out.println("Assigning volume");
        volume = vol;

        System.out.println("Computing gradients");
        gradients = new GradientVolume(vol);

        // set up image for storing the resulting rendering
        // the image width and height are equal to the length of the volume diagonal
        int imageSize = (int) Math.floor(Math.sqrt(vol.getDimX() * vol.getDimX() + vol.getDimY() * vol.getDimY()
                + vol.getDimZ() * vol.getDimZ())* (1/max_res_factor));
        if (imageSize % 2 != 0) {
            imageSize = imageSize + 1;
        }
        
        image = new BufferedImage(imageSize, imageSize, BufferedImage.TYPE_INT_ARGB);
      
                  
           
        // Initialize transferfunction 
        tFunc = new TransferFunction(volume.getMinimum(), volume.getMaximum());
        tFunc.setTestFunc();
        tFunc.addTFChangeListener(this);
        tfEditor = new TransferFunctionEditor(tFunc, volume.getHistogram());
        
        tFunc2D= new TransferFunction2D((short) (volume.getMaximum() / 2), 0.2*volume.getMaximum());
        tfEditor2D = new TransferFunction2DEditor(tFunc2D,volume, gradients);
        tfEditor2D.addTFChangeListener(this);

        System.out.println("Finished initialization of RaycastRenderer");
    }
    //Do NOT modify this function
    public RaycastRendererPanel getPanel() {
        return panel;
    }

    //Do NOT modify this function
    public TransferFunction2DEditor getTF2DPanel() {
        return tfEditor2D;
    }
    //Do NOT modify this function
    public TransferFunctionEditor getTFPanel() {
        return tfEditor;
    }
    //Do NOT modify this function
    public void setShadingMode(boolean mode) {
        shadingMode = mode;
        changed();
    }
    //Do NOT modify this function
    public void setMIPMode() {
        setMode(false, true, false, false,false);
    }
    //Do NOT modify this function
    public void setSlicerMode() {
        setMode(true, false, false, false,false);
    }
    //Do NOT modify this function
    public void setCompositingMode() {
        setMode(false, false, true, false,false);
    }
    //Do NOT modify this function
    public void setTF2DMode() {
        setMode(false, false, false, true, false);
    }
    //Do NOT modify this function
    public void setIsoSurfaceMode(){
        setMode(false, false, false, false, true);
     }
    //Do NOT modify this function
    public void setIsoValue(float pIsoValue){
         iso_value = pIsoValue;
         if (isoMode){
             changed();
         }
             
     }
    //Do NOT modify this function
    public void setResFactor(int value) {
         float newRes= 1.0f/value;
         if (res_factor != newRes)
         {
             res_factor=newRes;
             if (volume != null) changed();
         }
     }
     
   //Do NOT modify this function
   public void setIsoColor(TFColor newColor)
     {
         this.isoColor.r=newColor.r;
         this.isoColor.g=newColor.g;
         this.isoColor.b=newColor.b;
         if ((volume!=null) && (this.isoMode)) changed();
     }
     
    //Do NOT modify this function
     public float getIsoValue(){
         return iso_value;
     }
    //Do NOT modify this function
    private void setMode(boolean slicer, boolean mip, boolean composite, boolean tf2d, boolean iso) {
        slicerMode = slicer;
        mipMode = mip;
        compositingMode = composite;
        tf2dMode = tf2d;        
        isoMode = iso;
        changed();
    }
    //Do NOT modify this function
    private boolean intersectLinePlane(double[] plane_pos, double[] plane_normal,
            double[] line_pos, double[] line_dir, double[] intersection) {

        double[] tmp = new double[3];

        for (int i = 0; i < 3; i++) {
            tmp[i] = plane_pos[i] - line_pos[i];
        }

        double denom = VectorMath.dotproduct(line_dir, plane_normal);
        if (Math.abs(denom) < 1.0e-8) {
            return false;
        }

        double t = VectorMath.dotproduct(tmp, plane_normal) / denom;

        for (int i = 0; i < 3; i++) {
            intersection[i] = line_pos[i] + t * line_dir[i];
        }

        return true;
    }
    //Do NOT modify this function
    private boolean validIntersection(double[] intersection, double xb, double xe, double yb,
            double ye, double zb, double ze) {

        return (((xb - 0.5) <= intersection[0]) && (intersection[0] <= (xe + 0.5))
                && ((yb - 0.5) <= intersection[1]) && (intersection[1] <= (ye + 0.5))
                && ((zb - 0.5) <= intersection[2]) && (intersection[2] <= (ze + 0.5)));

    }
    //Do NOT modify this function
    private void intersectFace(double[] plane_pos, double[] plane_normal,
            double[] line_pos, double[] line_dir, double[] intersection,
            double[] entryPoint, double[] exitPoint) {

        boolean intersect = intersectLinePlane(plane_pos, plane_normal, line_pos, line_dir,
                intersection);
        if (intersect) {

            double xpos0 = 0;
            double xpos1 = volume.getDimX();
            double ypos0 = 0;
            double ypos1 = volume.getDimY();
            double zpos0 = 0;
            double zpos1 = volume.getDimZ();

            if (validIntersection(intersection, xpos0, xpos1, ypos0, ypos1,
                    zpos0, zpos1)) {
                if (VectorMath.dotproduct(line_dir, plane_normal) < 0) {
                    entryPoint[0] = intersection[0];
                    entryPoint[1] = intersection[1];
                    entryPoint[2] = intersection[2];
                } else {
                    exitPoint[0] = intersection[0];
                    exitPoint[1] = intersection[1];
                    exitPoint[2] = intersection[2];
                }
            }
        }
    }
    
     
    
    //Do NOT modify this function
    void computeEntryAndExit(double[] p, double[] viewVec, double[] entryPoint, double[] exitPoint) {

        for (int i = 0; i < 3; i++) {
            entryPoint[i] = -1;
            exitPoint[i] = -1;
        }

        double[] plane_pos = new double[3];
        double[] plane_normal = new double[3];
        double[] intersection = new double[3];

        VectorMath.setVector(plane_pos, volume.getDimX(), 0, 0);
        VectorMath.setVector(plane_normal, 1, 0, 0);
        intersectFace(plane_pos, plane_normal, p, viewVec, intersection, entryPoint, exitPoint);

        VectorMath.setVector(plane_pos, 0, 0, 0);
        VectorMath.setVector(plane_normal, -1, 0, 0);
        intersectFace(plane_pos, plane_normal, p, viewVec, intersection, entryPoint, exitPoint);

        VectorMath.setVector(plane_pos, 0, volume.getDimY(), 0);
        VectorMath.setVector(plane_normal, 0, 1, 0);
        intersectFace(plane_pos, plane_normal, p, viewVec, intersection, entryPoint, exitPoint);

        VectorMath.setVector(plane_pos, 0, 0, 0);
        VectorMath.setVector(plane_normal, 0, -1, 0);
        intersectFace(plane_pos, plane_normal, p, viewVec, intersection, entryPoint, exitPoint);

        VectorMath.setVector(plane_pos, 0, 0, volume.getDimZ());
        VectorMath.setVector(plane_normal, 0, 0, 1);
        intersectFace(plane_pos, plane_normal, p, viewVec, intersection, entryPoint, exitPoint);

        VectorMath.setVector(plane_pos, 0, 0, 0);
        VectorMath.setVector(plane_normal, 0, 0, -1);
        intersectFace(plane_pos, plane_normal, p, viewVec, intersection, entryPoint, exitPoint);

    }

    //Do NOT modify this function
    private void drawBoundingBox(GL2 gl) {
        gl.glPushAttrib(GL2.GL_CURRENT_BIT);
        gl.glDisable(GL2.GL_LIGHTING);
        gl.glColor4d(1.0, 1.0, 1.0, 1.0);
        gl.glLineWidth(1.5f);
        gl.glEnable(GL.GL_LINE_SMOOTH);
        gl.glHint(GL.GL_LINE_SMOOTH_HINT, GL.GL_NICEST);
        gl.glEnable(GL.GL_BLEND);
        gl.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);

        gl.glBegin(GL.GL_LINE_LOOP);
        gl.glVertex3d(-volume.getDimX() / 2.0, -volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(-volume.getDimX() / 2.0, volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, -volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glEnd();

        gl.glBegin(GL.GL_LINE_LOOP);
        gl.glVertex3d(-volume.getDimX() / 2.0, -volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glVertex3d(-volume.getDimX() / 2.0, volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, -volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glEnd();

        gl.glBegin(GL.GL_LINE_LOOP);
        gl.glVertex3d(volume.getDimX() / 2.0, -volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, -volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glEnd();

        gl.glBegin(GL.GL_LINE_LOOP);
        gl.glVertex3d(-volume.getDimX() / 2.0, -volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glVertex3d(-volume.getDimX() / 2.0, -volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(-volume.getDimX() / 2.0, volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(-volume.getDimX() / 2.0, volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glEnd();

        gl.glBegin(GL.GL_LINE_LOOP);
        gl.glVertex3d(-volume.getDimX() / 2.0, volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glVertex3d(-volume.getDimX() / 2.0, volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glEnd();

        gl.glBegin(GL.GL_LINE_LOOP);
        gl.glVertex3d(-volume.getDimX() / 2.0, -volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glVertex3d(-volume.getDimX() / 2.0, -volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, -volume.getDimY() / 2.0, volume.getDimZ() / 2.0);
        gl.glVertex3d(volume.getDimX() / 2.0, -volume.getDimY() / 2.0, -volume.getDimZ() / 2.0);
        gl.glEnd();

        gl.glDisable(GL.GL_LINE_SMOOTH);
        gl.glDisable(GL.GL_BLEND);
        gl.glEnable(GL2.GL_LIGHTING);
        gl.glPopAttrib();


    }
    //Do NOT modify this function
    @Override
    public void visualize(GL2 gl) {

        double[] viewMatrix = new double[4 * 4];
        
        if (volume == null) {
            return;
        }
        	
         drawBoundingBox(gl);

        
     //    gl.glGetDoublev(GL2.GL_PROJECTION_MATRIX,viewMatrix,0);

         gl.glGetDoublev(GL2.GL_MODELVIEW_MATRIX, viewMatrix, 0);
                      
         
        long startTime = System.currentTimeMillis();
        if (slicerMode) {
            slicer(viewMatrix);    
        } else {
            raycast(viewMatrix);
        }
        
        long endTime = System.currentTimeMillis();
        double runningTime = (endTime - startTime);
        panel.setSpeedLabel(Double.toString(runningTime));

        Texture texture = AWTTextureIO.newTexture(gl.getGLProfile(), image, false);
        
        gl.glPushAttrib(GL2.GL_LIGHTING_BIT);
        gl.glDisable(GL2.GL_LIGHTING);
        //gl.glEnable(GL.GL_BLEND);
        //gl.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);

        // draw rendered image as a billboard texture
        texture.enable(gl);
        texture.bind(gl);
        
        double halfWidth = res_factor*image.getWidth() / 2.0;
        gl.glPushMatrix();
        gl.glLoadIdentity();
        gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST);
        gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST);
        gl.glBegin(GL2.GL_QUADS);
        gl.glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        gl.glTexCoord2d(texture.getImageTexCoords().left(), texture.getImageTexCoords().top());
        gl.glVertex3d(-halfWidth, -halfWidth, 0.0);
        gl.glTexCoord2d(texture.getImageTexCoords().left(), texture.getImageTexCoords().bottom());
        gl.glVertex3d(-halfWidth, halfWidth, 0.0);
        gl.glTexCoord2d(texture.getImageTexCoords().right(), texture.getImageTexCoords().bottom());
        gl.glVertex3d(halfWidth, halfWidth, 0.0);
        gl.glTexCoord2d(texture.getImageTexCoords().right(), texture.getImageTexCoords().top());
        gl.glVertex3d(halfWidth, -halfWidth, 0.0);
        gl.glEnd();
        texture.disable(gl);
        
        texture.destroy(gl);
        gl.glPopMatrix();

        gl.glPopAttrib();


        if (gl.glGetError() > 0) {
            System.out.println("some OpenGL error: " + gl.glGetError());
        }


    }
    
    //Do NOT modify this function
    public BufferedImage image;

    //Do NOT modify this function
    @Override
    public void changed() {
        for (int i=0; i < listeners.size(); i++) {
            listeners.get(i).changed();
        }
    }
    

}