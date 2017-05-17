// Copyright (c) 2008-2014, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "dubins.h"
#include <mex.h>
#include <math.h>

static int n;
static int storeData(double q[3], double x, void* user_data);

// The gateway function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Check for proper number of arguments
    if (nrhs != 4)
        mexErrMsgIdAndTxt("Dubins:path:nrhs", "Four inputs required.");

    if (nlhs != 1)
        mexErrMsgIdAndTxt("Dubins:path:nlhs", "One output required.");

    // Check that number of rows in second input argument is 1
    if (mxGetM(prhs[0]) != 1)
        mexErrMsgIdAndTxt("Dubins:q0:notRowVector", "q0 must be a row vector.");

    if (mxGetM(prhs[1]) != 1)
        mexErrMsgIdAndTxt("Dubins:q1:notRowVector", "q1 must be a row vector.");

    if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1)
        mexErrMsgIdAndTxt("Dubins:r:notScalar", "Input radius must be a scalar.");

    if (!mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) || mxGetNumberOfElements(prhs[3]) !=1)
        mexErrMsgIdAndTxt("Dubins:step:notScalar", "Step size must be a scalar.");

    // Get Input 1
    double *q0      = mxGetPr(prhs[0]);
    double *q1      = mxGetPr(prhs[1]);
    double r        = mxGetScalar(prhs[2]);
    //double stepSize = mxGetScalar(prhs[3]);
    double num_steps = mxGetScalar(prhs[3]);
    
    DubinsPath path;
    dubins_init(q0, q1, r, &path);
    double length = dubins_path_length(&path);
    double stepSize = length / num_steps;
    
//n = (int)floor(length/stepSize);
    n = num_steps;
    if (n < 1)
        n = 1;

    // Create the output matrix
    plhs[0] = mxCreateDoubleMatrix((mwSize)4,(mwSize)n,mxREAL);

    // Get a pointer to the real data in the output matrix
    double *outMatrix = mxGetPr(plhs[0]);

    // Do the computation
    dubins_path_sample_many(&path, storeData, stepSize, outMatrix);
}

static int storeData(double q[3], double x, void* user_data) {
    static int i;
    if (x == 0)
        i = 0; // Reset index counter when distance is equal to 0

    ((double*)user_data)[4 * i + 0] = q[0]; // Write to the output matrix
    ((double*)user_data)[4 * i + 1] = q[1];
    ((double*)user_data)[4 * i + 2] = q[2];
    ((double*)user_data)[4 * i + 3] = x; // Save distance along the path

    if (++i >= n) // Prevent buffer overflow
        return 1; // Stop sampling

    return 0;
}
