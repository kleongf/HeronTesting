package org.firstinspires.ftc.teamcode.localizer;

import java.util.Arrays;
public class ukf {
    private double[] x; // State estimate
    private double[][] P; // Covariance
    private double[][] Q; // Process noise covariance
    private double[][] R; // Measurement noise covariance
    private int stateDim; // State dimension
    private int measurementDim; // Measurement dimension
    private double alpha;
    private double beta;
    private double kappa; // Parameters for sigma point generation
    private double[] Wm; // Weights for means
    private double[] Wc; // Weights for covariances
    // Constructor
    public ukf(int stateDim, int measurementDim, double alpha, double beta, double kappa,
               double[][] Q, double[][] R) {
        this.stateDim = stateDim;
        this.measurementDim = measurementDim;
        this.alpha = alpha;
        this.beta = beta;
        this.kappa = kappa;
        this.Q = Q;
        this.R = R;
// Initialize state and covariance
        this.x = new double[stateDim];
        this.P = new double[stateDim][stateDim];
        for (int i = 0; i < stateDim; i++) {
            this.P[i][i] = 1.0;
        }
// Compute weights
        int numSigmaPoints = 2 * stateDim + 1;
        this.Wm = new double[numSigmaPoints];
        this.Wc = new double[numSigmaPoints];
        double lambda = Math.pow(alpha, 2) * (stateDim + kappa) - stateDim;
        Wm[0] = lambda / (stateDim + lambda);
        Wc[0] = Wm[0] + (1 - Math.pow(alpha, 2) + beta);
        for (int i = 1; i < numSigmaPoints; i++) {
            Wm[i] = 1.0 / (2 * (stateDim + lambda));

            Wc[i] = Wm[i];
        }
    }
    // Getter for the current state estimate
    public double[] getState() {
        return x;
    }
    // Setter for the initial state estimate
    public void setInitialState(double[] initialState) {
        this.x = initialState;
    }
    // Setter for the initial covariance estimate
    public void setInitialCovariance(double[][] initialCovariance) {
        this.P = initialCovariance;
    }
    // Nonlinear process model
    private double[] processModel(double[] sigmaPoint, double[] controlInput, double dt) {
        double[] newSigmaPoint = new double[sigmaPoint.length];
// Simple motion model: position and velocity
        newSigmaPoint[0] = sigmaPoint[0] + sigmaPoint[1] * dt; // Update position
        newSigmaPoint[1] = sigmaPoint[1]; // Velocity remains constant
        newSigmaPoint[1] += controlInput[0] * dt; // Apply control input (acceleration)
        return newSigmaPoint;
    }
    // Measurement model
    private double[] measurementModel(double[] sigmaPoint) {
        double[] measurement = new double[measurementDim];
// Each measurement is the position (x, y) with independent noise
// Set to 4 inputs so far, hand tune here
        for (int i = 0; i < 4; i++) {
            measurement[2 * i] = sigmaPoint[0]; // x position
            measurement[2 * i + 1] = sigmaPoint[1]; // y position
        }
        return measurement;
    }
    // Predict function
    public void predict(double[] controlInput, double dt) {

        // Generate sigma points
        double[][] sigmaPoints = generateSigmaPoints(x, P);
// Propagate sigma points through the process model
        double[][] predictedSigmaPoints = new double[sigmaPoints.length][stateDim];
        for (int i = 0; i < sigmaPoints.length; i++) {
            predictedSigmaPoints[i] = processModel(sigmaPoints[i], controlInput, dt);
        }
// Compute predicted state mean
        double[] predictedStateMean = calculateMean(predictedSigmaPoints, Wm);
// Compute predicted state covariance
        double[][] predictedStateCovariance = calculateCovariance(predictedSigmaPoints,
                predictedStateMean, Wc);
        predictedStateCovariance = addMatrices(predictedStateCovariance, Q); // Add process

// Update state and covariance estimates
        x = predictedStateMean;
        P = predictedStateCovariance;
    }
    // Update function
    public void update(double[] measurement) {
// Generate sigma points
        double[][] sigmaPoints = generateSigmaPoints(x, P);
// Transform sigma points through the measurement model
        double[][] measurementSigmaPoints = new double[sigmaPoints.length][measurementDim];
        for (int i = 0; i < sigmaPoints.length; i++) {
            measurementSigmaPoints[i] = measurementModel(sigmaPoints[i]);
        }
// Compute predicted measurement mean
        double[] predictedMeasurementMean = calculateMean(measurementSigmaPoints, Wm);
// Compute innovation covariance
        double[][] innovationCovariance = calculateCovariance(measurementSigmaPoints,
                predictedMeasurementMean, Wc);
        innovationCovariance = addMatrices(innovationCovariance, R); // Add measurement noise
// Compute cross-covariance

        double[][] crossCovariance = calculateCrossCovariance(sigmaPoints, x,
                measurementSigmaPoints, predictedMeasurementMean, Wc);
// Compute Kalman gain
        double[][] K = multiplyMatrices(crossCovariance, invertMatrix(innovationCovariance));
// Update state estimate
        double[] innovation = subtractVectors(measurement, predictedMeasurementMean);
        double[] stateUpdate = multiplyMatrixVector(K, innovation);
        x = addVectors(x, stateUpdate);
// Update covariance estimate
        double[][] temp = multiplyMatrices(K, innovationCovariance);
        double[][] K_S_Kt = multiplyMatrices(temp, transposeMatrix(K));
        P = subtractMatrices(P, K_S_Kt);
    }
    // Generate sigma points
    private double[][] generateSigmaPoints(double[] x, double[][] P) {
        int n = x.length;
        int numSigmaPoints = 2 * n + 1;
        double[][] sigmaPoints = new double[numSigmaPoints][n];
        double lambda = Math.pow(alpha, 2) * (n + kappa) - n;
        double scalingFactor = Math.sqrt(n + lambda);
// Compute square root of P using Cholesky decomposition
        double[][] sqrtP = choleskyDecomposition(P);
// First sigma point is the mean
        System.arraycopy(x, 0, sigmaPoints[0], 0, n);
// Generate remaining sigma points
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < n; k++) {
                sigmaPoints[i + 1][k] = x[k] + scalingFactor * sqrtP[k][i];
                sigmaPoints[i + 1 + n][k] = x[k] - scalingFactor * sqrtP[k][i];
            }
        }
        return sigmaPoints;
    }
// Cholesky decomposition

    private double[][] choleskyDecomposition(double[][] matrix) {
        int n = matrix.length;
        double[][] L = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double sum = matrix[i][j];
                for (int k = 0; k < j; k++) {
                    sum -= L[i][k] * L[j][k];
                }
                if (i == j) {
                    if (sum <= 0.0) {
                        throw new RuntimeException("Matrix is not positive definite");
                    }
                    L[i][j] = Math.sqrt(sum);
                } else {
                    L[i][j] = sum / L[j][j];
                }
            }
        }
        return L;
    }
    // Calculate mean from sigma points
    private double[] calculateMean(double[][] sigmaPoints, double[] Wm) {
        int n = sigmaPoints[0].length;
        double[] mean = new double[n];
        for (int i = 0; i < sigmaPoints.length; i++) {
            for (int j = 0; j < n; j++) {
                mean[j] += Wm[i] * sigmaPoints[i][j];
            }
        }
        return mean;
    }
    // Calculate covariance from sigma points and mean
    private double[][] calculateCovariance(double[][] sigmaPoints, double[] mean, double[] Wc) {
        int n = mean.length;
        double[][] covariance = new double[n][n];
        for (int i = 0; i < sigmaPoints.length; i++) {
            double[] diff = subtractVectors(sigmaPoints[i], mean);
            for (int j = 0; j < n; j++) {

                for (int k = 0; k < n; k++) {
                    covariance[j][k] += Wc[i] * diff[j] * diff[k];
                }
            }
        }
        return covariance;
    }
    // Calculate cross-covariance
    private double[][] calculateCrossCovariance(double[][] sigmaPoints, double[] xMean,
                                                double[][] measurementSigmaPoints, double[] zMean, double[] Wc) {
        int n = xMean.length;
        int m = zMean.length;
        double[][] crossCovariance = new double[n][m];
        for (int i = 0; i < sigmaPoints.length; i++) {
            double[] xDiff = subtractVectors(sigmaPoints[i], xMean);
            double[] zDiff = subtractVectors(measurementSigmaPoints[i], zMean);
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < m; k++) {
                    crossCovariance[j][k] += Wc[i] * xDiff[j] * zDiff[k];
                }
            }
        }
        return crossCovariance;
    }
    // Vector operations
    private double[] subtractVectors(double[] a, double[] b) {
        double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = a[i] - b[i];
        }
        return result;
    }
    private double[] addVectors(double[] a, double[] b) {
        double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = a[i] + b[i];
        }
        return result;
    }
    private double[] multiplyMatrixVector(double[][] matrix, double[] vector) {

        int rows = matrix.length;
        int cols = vector.length;
        double[] result = new double[rows];
        for (int i = 0; i < rows; i++) {
            result[i] = 0;
            for (int j = 0; j < cols; j++) {
                result[i] += matrix[i][j] * vector[j];
            }
        }
        return result;
    }
    // Matrix operations
    private double[][] addMatrices(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = A[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = A[i][j] + B[i][j];
            }
        }
        return result;
    }
    private double[][] subtractMatrices(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = A[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = A[i][j] - B[i][j];
            }
        }
        return result;
    }
    private double[][] multiplyMatrices(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = B[0].length;
        int commonDim = A[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {

                for (int k = 0; k < commonDim; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }
    private double[][] transposeMatrix(double[][] matrix) {
        int rows = matrix.length;
        int cols = matrix[0].length;
        double[][] transposedMatrix = new double[cols][rows];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                transposedMatrix[j][i] = matrix[i][j];
            }
        }
        return transposedMatrix;
    }
    // Invert a 1x1 or 2x2 matrix
    private double[][] invertMatrix(double[][] matrix) {
        if (matrix == null || matrix.length == 0 || matrix[0].length == 0) {
            throw new IllegalArgumentException("Matrix is empty or null");
        }
        double[][] inverseMatrix = new double[matrix.length][matrix.length];
        if (matrix[0].length == 1){
            inverseMatrix[0][0] = 1 / matrix[0][0];
            return(inverseMatrix);
        }
        if (matrix[0].length == 2){
            inverseMatrix[0][0] = matrix[1][1]/determinant(matrix);
            inverseMatrix[0][1] = -matrix[1][0]/determinant(matrix);
            inverseMatrix[1][0] = -matrix[0][1]/determinant(matrix);
            inverseMatrix[1][1] = matrix[0][0]/determinant(matrix);
            return inverseMatrix;
        }

// Minors and cofactors
        for (int i = 0; i < inverseMatrix.length; i++) {
            for (int j = 0; j < inverseMatrix[i].length; j++) {
                inverseMatrix[i][j] = Math.pow(-1, i + j) * determinant(submatrix(matrix, i, j));

            }
        }
// Adjugate and determinant
        double det = determinant(matrix);
        if (det == 0) {
            throw new ArithmeticException("Matrix is singular and cannot be inverted");
        }
        double invDet = 1.0 / det;
        for (int i = 0; i < inverseMatrix.length; i++) {
            for (int j = 0; j < inverseMatrix[i].length; j++) {
                inverseMatrix[i][j] *= invDet;
            }
        }
        return inverseMatrix;
    }
    // Determinant of a matrix
    private double determinant(double[][] matrix) {
        if (matrix.length == 2)
            return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        double det = 0;
        for (int i = 0; i < matrix[0].length; i++)
            det += Math.pow(-1, i) * matrix[0][i]
                    * determinant(submatrix(matrix, 0, i));
        return det;
    }
    // IDK what this does it was on StackExchange OK?
    private double[][] submatrix(double[][] matrix, int row, int column) {
        double[][] submatrix = new double[matrix.length - 1][matrix.length - 1];
        for (int i = 0; i < matrix.length; i++)
            for (int j = 0; i != row && j < matrix[i].length; j++)
                if (j != column)
                    submatrix[i < row ? i : i - 1][j < column ? j : j - 1] = matrix[i][j];
        return submatrix;
    }
    // Add this method to your ukf class or any utility class
    public void printMatrix(double[][] matrix) {
        if (matrix == null) {
            System.out.println("Matrix is null.");

            return;
        }
        for (double[] row : matrix) {
            System.out.println(Arrays.toString(row));
        }
    }
}
