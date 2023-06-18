#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Kalman filter variables
double Q = 0.125; // process noise
double R = 32; // measurement noise
double X = 0, P = 1023; // initial state
float measurements[7658][3] = { 0 };
float estimates[7658][2];
float xarray[7658];
float mesarray[7658][2];
float compare1[7658][2];
float compare2[7658][2];


void kalman_filter(double meas) {
    // prediction update
    P = P + Q;

    // measurement update
    double K = P / (P + R);
    X = X + K * (meas - X);
    P = (1 - K) * P;
}

void ranKalman (float measurement[7658][3], float estimates[7658][2], int types) {
    int j = 0;
    for (int i = 0; i < 7658; i++) {
        //printf("Before filtering: %f\n", measurements[i][types]);
        kalman_filter(measurements[i][types]);
        estimates[i][types-1] = X;
        //printf("After filtering: %f\n\n", X);
        j++;
    }  
    printf("Iteration count: %d\n", j); // Print the number of iterations
}

void read_data_from_csv(const char* filename, float data[7658][3]) {
    size_t ROWS = 7658;
    size_t COLUMNS = 3;
    //printf("rows = %d", ROWS);
    //printf("cols = %d", COLUMNS);


    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        printf("Failed to open the file for reading.\n");
        return;
    }

    // Read data from the CSV file
    for (int i = 0; i < ROWS; i++) {
        if (fscanf(file, "%f,%f,%f", &data[i][0], &data[i][1], &data[i][2]) != COLUMNS) {
            printf("Error reading data from the file.\n");
            break;
        }
    }

    fclose(file);
}

void write_data_to_file(const char *filename, float data[7658][2], int cols) {
    size_t ROWS = 7658;
    printf("Rows: %d.\n", ROWS);

    FILE *file = fopen(filename, "w+");
    if (file == NULL) {
        printf("Failed to open file\n");
        return;
    }

    for(int i = 0; i < ROWS; i++) {
        for (int j = 0; j < cols-1; j++) {
            fprintf(file, "%.2f,", data[i][j]);
         //fprintf(file, "\n");
        }
        fprintf(file, "%.2f\n", data[i][cols-1]);
    }

    fclose(file);
    printf("Data written to %s successfully.\n", filename);
} 

void gnuplot(float x[7658], float data[7658][2]) {
    int n = 7658;
    FILE *gnuplot = popen("gnuplot -persist", "w");
    fprintf(gnuplot, "plot '-' using 1:2 with lines, '-' using 1:2 with lines\n");
    for (int i = 0; i < n; i++)
        fprintf(gnuplot, "%g %g\n", x[i], data[i][0]);
    fprintf(gnuplot, "e\n");
    for (int i = 0; i < n; i++)
        fprintf(gnuplot, "%g %g\n", x[i], data[i][1]);
    fprintf(gnuplot, "e\n");
    fflush(gnuplot);
    printf("rows %d\n", n);
    printf("Plot generated.\n");
    //sleep(5); 
    //pclose(gnuplot);
}

int main() {
    // example measurements
    //double measurements[] = {100, 102, 98, 102, 101, 99, 100, 101, 102, 100};
    read_data_from_csv("cleaneddata.csv", measurements);
    printf("Data read successfully.\n");
    ranKalman (measurements, estimates, 1); // 1 means alitude
    printf("Final value of X: %f\n", X);
    ranKalman (measurements, estimates, 2); // 2 means acceleration

    for (int i = 0; i < 7658; i++) {
        xarray[i] = measurements[i][0];
        mesarray[i][0] = measurements[i][1];
        mesarray[i][1] = measurements[i][2];
        compare1[i][0] = measurements[i][1];
        compare1[i][1] = estimates[i][0];
        compare2[i][0] = measurements[i][2];
        compare2[i][1] = estimates[i][1];
    }

    write_data_to_file("filteredData.csv", estimates, 2);
    //gnuplot(xarray, compare1);
    //gnuplot(xarray, compare2);
    return 0;
}
