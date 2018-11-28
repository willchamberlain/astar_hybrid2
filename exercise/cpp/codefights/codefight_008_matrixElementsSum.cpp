#include <iostream>
#include <vector>
#include <algorithm>



int hauntedMatrixAvailableRooms(std::vector<std::vector<int>> matrix) {    
    std::vector<int> first_row = matrix[0]; int row_size = first_row.size();
    int haunted_row[row_size]; for(int ii_=0; ii_<row_size; ii_++){haunted_row[ii_]=-1;}
    int row_sum[row_size]; for(int ii_=0; ii_<row_size; ii_++){row_sum[ii_]=0;}
    for(int ii_=0; ii_<matrix.size(); ii_++) {
        std::vector<int> row_vector = matrix[ii_];
        for(int jj_=0; jj_<row_size; jj_++){
            if( haunted_row[jj_]<0 ) {
                if(row_vector[jj_] == 0) {
                    haunted_row[jj_] = ii_;
                } else {
                    row_sum[jj_] += row_vector[jj_];
                }
            }
        }
    }
    int sum = 0;
    for(int ii_=0; ii_<row_size; ii_++) {
        sum += row_sum[ii_];
    }
    return sum;
}

int hauntedMatrixAvailableRooms2(std::vector<std::vector<int>> matrix) {
    int col_sum[matrix[0].size()]; for(int ii_=0; ii_<matrix[0].size(); ii_++) {col_sum[ii_]=0;}
    for(int col=0; col<matrix[0].size(); col++){
        for(int row=0; row<matrix.size(); row++){
            // std::cout<<matrix[row][col]<<",";
            if(0==matrix[row][col]) {
                break;
            } else {
                col_sum[col] += matrix[row][col];
            }
        }
        // std::cout<<std::endl;
    }
    int sum = 0;    
    for(int ii_=0; ii_<matrix[0].size(); ii_++) {
        sum += col_sum[ii_];
    }
    return sum;
}

int matrixElementsSum(std::vector<std::vector<int>> matrix) {
    return hauntedMatrixAvailableRooms2(matrix);
}


int main(int argc, char const *argv[])
{
    std::vector<std::vector<int>> myvect 
                    = { {10,20,30,40},
                               {50,60,70,80} };
    int m_array[3][4] = {{0, 1, 1, 2}, 
                         {0, 5, 0, 0}, 
                         {2, 0, 3, 3}};

    std::cout << "------" << std::endl;
    std::vector<std::vector<int>> matrix 
        = {{0, 1, 1, 2}, 
           {0, 5, 0, 0}, 
           {2, 0, 3, 3}};
    std::cout << matrix.size() << "," << matrix[0].size() << std::endl;         
    std::cout << matrixElementsSum(matrix) << std::endl;
    std::cout << "------" << std::endl;
    matrix =   {{1,1,1}, 
                {2,2,2}, 
                {3,3,3}};
    std::cout << matrix.size() << "," << matrix[0].size() << std::endl;         
    std::cout << matrixElementsSum(matrix) << std::endl;
    std::cout << "------" << std::endl;
    matrix =   {{0}};
    std::cout << matrixElementsSum(matrix) << std::endl;
    std::cout << "------" << std::endl;
    matrix =   {{25}};
    std::cout << matrixElementsSum(matrix) << std::endl;
    return 0;
}
