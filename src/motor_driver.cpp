// #include "motor_driver.h"

// int seven_motor_move(int CanDeviceId, vector<int> can_list, vector<vector<vector<int>>> position_velocity)
// {
//     cout << showpos;
//     int n = can_list.size();

//     vector<std::array<int, 3>> CSP_list(n);

//     while (1)
//     {
//         getCSP(CanDeviceId, can_list, CSP_list);
//         int count = 0;
//         for(int i = 0; i < n; i++){
//             if(CSP_list[i][0] == CSP_list[i][1] && CSP_list[i][1] == CSP_list[i][2]){
//                 i = 0;
//             }else{
//                 count++;
//             }
//         }
//         if (count == n)
//         {
//             break;
//         }
//     }
//     cout << "here" << endl;


//     int temp[n] = dataList1[2], temp2 = dataList2[2], temp3 = dataList3[2], temp4 = dataList4[2], temp5 = dataList5[2], temp6 = dataList6[2], temp7 = dataList7[2];

//     int stop;

//     int n = position_velocity1[1].size(), m = position_velocity2[1].size(), l = position_velocity3[1].size();
//     int q = position_velocity1[1].size(), w = position_velocity2[1].size(), e = position_velocity3[1].size(), r = position_velocity3[1].size();

//     int i = 0, j = 0, k = 0, z = 0, x = 0, c = 0, v = 0;
//     int count = 0;
//     auto start = std::chrono::high_resolution_clock::now();
//     while (1)
//     {

//         count++;
//         setSpeed(CanDeviceId, 0, numaccuator, can_list[0], dataList1, position_velocity1[1].data() + i);
//         setSpeed(CanDeviceId, 0, numaccuator, can_list[1], dataList2, position_velocity2[1].data() + j);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList3, dataList3, position_velocity3[1].data() + k);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList4, dataList4, position_velocity4[1].data() + z);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList5, dataList5, position_velocity5[1].data() + x);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList6, dataList6, position_velocity6[1].data() + c);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList7, dataList7, position_velocity7[1].data() + v);

//         if (i >= n - 1 && j >= m - 1 && k >= l - 1 && z >= q - 1 && x >= w - 1 && c >= e - 1 && v >= r - 1){

//             break;
//         }
            

//         if ((abs(temp1 - dataList1[2]) >= abs(position_velocity1[0][i]) || abs(position_velocity1[1][i]) == 0) && i < n - 1)
//             i++;
//         if ((abs(temp2 - dataList2[2]) >= abs(position_velocity2[0][j]) || abs(position_velocity2[1][j]) == 0) && j < m - 1)
//             j++;
//         if ((abs(temp3 - dataList3[2]) >= abs(position_velocity3[0][k]) || abs(position_velocity3[1][k]) == 0) && k < l - 1)
//             k++;
//         if ((abs(temp4 - dataList4[2]) >= abs(position_velocity4[0][z]) || abs(position_velocity4[1][z]) == 0) && z < q - 1)
//             z++;
//         if ((abs(temp5 - dataList5[2]) >= abs(position_velocity5[0][x]) || abs(position_velocity5[1][x]) == 0) && x < w - 1)
//             x++;
//         if ((abs(temp6 - dataList6[2]) >= abs(position_velocity6[0][c]) || abs(position_velocity6[1][c]) == 0) && c < e - 1)
//             c++;
//         if ((abs(temp7 - dataList7[2]) >= abs(position_velocity7[0][v]) || abs(position_velocity7[1][v]) == 0) && v < r - 1)
//             v++;

//     }
//     auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> gap = end - start;
//     cout << "time: " << gap.count() << endl;
//     cout << "count: " << count << endl;
//     cout << "frequence: " << count / gap.count() << endl;

//     return 0;
// }


// vector<vector<double>> sequence_generate(MatrixXd &Q, MatrixXd &DQ){
    
// }