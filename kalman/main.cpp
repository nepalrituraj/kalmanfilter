//
//  main.cpp
//  kalman
//
//  Created by Rituraj Nepal on 17.12.2019.
//  Copyright © 2019 welete. All rights reserved.
//

#define SAMPLE_SIZE_KALMAN_2D 5

#include <iostream>
#include "kalman_2D.hpp"
#include <boost/numeric/ublas/matrix.hpp>
//#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/io.hpp>
//#include<boost/array.hpp>
//#include <boost/numeric/ublas/vector.hpp>

using namespace std;
using namespace boost::numeric::ublas;

int main(int argc, const char * argv[]) {
    // insert code here...
    cout << "Kalman Filter 2D !\n";
    typedef matrix<double> arrayMatrix;
    arrayMatrix MeasuredSample[SAMPLE_SIZE_KALMAN_2D];
    
    // Initial State but while object initializes , initial state set to s0
    matrix<double> s0 (2,1,0);
    s0(0,0) = 4000;
    s0(1,0) = 280;
    MeasuredSample[0] = s0 ;
    
    matrix<double> s1 (2,1,0);
    s1(0,0) = 4260;
    s1(1,0) = 282;
    MeasuredSample[1] = s1 ;
    
    matrix<double> s2 (2,1,0);
    s2(0,0) = 4550;
    s2(1,0) = 285;
    MeasuredSample[2] = s2;
    
    matrix<double> s3(2,1,0);
    s3(0,0) = 4860;
    s3(1,0) = 286;
    MeasuredSample[3] = s3 ;
    
    matrix<double> s4 (2,1,0);
    s4(0,0) = 5110;
    s4(1,0) = 290;
    MeasuredSample[4] = s4 ;
    
    
    matrix<double> measuredCovariance(2,2,0);
    measuredCovariance(0,0) = 625.f ; // 25 * 25
    measuredCovariance(1,1) = 36.f ; // 6*6
    
    
    Kalman_2D test;
    cout << "Measured State                 State Corrected by Kalman Filter  " << endl ;
    
    for ( unsigned int i = 1 ; i < SAMPLE_SIZE_KALMAN_2D ; ++i) {
        test.predictState(MeasuredSample[i], measuredCovariance);
        cout << i << " "<<  MeasuredSample[i] << "          " << test.getCurrentState() <<  endl ;
        cout << "-------------" << endl ;
    }
    
    
    
    
    
    
    return 0;
}
