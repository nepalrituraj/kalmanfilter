//
//  kalman_2D.hpp
//  kalman
//
//  Created by Rituraj Nepal on 17.12.2019.
//  Copyright © 2019 welete. All rights reserved.
//

#ifndef kalman_2D_hpp
#define kalman_2D_hpp

#include <stdio.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/io.hpp>


using namespace std;
using namespace boost::numeric::ublas;


class Kalman_2D{
public:
    Kalman_2D();
    ~Kalman_2D();
    matrix<double> I();
    matrix<double> initA(double dt);
    matrix<double> initB(double dt);
    matrix<double> initProcessCov(double varPosition, double varVelocity);
    matrix<double> initCurrentState(double position, double velocity);
    matrix<double> initKalmanGain();
    matrix<double> updateProcessState(matrix<double> A, matrix<double>B, matrix<double> state , double control);
    matrix<double> calculateProcessCov(matrix<double> A ,matrix<double> cov );
    matrix<double> calculateKalmanGain(matrix<double> processCov, matrix<double> measurementCov);
    void predictState(matrix<double> measuredState,matrix<double> measuredCov);
    void printState();
    matrix<double> updateProcessCov(matrix<double> kalmanGain);
    matrix<double> getCurrentState();
private:
    matrix<double> currentState;
    matrix<double> processCov; // process Covariance Matrix that represents error in process
    matrix<double> kalmanGain; // Kalman Filter uses Kalman gain matrix to update current state
    matrix<double> A;
    matrix<double> B;
    double acc ; // This is control unit called acceleration that affects both position and velocity
    
    
};


#endif /* kalman_2D_hpp */
