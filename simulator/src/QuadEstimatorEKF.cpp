#include "Common.h"
#include "QuadEstimatorEKF.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Math/Quaternion.h"
#include <iostream>
using namespace SLR;

const int QuadEstimatorEKF::QUAD_EKF_NUM_STATES;

QuadEstimatorEKF::QuadEstimatorEKF(string config, string name)
  : BaseQuadEstimator(config),
  Q(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
  R_GPS(6, 6),
  R_Mag(1, 1),
  ekfState(QUAD_EKF_NUM_STATES),
  ekfCov(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
  trueError(QUAD_EKF_NUM_STATES)
{
  _name = name;
  Init();
}

QuadEstimatorEKF::~QuadEstimatorEKF()
{

}

void QuadEstimatorEKF::Init()
{
  ParamsHandle paramSys = SimpleConfig::GetInstance();

  paramSys->GetFloatVector(_config + ".InitState", ekfState);

  VectorXf initStdDevs(QUAD_EKF_NUM_STATES);
  paramSys->GetFloatVector(_config + ".InitStdDevs", initStdDevs);
  ekfCov.setIdentity();
  for (int i = 0; i < QUAD_EKF_NUM_STATES; i++)
  {
    ekfCov(i, i) = initStdDevs(i) * initStdDevs(i);
  }

  // complementary filter params
  attitudeTau = paramSys->Get(_config + ".AttitudeTau", .1f);
  dtIMU = paramSys->Get(_config + ".dtIMU", .002f);

  pitchEst = 0;
  rollEst = 0;
  
  // GPS measurement model covariance
  R_GPS.setZero();
  R_GPS(0, 0) = R_GPS(1, 1) = powf(paramSys->Get(_config + ".GPSPosXYStd", 0), 2);
  R_GPS(2, 2) = powf(paramSys->Get(_config + ".GPSPosZStd", 0), 2);
  R_GPS(3, 3) = R_GPS(4, 4) = powf(paramSys->Get(_config + ".GPSVelXYStd", 0), 2);
  R_GPS(5, 5) = powf(paramSys->Get(_config + ".GPSVelZStd", 0), 2);

  // magnetometer measurement model covariance
  R_Mag.setZero();
  R_Mag(0, 0) = powf(paramSys->Get(_config + ".MagYawStd", 0), 2);

  // load the transition model covariance
  Q.setZero();
  Q(0, 0) = Q(1, 1) = powf(paramSys->Get(_config + ".QPosXYStd", 0), 2);
  Q(2, 2) = powf(paramSys->Get(_config + ".QPosZStd", 0), 2);
  Q(3, 3) = Q(4, 4) = powf(paramSys->Get(_config + ".QVelXYStd", 0), 2);
  Q(5, 5) = powf(paramSys->Get(_config + ".QVelZStd", 0), 2);
  Q(6, 6) = powf(paramSys->Get(_config + ".QYawStd", 0), 2);
  Q *= dtIMU;

  rollErr = pitchErr = maxEuler = 0;
  posErrorMag = velErrorMag = 0;
}

void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{
  // Improve a complementary filter-type attitude filter
  // 
  // Currently a small-angle approximation integration method is implemented
  // The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
  // 
  // Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))
  // to integrate the body rates into new Euler angles.
  //
  // HINTS:
  //  - there are several ways to go about this, including:
  //    1) create a rotation matrix based on your current Euler angles, integrate that, convert back to Euler angles
  //    OR 
  //    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
  //       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float accelRoll;    // Roll angle estimated from IMU accelerometer
  float accelPitch;   // Pitch " "

  float gyroRoll;      // Roll angle estimated from IMU gyroscope
  float gyroPitch;     // Pitch " "
  float gyroYaw;       // Yaw " "

  float gyroDotRoll;   // Roll angle derivative estimated from IMU gyroscope
  float gyroDotPitch;  // Pitch " "
  float gyroDotYaw;    // Yaw " "

  // ESTIMATED EULER DERIVATIVES - BODY RATE APPROXIMATION (DEFAULT METHOD PROVIDED BY UDACITY)

  // gyroDotRoll = gyro.x;
  // gyroDotPitch = gyro.y;
  // gyroDotYaw = gyro.z;

  // ESTIMATED EULER DERIVATIVES - EULER FORWARD METHOD (METHOD SELECTED FOR THIS PROJECT)

  gyroDotRoll = gyro.x + ( sin(rollEst) * tan(pitchEst) ) * gyro.y + ( cos(rollEst) * tan(pitchEst) ) * gyro.z;
  gyroDotPitch = (cos(rollEst)) * gyro.y - (sin(rollEst)) * gyro.z;
  gyroDotYaw = ( sin(rollEst) / cos(pitchEst) ) * gyro.y + ( cos(rollEst) / cos(pitchEst) ) * gyro.z;

  // PREDICTED ROLL / PITCH / YAW

  gyroRoll = rollEst + gyroDotRoll * dtIMU;
  gyroPitch = pitchEst + gyroDotPitch * dtIMU;
  gyroYaw = ekfState(6) + gyroDotYaw * dtIMU;

  // normalize gyroYaw to -pi .. pi
  if (gyroYaw > F_PI) gyroYaw -= 2.f*F_PI;
  if (gyroYaw < -F_PI) gyroYaw += 2.f*F_PI;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  // CALCULATE UPDATE
  accelRoll = atan2f(accel.y, accel.z);
  accelPitch = atan2f(-accel.x, 9.81f);

  // FUSE INTEGRATION AND UPDATE
  rollEst = attitudeTau / (attitudeTau + dtIMU) * (gyroRoll)+dtIMU / (attitudeTau + dtIMU) * accelRoll;
  pitchEst = attitudeTau / (attitudeTau + dtIMU) * (gyroPitch)+dtIMU / (attitudeTau + dtIMU) * accelPitch;
  ekfState(6) = gyroYaw;

  lastGyro = gyro;
}

void QuadEstimatorEKF::UpdateTrueError(V3F truePos, V3F trueVel, Quaternion<float> trueAtt)
{
  VectorXf trueState(QUAD_EKF_NUM_STATES);
  trueState(0) = truePos.x;
  trueState(1) = truePos.y;
  trueState(2) = truePos.z;
  trueState(3) = trueVel.x;
  trueState(4) = trueVel.y;
  trueState(5) = trueVel.z;
  trueState(6) = trueAtt.Yaw();

  trueError = ekfState - trueState;
  if (trueError(6) > F_PI) trueError(6) -= 2.f*F_PI;
  if (trueError(6) < -F_PI) trueError(6) += 2.f*F_PI;

  pitchErr = pitchEst - trueAtt.Pitch();
  rollErr = rollEst - trueAtt.Roll();
  maxEuler = MAX(fabs(pitchErr), MAX(fabs(rollErr), fabs(trueError(6))));

  posErrorMag = truePos.dist(V3F(ekfState(0), ekfState(1), ekfState(2)));
  velErrorMag = trueVel.dist(V3F(ekfState(3), ekfState(4), ekfState(5)));
}

VectorXf QuadEstimatorEKF::PredictState(VectorXf curState, float dt, V3F accel, V3F gyro)
{
  assert(curState.size() == QUAD_EKF_NUM_STATES);
  VectorXf predictedState = curState;
  // Predict the current state forward by time dt using current accelerations and body rates as input
  // INPUTS: 
  //   curState: starting state
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   
  // OUTPUT:
  //   return the predicted state as a vector

  // HINTS 
  // - dt is the time duration for which you should predict. It will be very short (on the order of 1ms)
  //   so simplistic integration methods are fine here
  // - we've created an Attitude Quaternion for you from the current state. Use 
  //   attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
  // - the yaw integral is already done in the IMU update. Be sure not to integrate it again here

  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  predictedState[0] = curState[0] + curState[3]*dt;
  predictedState[1] = curState[1] + curState[4]*dt;
  predictedState[2] = curState[2] + curState[5]*dt;

  // Transform accel from body frame to global frame.
  V3F accel_global = attitude.Rotate_BtoI(accel);

  predictedState[3] = curState[3] + accel_global.x*dt;
  predictedState[4] = curState[4] + accel_global.y*dt;
  predictedState[5] = curState[5] + accel_global.z*dt - CONST_GRAVITY*dt;

  // Uncomment this line to disable PredictState()
  // predictedState = curState;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return predictedState;
}

MatrixXf QuadEstimatorEKF::GetRbgPrime(float roll, float pitch, float yaw)
{
  // first, figure out the Rbg_prime
  MatrixXf RbgPrime(3, 3);
  RbgPrime.setZero();

  // Return the partial derivative of the Rbg rotation matrix with respect to yaw. We call this RbgPrime.
  // INPUTS: 
  //   roll, pitch, yaw: Euler angles at which to calculate RbgPrime
  //   
  // OUTPUT:
  //   return the 3x3 matrix representing the partial derivative at the given point

  // HINTS
  // - this is just a matter of putting the right sin() and cos() functions in the right place.
  //   make sure you write clear code and triple-check your math
  // - You can also do some numerical partial derivatives in a unit test scheme to check 
  //   that your calculations are reasonable

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float s_phi   = sin(roll);
  float c_phi   = cos(roll);
  float s_theta = sin(pitch);
  float c_theta = cos(pitch);
  float s_psi   = sin(yaw);
  float c_psi   = cos(yaw);

  RbgPrime(0,0) = -c_phi * s_psi;
  RbgPrime(0,1) = -s_phi * s_theta * s_psi - c_phi * c_psi;
  RbgPrime(0,2) = -c_phi * s_theta * s_psi + s_phi * c_psi;
  RbgPrime(1,0) =  c_phi * c_psi;
  RbgPrime(1,1) =  s_phi * s_theta * c_psi - c_phi * s_psi;
  RbgPrime(1,2) =  c_phi * s_theta * c_psi + s_phi * s_psi;

  // Uncomment this line to disable GetRbgPrime()
  // RbgPrime.setZero();

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return RbgPrime;
}

void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
  // predict the state forward
  VectorXf newState = PredictState(ekfState, dt, accel, gyro);

  // Predict the current covariance forward by dt using the current accelerations and body rates as input.
  // INPUTS: 
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   state (member variable): current state (state at the beginning of this prediction)
  //   
  // OUTPUT:
  //   update the member variable cov to the predicted covariance

  // HINTS
  // - update the covariance matrix cov according to the EKF equation.
  // 
  // - you may find the current estimated attitude in variables rollEst, pitchEst, state(6).
  //
  // - use the class MatrixXf for matrices. To create a 3x5 matrix A, use MatrixXf A(3,5).
  //
  // - the transition model covariance, Q, is loaded up from a parameter file in member variable Q
  // 
  // - This is unfortunately a messy step. Try to split this up into clear, manageable steps:
  //   1) Calculate the necessary helper matrices, building up the transition jacobian
  //   2) Once all the matrices are there, write the equation to update cov.
  //
  // - if you want to transpose a matrix in-place, use A.transposeInPlace(), not A = A.transpose()
  // 

  // we'll want the partial derivative of the Rbg matrix
  MatrixXf RbgPrime = GetRbgPrime(rollEst, pitchEst, ekfState(6));

  // we've created an empty Jacobian for you, currently simply set to identity
  MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
  gPrime.setIdentity();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  gPrime(0,3) = dt;
  gPrime(1,4) = dt;
  gPrime(2,5) = dt;

  gPrime(3,6) = (RbgPrime(0,0)*accel.x + RbgPrime(0,1)*accel.y + RbgPrime(0,2)*accel.z) * dt;
  gPrime(4,6) = (RbgPrime(1,0)*accel.x + RbgPrime(1,1)*accel.y + RbgPrime(1,2)*accel.z) * dt;
  gPrime(5,6) = (RbgPrime(2,0)*accel.x + RbgPrime(2,1)*accel.y + RbgPrime(2,2)*accel.z) * dt;

  // Comment line below to disable covariance update
  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q; // Q = process noise covariance matrix

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  ekfState = newState;
}

void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  // GPS UPDATE
  // Hints: 
  //  - The GPS measurement covariance is available in member variable R_GPS
  //  - this is a very simple update
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Initiliaze variables for measurement update

  MatrixXf h(6,6);                          // Measurement model for GPS
  MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);  // Jacobian for GPS

  VectorXf x_predicted(6);
  VectorXf z_measured(6);
  VectorXf z_predicted(6);

  // Set variables for measurement update

  h.setIdentity();

  hPrime.setZero();
  for ( int i = 0; i < 6; i++) {  // hPrime is 6x7, last column must remain 0's
      hPrime(i,i) = 1;
  }

  for ( int i = 0; i < 6; i++) {
      x_predicted(i) = ekfState(i);
  }

  z_measured(0) = pos.x;
  z_measured(1) = pos.y;
  z_measured(2) = pos.z;
  z_measured(3) = vel.x;
  z_measured(4) = vel.y;
  z_measured(5) = vel.z;

  for ( int i = 0; i < 6; i++) {
      z_predicted(i) = h(i,i) * x_predicted(i);
  }

  // Uncomment these 2 lines to disable UpdateFromGPS()
  // hPrime.setZero();
  // z_predicted.setZero();

  Update(z_measured, hPrime, R_GPS, z_predicted);

  /////////////////////////////// END STUDENT CODE ////////////////////////////


}

void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{

  // MAGNETOMETER UPDATE
  // Hints: 
  //  - Your current estimated yaw can be found in the state vector: ekfState(6)
  //  - Make sure to normalize the difference between your measured and estimated yaw
  //    (you don't want to update your yaw the long way around the circle)
  //  - The magnetomer measurement covariance is available in member variable R_Mag

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Initiliaze variables for measurement update

  VectorXf h(1);                            // Measurement model for magnetometer
  MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);  // Jacobian for magnetometer

  VectorXf x_predicted(1);
  VectorXf z_measured(1);
  VectorXf z_predicted(1);

  // Set variables for measurement update

  h(0) = 1;

  hPrime.setZero();
  hPrime(0,6) = 1;

  x_predicted(0) = ekfState(6);
  z_measured(0) = magYaw;
  z_predicted = h * x_predicted;

  // Normalize z_predicted
  //
  // z_predicted is an angle which can have a value from -180 deg (-PI) to 180 deg (PI)
  // z_predicted can therefore have 2 values, one positive, one negative, which both represent
  // the same angle.
  // We want to make sure the value sent to the measurement update function
  // is the one with the smallest difference from the measured value
  // so that term K*(z_measured - z_predicted) in the covariance update
  // is has small as possible.
  //
  // References:  https://knowledge.udacity.com/questions/556615
  //              https://knowledge.udacity.com/questions/513624
  //

  float z_delta = z_measured(0) - z_predicted(0);

  if (z_delta > F_PI) z_measured(0) -= 2.f*F_PI;   // Reusing code from function UpdateFromIMU().
  if (z_delta < -F_PI) z_measured(0) += 2.f*F_PI;

  // Run measurement update step

  // Uncomment these 2 lines to disable UpdateFromMag()
  // hPrime.setZero();
  // z_predicted.setZero();

  Update(z_measured, hPrime, R_Mag, z_predicted);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

}

// Execute an EKF update step
// z: measurement
// H: Jacobian of observation function evaluated at the current estimated state
// R: observation error model covariance 
// zFromX: measurement prediction based on current state
void QuadEstimatorEKF::Update(VectorXf& z, MatrixXf& H, MatrixXf& R, VectorXf& zFromX)
{
  assert(z.size() == H.rows());
  assert(QUAD_EKF_NUM_STATES == H.cols());
  assert(z.size() == R.rows());
  assert(z.size() == R.cols());
  assert(z.size() == zFromX.size());

  MatrixXf toInvert(z.size(), z.size());
  toInvert = H*ekfCov*H.transpose() + R;
  MatrixXf K = ekfCov * H.transpose() * toInvert.inverse();

  ekfState = ekfState + K*(z - zFromX);

  MatrixXf eye(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
  eye.setIdentity();

  ekfCov = (eye - K*H)*ekfCov;
}

// Calculate the condition number of the EKF ovariance matrix (useful for numerical diagnostics)
// The condition number provides a measure of how similar the magnitudes of the error metric beliefs 
// about the different states are. If the magnitudes are very far apart, numerical issues will start to come up.
float QuadEstimatorEKF::CovConditionNumber() const
{
  MatrixXf m(7, 7);
  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      m(i, j) = ekfCov(i, j);
    }
  }

  Eigen::JacobiSVD<MatrixXf> svd(m);
  float cond = svd.singularValues()(0)
    / svd.singularValues()(svd.singularValues().size() - 1);
  return cond;
}

// Access functions for graphing variables
bool QuadEstimatorEKF::GetData(const string& name, float& ret) const
{
  if (name.find_first_of(".") == string::npos) return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');

  if (ToUpper(leftPart) == ToUpper(_name))
  {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
    GETTER_HELPER("Est.roll", rollEst);
    GETTER_HELPER("Est.pitch", pitchEst);

    GETTER_HELPER("Est.x", ekfState(0));
    GETTER_HELPER("Est.y", ekfState(1));
    GETTER_HELPER("Est.z", ekfState(2));
    GETTER_HELPER("Est.vx", ekfState(3));
    GETTER_HELPER("Est.vy", ekfState(4));
    GETTER_HELPER("Est.vz", ekfState(5));
    GETTER_HELPER("Est.yaw", ekfState(6));

    GETTER_HELPER("Est.S.x", sqrtf(ekfCov(0, 0)));
    GETTER_HELPER("Est.S.y", sqrtf(ekfCov(1, 1)));
    GETTER_HELPER("Est.S.z", sqrtf(ekfCov(2, 2)));
    GETTER_HELPER("Est.S.vx", sqrtf(ekfCov(3, 3)));
    GETTER_HELPER("Est.S.vy", sqrtf(ekfCov(4, 4)));
    GETTER_HELPER("Est.S.vz", sqrtf(ekfCov(5, 5)));
    GETTER_HELPER("Est.S.yaw", sqrtf(ekfCov(6, 6)));

    // diagnostic variables
    GETTER_HELPER("Est.D.AccelPitch", accelPitch);
    GETTER_HELPER("Est.D.AccelRoll", accelRoll);

    GETTER_HELPER("Est.D.ax_g", accelG[0]);
    GETTER_HELPER("Est.D.ay_g", accelG[1]);
    GETTER_HELPER("Est.D.az_g", accelG[2]);

    GETTER_HELPER("Est.E.x", trueError(0));
    GETTER_HELPER("Est.E.y", trueError(1));
    GETTER_HELPER("Est.E.z", trueError(2));
    GETTER_HELPER("Est.E.vx", trueError(3));
    GETTER_HELPER("Est.E.vy", trueError(4));
    GETTER_HELPER("Est.E.vz", trueError(5));
    GETTER_HELPER("Est.E.yaw", trueError(6));
    GETTER_HELPER("Est.E.pitch", pitchErr);
    GETTER_HELPER("Est.E.roll", rollErr);
    GETTER_HELPER("Est.E.MaxEuler", maxEuler);

    GETTER_HELPER("Est.E.pos", posErrorMag);
    GETTER_HELPER("Est.E.vel", velErrorMag);

    GETTER_HELPER("Est.D.covCond", CovConditionNumber());
#undef GETTER_HELPER
  }
  return false;
};

vector<string> QuadEstimatorEKF::GetFields() const
{
  vector<string> ret = BaseQuadEstimator::GetFields();
  ret.push_back(_name + ".Est.roll");
  ret.push_back(_name + ".Est.pitch");

  ret.push_back(_name + ".Est.x");
  ret.push_back(_name + ".Est.y");
  ret.push_back(_name + ".Est.z");
  ret.push_back(_name + ".Est.vx");
  ret.push_back(_name + ".Est.vy");
  ret.push_back(_name + ".Est.vz");
  ret.push_back(_name + ".Est.yaw");

  ret.push_back(_name + ".Est.S.x");
  ret.push_back(_name + ".Est.S.y");
  ret.push_back(_name + ".Est.S.z");
  ret.push_back(_name + ".Est.S.vx");
  ret.push_back(_name + ".Est.S.vy");
  ret.push_back(_name + ".Est.S.vz");
  ret.push_back(_name + ".Est.S.yaw");

  ret.push_back(_name + ".Est.E.x");
  ret.push_back(_name + ".Est.E.y");
  ret.push_back(_name + ".Est.E.z");
  ret.push_back(_name + ".Est.E.vx");
  ret.push_back(_name + ".Est.E.vy");
  ret.push_back(_name + ".Est.E.vz");
  ret.push_back(_name + ".Est.E.yaw");
  ret.push_back(_name + ".Est.E.pitch");
  ret.push_back(_name + ".Est.E.roll");

  ret.push_back(_name + ".Est.E.pos");
  ret.push_back(_name + ".Est.E.vel");

  ret.push_back(_name + ".Est.E.maxEuler");

  ret.push_back(_name + ".Est.D.covCond");

  // diagnostic variables
  ret.push_back(_name + ".Est.D.AccelPitch");
  ret.push_back(_name + ".Est.D.AccelRoll");
  ret.push_back(_name + ".Est.D.ax_g");
  ret.push_back(_name + ".Est.D.ay_g");
  ret.push_back(_name + ".Est.D.az_g");
  return ret;
};
