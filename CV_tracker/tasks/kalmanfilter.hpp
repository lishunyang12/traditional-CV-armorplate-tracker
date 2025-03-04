#include <opencv2/opencv.hpp>

const float processnoise_factor = 0.15f;
const float measurementnoise_factor = 0.05f;

class kalman_filter{
private:
    cv::KalmanFilter kalman;
    cv::Mat state;       // [x, y, z, vx, vy, vz]
    cv::Mat measurement; // [x, y, z]

    void setTransitionMatrix(float dt) {
        kalman.transitionMatrix = (cv::Mat_<float>(6, 6) <<
            1, 0, 0, dt,  0,  0,
            0, 1, 0,  0, dt,  0,
            0, 0, 1,  0,  0, dt,
            0, 0, 0,  1,  0,  0,
            0, 0, 0,  0,  1,  0,
            0, 0, 0,  0,  0,  1);
    }

public:
    kalman_filter(float dt=0.011) {
        // Initialize Kalman filter with 6 dynamic states (x, y, z, vx, vy, vz) and 3 measured states (x, y, z)
        kalman.init(6, 3, 0);

        // State: [x, y, z, vx, vy, vz]
        state = cv::Mat::zeros(6, 1, CV_32F);
        // Measurement: [x, y, z]
        measurement = cv::Mat::zeros(3, 1, CV_32F);

        // Transition matrix
        kalman.transitionMatrix = (cv::Mat_<float>(6, 6) <<
            1, 0, 0, dt,  0,  0,
            0, 1, 0,  0, dt,  0,
            0, 0, 1,  0,  0, dt,
            0, 0, 0,  1,  0,  0,
            0, 0, 0,  0,  1,  0,
            0, 0, 0,  0,  0,  1);

        // Measurement matrix
        kalman.measurementMatrix = (cv::Mat_<float>(3, 6) <<
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0);

        // Process noise covariance
        cv::setIdentity(kalman.processNoiseCov, cv::Scalar(processnoise_factor));
        // Measurement noise covariance
        cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar(measurementnoise_factor));
        // Error covariance post
        //cv::setIdentity(kalman.errorCovPost, cv::Scalar(0.8));

        // Initial state (can be set externally if needed)
        kalman.statePost = state.clone();
    }

    cv::Point3f getFinalEstimation() const {
        return cv::Point3f(
            kalman.statePost.at<float>(0), // X
            kalman.statePost.at<float>(1), // Y
            kalman.statePost.at<float>(2)  // Z
        );
    }

    // Predict the next position
    void predict(float dt) {
        // Update transition matrix based on the time interval
        setTransitionMatrix(dt);
        // Predict the state
        kalman.predict();
    }

    // Update the filter with a new measurement
    cv::Point3f update(float x, float y, float z) {
        measurement.at<float>(0) = x;
        measurement.at<float>(1) = y;
        measurement.at<float>(2) = z;

        cv::Mat estimated = kalman.correct(measurement); 
        return cv::Point3f(
            estimated.at<float>(0),
            estimated.at<float>(1),
            estimated.at<float>(2)
        );
    }
};