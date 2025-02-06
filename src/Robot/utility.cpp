#include "utility.hpp"

double l2_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

std::vector<double> savgolFilter(const std::vector<double> &data, int windowSize, int polyOrder) {
    if (windowSize % 2 == 0 || windowSize < 1)
        throw std::invalid_argument("Window size must be an odd positive integer.");

    if (polyOrder >= windowSize)
        throw std::invalid_argument("Polynomial order must be less than the window size.");

    int halfWindow = (windowSize - 1) / 2;
    Eigen::MatrixXd A(windowSize, polyOrder + 1);

    for (int i = 0; i < windowSize; ++i) {
        for (int j = 0; j <= polyOrder; ++j)
            A(i, j) = std::pow(i - halfWindow, j);
    }

    Eigen::VectorXd coeff = (A.transpose() * A).inverse() * A.transpose() * Eigen::VectorXd::Ones(windowSize);

    std::vector<double> filteredData(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        double smoothedValue = 0.0;

        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = std::clamp(static_cast<int>(i) + j, 0, static_cast<int>(data.size()) - 1);

            int coeffIndex = j + halfWindow; // This should always be valid between 0 and windowSize - 1
            if (coeffIndex < 0 || coeffIndex >= coeff.size())
                continue; // Avoid out-of-bounds access

            smoothedValue += coeff(coeffIndex) * data[idx];
        }
        filteredData[i] = smoothedValue;
    }

    return filteredData;
}

geometry_msgs::PoseStamped makePose(double x, double y, double theta) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta);
    pose.pose.orientation = tf2::toMsg(quaternion);
    return pose;
}

std::vector<double> transform_lg(double x, double y, double X, double Y, double PSI) {
    Eigen::Matrix3d R_r2i;
    R_r2i << std::cos(PSI), -std::sin(PSI), X,
            std::sin(PSI), std::cos(PSI), Y,
            0, 0, 1;
    Eigen::Matrix3d R_i2r = R_r2i.inverse();
    Eigen::Vector3d pi(x, y, 1);
    Eigen::Vector3d pr = R_i2r * pi;
    std::vector<double> lg = {pr(0), pr(1)};

    return lg;
}
