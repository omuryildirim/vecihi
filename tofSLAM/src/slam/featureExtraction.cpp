#include "utility.h"
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"

struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer {
public:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTOF;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    // create pointRange float array
    float pointRange[0] = {};
    sensor_msgs::msg::PointCloud2 pointCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction() {
        subTOF = node->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 1,
                                                             std::bind(&FeatureExtraction::tofMessageHandler, this,
                                                                       std::placeholders::_1));

        pubCornerPoints = node->advertise<sensor_msgs::msg::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = node->advertise<sensor_msgs::msg::PointCloud2>("lio_sam/feature/cloud_surface", 1);

        initializationValue();
    }

    void initializationValue() {
        cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN * Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
        cloudLabel = new int[N_SCAN * Horizon_SCAN];
    }

    void tofMessageHandler(const sensor_msgs::msg::PointCloud2::SharedPtr tof_data) {
        // Process the tof_data string to populate cloudInfo and extractedCloud if needed
        // For this example, assume tof_data contains the serialized lio_sam::cloud_info message

        // Deserialize tof_data to cloudInfo (this part depends on how the data is serialized)
        // Example: cloudInfo = deserializeCloudInfo(tof_data);

        // Dummy function call (replace with actual deserialization logic)
        pointCloud = *tof_data;
        // convert point cloud to pcl format
        fromROSMsg(pointCloud, *extractedCloud);

        calculateSmoothness();
        markOccludedPoints();
        extractFeatures();
        publishFeatureCloud();
    }

    void calculateSmoothness() {
        int cloudSize = extractedCloud->points.size();

        if (sizeof(pointRange) == cloudSize) {
            for (int i = 5; i < cloudSize - 5; i++) {
                float diffRange = pointRange[i - 5] + pointRange[i - 4]
                                  + pointRange[i - 3] + pointRange[i - 2]
                                  + pointRange[i - 1] - pointRange[i] * 10
                                  + pointRange[i + 1] + pointRange[i + 2]
                                  + pointRange[i + 3] + pointRange[i + 4]
                                  + pointRange[i + 5];

                cloudCurvature[i] = diffRange * diffRange;

                cloudNeighborPicked[i] = 0;
                cloudLabel[i] = 0;
                cloudSmoothness[i].value = cloudCurvature[i];
                cloudSmoothness[i].ind = i;
            }
        }
    }

    void markOccludedPoints() {
        int cloudSize = extractedCloud->points.size();
        if (sizeof(pointRange) == cloudSize) {
            for (int i = 5; i < cloudSize - 6; ++i) {
                float depth1 = pointRange[i];
                float depth2 = pointRange[i + 1];
                int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

                if (columnDiff < 10) {
                    if (depth1 - depth2 > 0.3) {
                        cloudNeighborPicked[i - 5] = 1;
                        cloudNeighborPicked[i - 4] = 1;
                        cloudNeighborPicked[i - 3] = 1;
                        cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    } else if (depth2 - depth1 > 0.3) {
                        cloudNeighborPicked[i + 1] = 1;
                        cloudNeighborPicked[i + 2] = 1;
                        cloudNeighborPicked[i + 3] = 1;
                        cloudNeighborPicked[i + 4] = 1;
                        cloudNeighborPicked[i + 5] = 1;
                        cloudNeighborPicked[i + 6] = 1;
                    }
                }

                float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
                float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

                if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                    cloudNeighborPicked[i] = 1;
            }
        }
    }

    void extractFeatures() {
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; i++) {
            surfaceCloudScan->clear();

            for (int j = 0; j < 6; j++) {
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) {
                        largestPickedNum++;
                        if (largestPickedNum <= 20) {
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(
                                int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(
                                int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold) {
                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(
                                int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(
                                int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    void freeCloudInfoMemory() {
        /* TODO:
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
        */
    }

    void publishFeatureCloud() {
        freeCloudInfoMemory();
        /* TODO:
        cloudInfo.cloud_corner = publishCloud(pubCornerPoints, cornerCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
        */
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    FeatureExtraction FE;
    RCLCPP_INFO(FE.node->get_logger(), "\033[1;32m----> Feature Extraction Started.\033[0m");
    rclcpp::spin(FE.node);
    rclcpp::shutdown();

    return 0;
}
