#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CameraCostmap
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_img;
    ros::Publisher pub_costmap;

    nav_msgs::OccupancyGrid costmap;
    
    // Python 코드의 ROI 파라미터와 일치시켜야 함
    const float ROI_X_MIN = 1.0f;   // 차량 앞 1m 부터 시작
    const float ROI_X_MAX = 20.0f;  // 차량 앞 20m 까지
    const float ROI_WIDTH = 24.0f;  // 좌우 폭 24m (-12m ~ +12m)
    
    // BEV 이미지 해상도 (Python 코드 출력값)
    const int IMG_W = 300;
    const int IMG_H = 237;

    // Costmap 설정
    const float GRID_RES = 0.1f;    // 1칸당 10cm
    const int GRID_W = 300;         // 30m 커버 (가로)
    const int GRID_H = 300;         // 30m 커버 (세로)
    
    // 계산용 변수
    float pixels_per_meter_x;
    float pixels_per_meter_y;

public:
    CameraCostmap()
    {
        // 토픽 설정
        sub_img = nh.subscribe("/drivable/mask/compressed", 1, &CameraCostmap::imageCallback, this);
        pub_costmap = nh.advertise<nav_msgs::OccupancyGrid>("/costmap/camera", 1);

        // 스케일 계산 (이미지 픽셀 / 실제 거리)
        // 세로(X방향): (20m - 1m) = 19m 영역을 237픽셀로 표현
        pixels_per_meter_x = (float)IMG_H / (ROI_X_MAX - ROI_X_MIN); 
        
        // 가로(Y방향): 24m 영역을 300픽셀로 표현
        pixels_per_meter_y = (float)IMG_W / ROI_WIDTH;

        initCostmap();
    }

    void initCostmap()
    {
        costmap.header.frame_id = "base_link";
        costmap.info.resolution = GRID_RES;
        costmap.info.width = GRID_W;
        costmap.info.height = GRID_H;
        
        // Origin: 차량(base_link)이 맵의 중앙 하단쯤에 오도록 설정
        // 여기서는 맵의 중심을 (0,0)에 맞추기 위해 세팅
        costmap.info.origin.position.x = -10.0; // 뒤로 10m 공간
        costmap.info.origin.position.y = -(GRID_W * GRID_RES) / 2.0; // 좌우 중앙 정렬
        costmap.info.origin.position.z = 0.0;
        costmap.info.origin.orientation.w = 1.0;

        costmap.data.resize(GRID_W * GRID_H);
    }

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        cv::Mat frame;
        try
        {
            // 흑백 이미지로 변환해서 받음
            frame = cv_bridge::toCvCopy(msg, "mono8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) return;

        updateCostmap(frame);
    }

    void updateCostmap(const cv::Mat& mask)
    {
        // 1. Costmap 초기화 (0: Free, 100: Lethal)
        // 일단 전체를 '장애물(100)'이나 'Unknown(-1)'으로 밀어버리고
        // 카메라가 "도로다!"라고 한 곳만 0으로 뚫어주는 방식이 안전함.
        // 혹은 반대로 전체 0(Free) -> 도로 아닌 곳 100(Obstacle)
        
        // 여기서는 "전체 0(Free) -> 도로 밖 100(Obstacle)" 전략 사용
        std::fill(costmap.data.begin(), costmap.data.end(), 0); 

        // 2. Grid 순회하며 매핑
        for (int gy = 0; gy < GRID_H; gy++)
        {
            for (int gx = 0; gx < GRID_W; gx++)
            {
                // 현재 그리드 셀의 물리적 좌표 (m)
                float base_x = costmap.info.origin.position.x + (gx + 0.5) * GRID_RES;
                float base_y = costmap.info.origin.position.y + (gy + 0.5) * GRID_RES;

                // ---------------------------------------------------------
                // [좌표 변환] Base_link(m) -> Image(pixel)
                // ---------------------------------------------------------
                
                // 범위 밖(ROI 밖)은 무시하거나 장애물 처리
                if (base_x < ROI_X_MIN || base_x > ROI_X_MAX) continue;
                
                // 이미지 좌표 계산
                // 이미지 V (세로): 위쪽이 0, 아래쪽이 237
                // 차량은 아래쪽에 있고, X가 커질수록 위로 올라감 (V 감소)
                // X=1.0m 일 때 V=237(맨 아래), X=20.0m 일 때 V=0(맨 위)
                int v = IMG_H - (int)((base_x - ROI_X_MIN) * pixels_per_meter_x);

                // 이미지 U (가로): 왼쪽이 0, 오른쪽이 300
                // Base_link Y는 왼쪽이 +, 오른쪽이 -
                // ROI는 -12m ~ +12m. 중앙(Y=0)이 U=150
                int u = (IMG_W / 2) + (int)(-base_y * pixels_per_meter_y); 
                // 주의: Y축 방향이 반대면 -base_y 부호를 +base_y로 변경

                // 3. 이미지 범위 체크 및 값 할당
                if (u >= 0 && u < IMG_W && v >= 0 && v < IMG_H)
                {
                    // 픽셀 값 확인 (0: 배경, 255: 도로)
                    unsigned char val = mask.at<unsigned char>(v, u);
                    
                    int index = gy * GRID_W + gx;

                    if (val < 100) // 어두운 색(검은색) = 도로 아님 = 장애물
                    {
                        costmap.data[index] = 60; // 벽 생성
                    }
                    else // 밝은 색(초록색/흰색) = 도로 = 주행 가능
                    {
                         // 이미 0으로 초기화했으므로 놔두거나 명시적 0
                         costmap.data[index] = 0;
                    }
                }
                else
                {
                    // ROI 안쪽(거리상)인데 이미지 밖으로 나감 -> 알 수 없음 or 장애물
                    // 보통은 그냥 둠
                }
            }
        }

        // 4. 발행
        costmap.header.stamp = ros::Time::now();
        pub_costmap.publish(costmap);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_costmap_node");
    CameraCostmap cc;
    ros::spin();
    return 0;
}