//
// Created by heyicheng on 10/18/21.
//

#ifndef SRC_MAZE_H
#define SRC_MAZE_H

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <inference_engine.hpp>
#include <mutex>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <rm_maze/MazeConfig.h>
#include <rm_msgs/FollowTrail.h>
#include <rm_msgs/TargetDetection.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_msgs/TargetPointArray.h>
#include <ros/ros.h>
#include <thread>

using namespace InferenceEngine;
using namespace cv;
using namespace std;

namespace rm_maze {

struct NearNode {
  int index;
  double distance;
  //  Points(){}
  //  Points(double x,double y):x(x),y(y){}
  //  bool operator==(const Points&p)const{
  //    return sig(x-p.x)==0&&sig(y-p.y)==0;
  //  }
};

struct Noode {
  bool node_type{};
  int directions[4]{};
  vector<NearNode> near_nodes;
  int index;
  int x, y;
};

enum class DrawImage {
  DISABLE = 0,
  RAW = 1,
  BINARY = 2,
  MORPHOLOGY = 3,
  BARS = 4,
  ARMORS = 5,
  ARMORS_VERTEXS = 6,
  BARS_ARMORS = 7,
  WARP = 8
};

class MazeProc : public nodelet::Nodelet {
public:
  void onInit() override;

  void initialize(ros::NodeHandle &nh);

  void imageProcess(cv_bridge::CvImagePtr &cv_image);

  void correctiveMap();

  void findTreasures();

  void generateMap();

  void planPath(const vector<Noode> &nodes, vector<int> &path);

  void followTrail();

  void sendPoints(const vector<Noode> &nodes, vector<int> &path);

  void draw(Mat &resize_img, const vector<Noode> &nodes, vector<int> &path);

  void drawNodes();

  bool isRangle(std::vector<cv::Point> &vertexs);

  void unevenLightCompensate(cv::Mat &image, int blockSize);

private:
  rm_msgs::TargetPointArray target_array_;
  ros::Publisher target_pub_;
  ros::NodeHandle nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  ros::Publisher angle_error_pub_;
  image_transport::Publisher corrective_map_debug_;
  image_transport::Publisher map_thresh_debug_;
  image_transport::Publisher follow_trail_debug_pub_;
  image_transport::Publisher map_debug_pub_;
  image_transport::Publisher path_debug_pub_;
  image_transport::CameraSubscriber cam_sub_;
  image_transport::Subscriber bag_sub_;
  std::thread my_thread_;

  /// generate map
  vector<Noode> nodes_;

  /// find treasures
  bool find_treasure_anew_ = false;
  vector<Noode> treasures_nodes_;
  bool is_treasures_found_ = false;

  /// plan path
  bool is_path_found_ = false;

  /// follow trail
  bool is_start_trail_ = false;
  double angle_error_test_{};
  double error_ratio_ = 0.2;

  /// send points
  bool start_send_point_ = false;
  bool send_debug_point_ = false;
  /// image
  Mat node_map_;
  Mat official_resize_img_;
  Mat map_warp_img_;
  Mat image_raw_; // predict image
  Mat target_path_;

  /// draw
  DrawImage draw_type_{};
  int binary_thresh_ = 90;
  int binary_element_ = 1;
  int canny_low_thresh_ = 150;
  int canny_high_thresh_ = 150;

  int start_index_ = 69;
  int end_index_ = 3;
  bool dynamic_reconfig_initialized_ = false;
  bool dynamicx_reconfig_flag_ = false;
  bool init_flag_ = false;
  vector<int> treasure_list{};
  //  vector<int> treasure_list{70, 45, 28, 8};
  //  vector<int> treasure_list{70, 45, 28, 8, 16, 32, 49, 69, 61, 7};
  //  vector<int> treasure_list{70, 69, 61, 55, 49, 28, 22, 16, 8, 7};
  dynamic_reconfigure::Server<rm_maze::MazeConfig>
      *maze_cfg_srv_; // server of dynamic config about armor
  dynamic_reconfigure::Server<rm_maze::MazeConfig>::CallbackType maze_cfg_cb_;
  void mazeconfigCB(rm_maze::MazeConfig &config, uint32_t level);

  void callback(const sensor_msgs::ImageConstPtr &img) {
    target_array_.target_point_array.clear();
    target_array_.is_treasure.clear();
    boost::shared_ptr<cv_bridge::CvImage> temp =
        boost::const_pointer_cast<cv_bridge::CvImage>(
            cv_bridge::toCvShare(img, "bgr8"));
    auto predict_start = std::chrono::high_resolution_clock::now();
    imageProcess(temp);
    correctiveMap();
    if (is_start_trail_) {
      followTrail();
    }

    if (init_flag_ && start_send_point_) {
      treasures_nodes_.erase(treasures_nodes_.begin(), treasures_nodes_.end());
      treasure_list.erase(treasure_list.begin(), treasure_list.end());
      findTreasures();
      vector<int> path{};
      if (is_treasures_found_ && !is_path_found_) {
        planPath(nodes_, path);
        draw(official_resize_img_, nodes_, path);
        sendPoints(nodes_, path);
        target_pub_.publish(target_array_);
      }
      //      if(resend_point_){
      //        resend_point_ = false;
      //        sendPoints(nodes_, path);
      //      }

      map_debug_pub_.publish(
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", node_map_)
              .toImageMsg());
      path_debug_pub_.publish(
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", target_path_)
              .toImageMsg());
      map_thresh_debug_.publish(
          cv_bridge::CvImage(std_msgs::Header(), "mono8", map_warp_img_)
              .toImageMsg());

      return;
    }
    if (send_debug_point_) {
      send_debug_point_ = false;
      geometry_msgs::PointStamped temp_point{};
      temp_point.header.frame_id = "odom";
      temp_point.header.stamp = ros::Time(0);
      temp_point.point.x = 2;
      temp_point.point.y = 0;
      temp_point.point.z = 0.;
      //      target_array_.is_treasure.emplace_back(
      //          nodes_[actual_path[i] - 1].node_type);
      geometry_msgs::PointStamped temp_point1{};
      temp_point1.header.frame_id = "odom";
      temp_point1.header.stamp = ros::Time(0);
      temp_point1.point.x = 2;
      temp_point1.point.y = 2;
      temp_point1.point.z = 0.;
      target_array_.target_point_array.emplace_back(temp_point);
      target_array_.target_point_array.emplace_back(temp_point1);
      target_pub_.publish(target_array_);
    }
    if (!init_flag_)
      generateMap();
    init_flag_ = true;

    auto predict_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> infer_time = predict_end - predict_start;
    //        ROS_INFO("infer_time: %f", infer_time.count());
  }
};

struct Nodee { // 定义表结点
  int adjvex;  // 该边所指向的顶点的位置
  int weight;  // 边的权值
  Nodee *next; // 下一条边的指针
};

struct HeadNode { // 定义头结点
  int nodeName;   // 顶点信息
  int inDegree;   // 入度
  int d; // 表示当前情况下起始顶点至该顶点的最短路径,初始化为无穷大
  bool
      isKnown; // 表示起始顶点至该顶点的最短路径是否已知,true表示已知，false表示未知
  int parent;  // 表示最短路径的上一个顶点
  Nodee *link; // 指向第一条依附该顶点的边的指针
};

/// dijkstra
class DijkstraMaze {
public:
  DijkstraMaze(vector<Noode> nodes) : input_nodes_(nodes) {}
  vector<Noode> input_nodes_;
  vector<int> path_node_;
  // G表示指向头结点数组的第一个结点的指针
  // nodeNum表示结点个数
  // arcNum表示边的个数
  void createGraph(HeadNode *G, int nodeNum, int arcNum) {
    cout << "开始创建图(" << nodeNum << ", " << arcNum << ")" << endl;
    // 初始化头结点
    for (int i = 0; i < nodeNum; i++) {
      G[i].nodeName = i + 1; // 位置0上面存储的是结点v1,依次类推
      G[i].inDegree = 0;     // 入度为0
      G[i].link = NULL;
    }
    //    for (int j = 0; j < arcNum; j++) {
    //      int begin, end, weight;
    //      cout << "请依次输入 起始点v， 结束点v ，之间边的权值: ";
    //      cin >> begin >> end >> weight;
    //      // 创建新的结点插入链接表
    //      Nodee* node = new Nodee;
    //      node->adjvex = end - 1;
    //      node->weight = weight;
    //      ++G[end - 1].inDegree; //入度加1
    //      //插入链接表的第一个位置
    //      node->next = G[begin - 1].link;
    //      G[begin - 1].link = node;
    //    }
    for (int i = 0; i < input_nodes_.size(); i++) {
      int begin, end, weight;
      cout << "请依次输入 起始点v， 结束点v ，之间边的权值: ";
      //      cin >> begin >> end >> weight;
      begin = i + 1;
      for (int j = 0; j < input_nodes_[i].near_nodes.size(); j++) {
        end = input_nodes_[i].near_nodes[j].index + 1;
        weight = input_nodes_[i].near_nodes[j].distance;
        // 创建新的结点插入链接表
        Nodee *node = new Nodee;
        node->adjvex = end - 1;
        node->weight = weight;
        ++G[end - 1].inDegree; // 入度加1
        // 插入链接表的第一个位置
        node->next = G[begin - 1].link;
        G[begin - 1].link = node;
      }
    }
  }

  void printGraph(HeadNode *G, int nodeNum) {
    for (int i = 0; i < nodeNum; i++) {
      //      cout << "结点v" << G[i].nodeName << "的入度为";
      //      cout << G[i].inDegree << ", 以它为起始顶点的边为: ";
      Nodee *node = G[i].link;
      while (node != NULL) {
        //        cout << "v" << G[node->adjvex].nodeName << "(权:" <<
        //        node->weight << ")"
        //             << " ";
        node = node->next;
      }
      cout << endl;
    }
  }

  // 得到begin->end权重
  int getWeight(HeadNode *G, int begin, int end) {
    Nodee *node = G[begin - 1].link;
    while (node) {
      if (node->adjvex == end - 1) {
        return node->weight;
      }
      node = node->next;
    }
  }

  // 从start开始，计算其到每一个顶点的最短路径
  void Dijkstra(HeadNode *G, int nodeNum, int start) {
    // 初始化所有结点
    for (int i = 0; i < nodeNum; i++) {
      G[i].d = 666666666;   // 到每一个顶点的距离初始化为无穷大
      G[i].isKnown = false; // 到每一个顶点的距离为未知数
    }
    G[start - 1].d = 0;       // 到其本身的距离为0
    G[start - 1].parent = -1; // 表示该结点是起始结点
    while (true) {
      //==== 如果所有的结点的最短距离都已知, 那么就跳出循环
      int k;
      bool ok = true; // 表示是否全部ok
      for (k = 0; k < nodeNum; k++) {
        // 只要有一个顶点的最短路径未知,ok就设置为false
        if (!G[k].isKnown) {
          ok = false;
          break;
        }
      }
      if (ok)
        return;
      //==========================================

      //==== 搜索未知结点中d最小的,将其变为known
      //==== 这里其实可以用最小堆来实现
      int i;
      int minIndex = -1;
      for (i = 0; i < nodeNum; i++) {
        if (!G[i].isKnown) {
          if (minIndex == -1)
            minIndex = i;
          else if (G[minIndex].d > G[i].d)
            minIndex = i;
        }
      }
      //===========================================

      //      cout << "当前选中的结点为: v" << (minIndex + 1) << endl;
      G[minIndex].isKnown = true; // 将其加入最短路径已知的顶点集
      // 将以minIndex为起始顶点的所有的d更新
      Nodee *node = G[minIndex].link;
      while (node != NULL) {
        int begin = minIndex + 1;
        int end = node->adjvex + 1;
        int weight = getWeight(G, begin, end);
        if (G[minIndex].d + weight < G[end - 1].d) {
          G[end - 1].d = G[minIndex].d + weight;
          G[end - 1].parent = minIndex; // 记录最短路径的上一个结点
        }
        node = node->next;
      }
    }
  }

  // 打印到end-1的最短路径
  void printPath(HeadNode *G, int end) {
    //    int path_node[]{};
    if (G[end - 1].parent == -1) {
      cout << "v" << end;
      //      cout << "(" << input_nodes_[end-1].y << "," <<
      //      input_nodes_[end-1].x << ")";
      //      path_node_.emplace_back(end);
    } else if (end != 0) {
      printPath(G, G[end - 1].parent +
                       1); // 因为这里的parent表示的是下标，从0开始，所以要加1
      cout << " -> v" << end;

      //      cout << "(" << input_nodes_[end-1].y << "," <<
      //      input_nodes_[end-1].x << ")";
      path_node_.emplace_back(end);
    }
  }
};

} // namespace rm_maze

#endif // SRC_MAZE_H
