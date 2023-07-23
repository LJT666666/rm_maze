//
// Created by heyicheng on 10/18/21.
//
#include "rm_maze/maze.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

using namespace InferenceEngine;
using namespace cv;

PLUGINLIB_EXPORT_CLASS(rm_maze::MazeProc, nodelet::Nodelet)

namespace rm_maze {
void MazeProc::onInit() {
  ros::NodeHandle &nh = getPrivateNodeHandle();
  static ros::CallbackQueue my_queue;
  nh.setCallbackQueue(&my_queue);
  initialize(nh);
  my_thread_ = std::thread([]() {
    ros::SingleThreadedSpinner spinner;
    spinner.spin(&my_queue);
  });
}

void MazeProc::initialize(ros::NodeHandle &nh) {
  //        nh_ = ros::NodeHandle(nh, "opvn_proc");
  //        if(!nh.getParam("xml_path", xml_path_))
  //            ROS_WARN("No xml_path specified");
  //        if(!nh.getParam("bin_path", bin_path_))
  //            ROS_WARN("No bin_path specified");

  maze_cfg_srv_ = new dynamic_reconfigure::Server<rm_maze::MazeConfig>(
      ros::NodeHandle(nh_, "maze_condition"));
  maze_cfg_cb_ = boost::bind(&MazeProc::mazeconfigCB, this, _1, _2);
  maze_cfg_srv_->setCallback(maze_cfg_cb_);

  it_ = make_shared<image_transport::ImageTransport>(nh_);
  corrective_map_debug_ = it_->advertise("/maze/corrective_map", 1);
  map_thresh_debug_ = it_->advertise("/maze/map_thresh", 1);
  follow_trail_debug_pub_ = it_->advertise("/maze/trail", 1);
  map_debug_pub_ = it_->advertise("/maze/map", 1);
  path_debug_pub_ = it_->advertise("/maze/path", 1);

  bag_sub_ = it_->subscribe("/galaxy_camera/galaxy_camera/image_raw", 1,
                            &MazeProc::callback, this);
  target_pub_ = nh.advertise<decltype(target_array_)>("/target_point", 1);
  angle_error_pub_ = nh.advertise<rm_msgs::FollowTrail>("/angle_error", 1);
}

void MazeProc::mazeconfigCB(rm_maze::MazeConfig &config, uint32_t level) {
  if (!dynamic_reconfig_initialized_) {
    config.start_index = start_index_;
    config.end_index = end_index_;
    config.start_send_point = start_send_point_;
    config.angle_error_test = angle_error_test_;
    config.binary_thresh = binary_thresh_;
    config.binary_element = binary_element_;
    config.canny_low_thresh = canny_low_thresh_;
    config.canny_high_thresh = canny_high_thresh_;
    config.start_send_point = start_send_point_;
    config.is_start_trail = is_start_trail_;
    dynamic_reconfig_initialized_ = true;
  } else {
    draw_type_ = DrawImage(config.draw_type);
    binary_thresh_ = config.binary_thresh;
    binary_element_ = config.binary_element;
    canny_low_thresh_ = config.canny_low_thresh;
    canny_high_thresh_ = config.canny_high_thresh;
    if (config.binary_element % 2)
      binary_element_ += 1;

    start_index_ = config.start_index;
    end_index_ = config.end_index;
    angle_error_test_ = config.angle_error_test;
    start_send_point_ = config.start_send_point;
    is_start_trail_ = config.is_start_trail;
    //    is_treasures_found_ = false;
    //    is_path_found_ = false;
  }
  ROS_INFO("dynamic %d", start_index_);
}

void MazeProc::generateMap() {
  Mat img_gry;
  Mat origin =
      imread("/home/ljt666666/polygon_yolox_openvino/Photo_0630_1a.jpg", 1);
  Rect rect(0.185 * origin.cols, 0.185 * origin.rows, 508, 508);
  Mat resize_src = origin(rect);
  node_map_ = resize_src.clone();
  official_resize_img_ = resize_src.clone();
  //  Mat origin = map_warp_img_.clone();
  //  //  Mat origin =
  //  imread("/home/ljt666666/polygon_yolox_openvino/warp_img.png",
  //  //  1);
  //  Rect rect(0.12 * origin.cols, 0.13 * origin.rows, 590, 590);
  //  Mat resize_src = origin(rect);

  /// The original operation
  Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));
  Mat out;
  // 进行膨胀操作
  erode(resize_src, out, element);

  cv::cvtColor(out, img_gry, cv::COLOR_RGB2GRAY);

  //  cv::threshold(img_gry, img_gry, 65, 255, THRESH_BINARY);
  cv::threshold(img_gry, img_gry, 0, 255,
                THRESH_BINARY | THRESH_OTSU); // 二值化

  /// unevenLightCompensate
  //  unevenLightCompensate(resize_src, 32);
  //  //  cv::imshow("unevenLightCompensate", origin_image);
  //
  //  cv::Mat img_thresh2;
  //  cv::threshold(resize_src, img_thresh2, 0, 255,
  //                THRESH_BINARY | THRESH_OTSU); // 二值化
  //
  //  Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));
  //  Mat out;
  //  // 进行膨胀操作
  //  erode(img_thresh2, img_gry, element);

  int map[19][19]{};
  int temp_pixel_value;
  for (int i = 1; i < 11; i++) {
    //      cout << i <<endl;
    for (int j = 1; j < 10; j++) {
      int a_x = j * 0.1 * img_gry.cols,
          a_y = i * 0.1 * img_gry.rows - 0.05 * img_gry.cols;
      //      if(i == 1)
      //        std::cout<< (uint)img_gry.at<uchar>(a_y,a_x) << " ";
      temp_pixel_value = (uint)img_gry.at<uchar>(a_y, a_x);
      if (temp_pixel_value == 0) {
        //        map[i][2*j-1] = 1;
        map[2 * (i - 1)][2 * j - 1] = 1;

        if (i != 1 && i != 10) {
          map[2 * (i - 1) - 1][2 * j - 1] = 1;
          map[2 * (i - 1) + 1][2 * j - 1] = 1;
        } else if (i == 1) {
          map[2 * (i - 1) + 1][2 * j - 1] = 1;
        } else if (i == 10) {
          map[2 * (i - 1) - 1][2 * j - 1] = 1;
        }
        //        cout << i << ",";
        //        cout << j << "--";
        //        cout << 2 * (i - 1) << "," << 2 * j - 1 << " ";
      }
      //      cv::circle(resize_src,Point (a_x,a_y),10,Scalar(0,255,0),-1);

      int b_x = i * 0.1 * img_gry.cols - 0.05 * img_gry.cols,
          b_y = j * 0.1 * img_gry.rows;
      //      if(i == 1)
      //        std::cout<<(uint)img_gry.at<uchar>(b_y,b_x) << std::endl;
      temp_pixel_value = (uint)img_gry.at<uchar>(b_y, b_x);
      if (temp_pixel_value == 0) {
        map[2 * j - 1][2 * (i - 1)] = 1;

        if (i != 1 && i != 10) {
          map[2 * j - 1][2 * (i - 1) - 1] = 1;
          map[2 * j - 1][2 * (i - 1) + 1] = 1;
        } else if (i == 1) {
          map[2 * j - 1][2 * (i - 1) + 1] = 1;
        } else if (i == 10) {
          map[2 * j - 1][2 * (i - 1) - 1] = 1;
        }

        //        cout << j << ",";
        //        cout << i << "--";
        //        cout << 2 * j - 1 << "," << 2 * (i - 1) << " ";
      }
      //      cv::circle(resize_src,Point (b_x,b_y),10,Scalar(255,0,0),-1);
    }
  }

  //  for (int i = 0; i < 19; i++) {
  //    cout << " " <<endl;
  //    for (int j = 0; j < 19; j++) {
  //      cout << map[i][j] << " ";
  //    }
  //  }

  /// Seak for nodes
  vector<Noode> nodes;
  for (int i = 0; i < 19; i++) {
    for (int j = 0; j < 19; j++) {
      if (map[i][j] == 0) {
        /// Normal corner
        int count = 0;
        Noode node;
        // 右
        if (j + 1 < 19 && map[i][j + 1] == 0) {
          count += 1;
          node.directions[3] = 1;
        }
        // 下
        if (i + 1 < 19 && map[i + 1][j] == 0) {
          count += 1;
          node.directions[1] = 1;
        }
        // 左
        if (j - 1 >= 0 && map[i][j - 1] == 0) {
          count += 1;
          node.directions[2] = 1;
        }
        // 上
        if (i - 1 >= 0 && map[i - 1][j] == 0) {
          count += 1;
          node.directions[0] = 1;
        }

        /// Treasures in the edge area
        // up
        bool is_edge_treasure = false;
        if (i == 0 && j != 0 && j != 18 && map[i][j] == 0) {
          if (map[i][j - 1] == 1 && map[i + 1][j] == 1) {
            // left
            node.directions[2] = 1;
            is_edge_treasure = true;
          } else if (map[i][j + 1] == 1 && map[i + 1][j] == 1) {
            // right
            node.directions[3] = 1;
            is_edge_treasure = true;
          }
        }
        // down
        if (i == 18 && j != 0 && j != 18 && map[i][j] == 0) {
          if (map[i][j - 1] == 1 && map[i - 1][j] == 1) {
            // right
            node.directions[3] = 1;
            is_edge_treasure = true;
          } else if (map[i][j + 1] == 1 && map[i - 1][j] == 1) {
            // left
            node.directions[2] = 1;
            is_edge_treasure = true;
          }
        }
        // left
        if (j == 0 && i != 0 && i != 18 && map[i][j] == 0) {
          if (map[i][j + 1] == 1 && map[i - 1][j] == 1) {
            // down
            node.directions[1] = 1;
            is_edge_treasure = true;
          } else if (map[i][j + 1] == 1 && map[i + 1][j] == 1) {
            // up
            node.directions[0] = 1;
            is_edge_treasure = true;
          }
        }
        // right
        if (j == 18 && i != 0 && i != 18 && map[i][j] == 0) {
          if (map[i][j - 1] == 1 && map[i - 1][j] == 1) {
            // down
            node.directions[1] = 1;
            is_edge_treasure = true;
          } else if (map[i][j - 1] == 1 && map[i + 1][j] == 1) {
            // up
            node.directions[1] = 1;
            is_edge_treasure = true;
          }
        }

        /// treasures in the middle area
        bool is_middle_treasure = false;
        if (i != 0 && i != 18 && j != 0 && j != 18) {
          if (map[i - 1][j] + map[i + 1][j] + map[i][j - 1] + map[i][j + 1] >
              2) {
            // Judge the direction
            if (map[i - 1][j] == 0)
              node.directions[0] = 1;
            else if (map[i + 1][j] == 0)
              node.directions[1] = 1;
            else if (map[i][j - 1] == 0)
              node.directions[2] = 1;
            else if (map[i][j + 1] == 0)
              node.directions[3] = 1;

            is_middle_treasure = true;
          }
        }

        if (count > 2) {
          node.x = j;
          node.y = i;
          node.node_type = false;
          nodes.emplace_back(node);
        } else if (count == 2) {
          if (node.directions[0] * node.directions[1] == 0 &&
              node.directions[2] * node.directions[3] == 0) {
            node.x = j;
            node.y = i;
            node.node_type = false;
            nodes.emplace_back(node);
          }
        } else if (is_edge_treasure) {
          node.x = j;
          node.y = i;
          //          node.node_type = true;
          nodes.emplace_back(node);
        } else if (is_middle_treasure) {
          node.x = j;
          node.y = i;
          //          node.node_type = true;
          nodes.emplace_back(node);
        }
      }
    }
  }

  /// Draw nodes
  //  cout << "--------------------" << endl;
  //  for (int i = 0; i < nodes.size(); i++) {
  //    if (nodes[i].node_type)
  //      map[nodes[i].y][nodes[i].x] = 6;
  //    else
  //      map[nodes[i].y][nodes[i].x] = 7;
  //  }
  //
  //  for (int i = 0; i < 19; i++) {
  //    cout << " " << endl;
  //    for (int j = 0; j < 19; j++) {
  //      if (map[i][j] == 6)
  //        cout << "^"
  //             << " ";
  //      else if (map[i][j] == 7)
  //        cout << "*"
  //             << " ";
  //      else
  //        cout << map[i][j] << " ";
  //    }
  //  }

  /// Find union nodes
  for (int i = 0; i < nodes.size(); i++) {
    // up
    if (nodes[i].directions[0] == 1) {
      int k = 2;
      bool is_find = false;
      while (map[nodes[i].y - k][nodes[i].x] == 0) {
        for (int h = 0; h < nodes.size(); h++) {
          if (nodes[i].y - k == nodes[h].y && nodes[i].x == nodes[h].x) {
            NearNode nearnode;
            nearnode.index = h;
            nearnode.distance = k;
            nodes[i].near_nodes.emplace_back(nearnode);
            is_find = true;
          }
        }
        if (is_find)
          break;
        k++;
      }
    }
    // down
    if (nodes[i].directions[1] == 1) {
      int k = 2;
      bool is_find = false;
      while (map[nodes[i].y + k][nodes[i].x] == 0) {
        for (int h = 0; h < nodes.size(); h++) {
          if (nodes[i].y + k == nodes[h].y && nodes[i].x == nodes[h].x) {
            NearNode nearnode;
            nearnode.index = h;
            nearnode.distance = k;
            nodes[i].near_nodes.emplace_back(nearnode);
            is_find = true;
          }
        }
        if (is_find)
          break;
        k++;
      }
    }
    // left
    if (nodes[i].directions[2] == 1) {
      int k = 2;
      bool is_find = false;
      while (map[nodes[i].y][nodes[i].x - k] == 0) {
        for (int h = 0; h < nodes.size(); h++) {
          if (nodes[i].y == nodes[h].y && nodes[i].x - k == nodes[h].x) {
            NearNode nearnode;
            nearnode.index = h;
            nearnode.distance = k;
            nodes[i].near_nodes.emplace_back(nearnode);
            is_find = true;
          }
        }
        if (is_find)
          break;
        k++;
      }
    }
    // right
    if (nodes[i].directions[3] == 1) {
      int k = 2;
      bool is_find = false;
      while (map[nodes[i].y][nodes[i].x + k] == 0) {
        for (int h = 0; h < nodes.size(); h++) {
          if (nodes[i].y == nodes[h].y && nodes[i].x + k == nodes[h].x) {
            NearNode nearnode;
            nearnode.index = h;
            nearnode.distance = k;
            nodes[i].near_nodes.emplace_back(nearnode);
            is_find = true;
          }
        }
        if (is_find)
          break;
        k++;
      }
    }
  }
  //  cout << nodes.size() << endl;
  //  if (nodes.size() != 76)
  //    return;
  //  else
  //    init_flag_ = true;
  nodes_ = nodes;
  //  vector<int> path{};
  //  planPath(nodes_, path);
  //  draw(resize_src, nodes, path);
  //  sendPoints(nodes, path);
}

void MazeProc::planPath(const vector<Noode> &nodes, vector<int> &path) {
  if (is_path_found_)
    return;
  DijkstraMaze maze(nodes);
  HeadNode *G;
  int nodeNum{}, arcNum{};
  cout << "请输入顶点个数，边长个数: ";
  //  cin >> nodeNum >> arcNum;
  nodeNum = nodes.size();
  for (auto &node : nodes) {
    arcNum += node.near_nodes.size();
  }

  G = new HeadNode[nodeNum];
  maze.createGraph(G, nodeNum, arcNum);

  cout << "=============================" << endl;
  cout << "下面开始打印图信息..." << endl;
  maze.printGraph(G, nodeNum);

  cout << "=============================" << endl;
  cout << "下面开始运行dijkstra算法..." << endl;
  maze.path_node_.clear();

  for (int i = 0; i < treasure_list.size() - 1; i++) {
    int start_index = treasure_list[i];
    maze.Dijkstra(G, nodeNum, start_index);
    cout << "=============================" << endl;
    cout << "打印从v" << start_index << "开始所有的最短路径" << endl;
    cout << nodeNum << endl;
    for (int k = 2; k <= nodeNum; k++) {
      if (k == treasure_list[i + 1]) {
        cout << "v" << start_index << "到v" << k << "的最短路径为" << G[k - 1].d
             << ": ";
        maze.printPath(G, k);
        cout << endl;
      }
    }
  }

  for (int &i : maze.path_node_) {
    //    cout << i << " ";
    path.emplace_back(i);
  }
  is_path_found_ = true;
}

void MazeProc::imageProcess(cv_bridge::CvImagePtr &cv_image) {
  if (cv_image->image.empty()) {
    ROS_ERROR("invalid image input");
    return;
  }
  cv_image->image.copyTo(image_raw_);
}

void MazeProc::correctiveMap() {
  if (start_send_point_)
    return;
  Mat origin_image = image_raw_.clone();
  Mat result_image = image_raw_.clone();
  //  Mat origin_image =
  //      imread("/home/ljt666666/polygon_yolox_openvino/origin_map.png", 1);
  //  Mat result_image =
  //      imread("/home/ljt666666/polygon_yolox_openvino/origin_map.png", 1);

  unevenLightCompensate(origin_image, 32);
  //  cv::imshow("unevenLightCompensate", origin_image);

  cv::Mat img_thresh;
  cv::threshold(origin_image, img_thresh, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU); // 二值化

  cv::Mat img_canny;
  cv::Canny(img_thresh, img_canny, 400, 200, 3); // 边缘检测

  //  cv::imshow("canny", img_canny);
  //  waitKey(0);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(
      img_canny, contours, hierarchy, cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE); // 检测记录轮廓，使用CV_RETR_CCOMP建立内外层关系，记录边线两点

  std::vector<std::vector<cv::Point>> conPoly(contours.size());
  std::vector<cv::Mat> resource;
  std::vector<cv::Point> vertexs;
  double area, length;
  double min_target = 800, max_target = 3000;
  for (int i = 0; i < contours.size(); ++i) {
    area = cv::contourArea(contours[i]); // 计算面积

    if (area > min_target && area < max_target) { // 过滤大小块
      length = cv::arcLength(contours[i], true);  // 计算周长
      cv::approxPolyDP(contours[i], conPoly[i], 0.03 * length,
                       true); // 近似多边形

      double ratio = 16 * area / pow(length, 2);
      if (conPoly[i].size() == 4 && ratio < 1.2 &&
          ratio > 0.8) { // 如果是四边形，同时轮廓无parent
        for (int j = 0; j < 4; ++j) {
          vertexs.push_back(conPoly[i][j]);
        }
      }
    }
  }

  if (vertexs.size() != 16) {
    //    isTarget = false;
    return;
  }
  std::sort(vertexs.begin(), vertexs.end(), [](cv::Point &a, cv::Point &b) {
    return (a.x + a.y) < (b.x + b.y);
  });

  std::sort(
      vertexs.begin() + 1, vertexs.end() - 1,
      [](cv::Point &a, cv::Point &b) { return (a.y - a.x) < (b.y - b.x); });

  std::vector<cv::Point> ans_vertexs;
  ans_vertexs.emplace_back(vertexs[0]);
  ans_vertexs.emplace_back(vertexs[1]);
  ans_vertexs.emplace_back(vertexs[vertexs.size() - 1]);
  ans_vertexs.emplace_back(vertexs[vertexs.size() - 2]);
  for (int j = 0; j < 4; ++j) {
    cv::line(result_image, ans_vertexs[j], ans_vertexs[(j + 1) % 4],
             cv::Scalar(255, 0, 0), cv::LINE_8);
  }
  //  cv::imshow("666", result_image);
  //  cv::waitKey(0);
  Mat warp_img;
  std::vector<Point2d> warp_points(4);
  warp_points[0] = {0, 0};
  warp_points[1] = {800, 0};
  warp_points[2] = {800, 800};
  warp_points[3] = {0, 800};
  Mat warp_mat = findHomography(ans_vertexs, warp_points, cv::RANSAC);
  warpPerspective(result_image, warp_img, warp_mat, cv::Size(800, 800));
  //  cv::imshow("warp", warp_img);

  unevenLightCompensate(warp_img, 32);

  cv::Mat img_thresh2;
  cv::threshold(warp_img, img_thresh2, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU); // 二值化

  Rect rect(0.12 * warp_img.cols, 0.13 * warp_img.rows, 590, 590);
  Mat resize_src = img_thresh2(rect);

  //  Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));
  //  Mat out;
  //  // 进行膨胀操作
  //  erode(resize_src, img_thresh2, element);

  //  //  cv::threshold(img_gry, img_gry, 65, 255, THRESH_BINARY);
  //  cv::threshold(img_gry, img_gry, 0, 255, THRESH_BINARY |
  //  THRESH_OTSU);//二值化
  map_warp_img_ = resize_src.clone();
  //  cv::cvtColor(resize_src, resize_src, cv::COLOR_RGB2GRAY);
  cv::threshold(resize_src, resize_src, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU); // 二值化
  Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
  Mat out;
  // 进行膨胀操作
  dilate(resize_src, resize_src, element);
  map_warp_img_ = resize_src.clone();
  sensor_msgs::ImagePtr msg;
  msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", map_warp_img_)
            .toImageMsg();
  corrective_map_debug_.publish(msg);
}

void MazeProc::findTreasures() {
  if (is_treasures_found_)
    return;
  Mat origin_image = image_raw_.clone();
  Mat result_image = image_raw_.clone();
  //  Mat origin_image =
  //      imread("/home/ljt666666/polygon_yolox_openvino/origin_map.png", 1);
  //  Mat result_image =
  //      imread("/home/ljt666666/polygon_yolox_openvino/origin_map.png", 1);

  unevenLightCompensate(origin_image, 32);
  //  cv::imshow("unevenLightCompensate", origin_image);

  cv::Mat img_thresh;
  cv::threshold(origin_image, img_thresh, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU); // 二值化

  cv::Mat img_canny;
  cv::Canny(img_thresh, img_canny, 400, 200, 3); // 边缘检测

  //  cv::imshow("canny", img_canny);
  //  waitKey(0);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(
      img_canny, contours, hierarchy, cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE); // 检测记录轮廓，使用CV_RETR_CCOMP建立内外层关系，记录边线两点

  std::vector<std::vector<cv::Point>> conPoly(contours.size());
  std::vector<cv::Mat> resource;
  std::vector<cv::Point> vertexs;
  double area, length;
  double min_target = 800, max_target = 3000;
  for (int i = 0; i < contours.size(); ++i) {
    area = cv::contourArea(contours[i]); // 计算面积

    if (area > min_target && area < max_target) { // 过滤大小块
      length = cv::arcLength(contours[i], true);  // 计算周长
      cv::approxPolyDP(contours[i], conPoly[i], 0.03 * length,
                       true); // 近似多边形

      double ratio = 16 * area / pow(length, 2);
      if (conPoly[i].size() == 4 && ratio < 1.2 &&
          ratio > 0.8) { // 如果是四边形，同时轮廓无parent
        for (int j = 0; j < 4; ++j) {
          vertexs.push_back(conPoly[i][j]);
        }
      }
    }
  }

  if (vertexs.size() != 16) {
    //    isTarget = false;
    return;
  }
  std::sort(vertexs.begin(), vertexs.end(), [](cv::Point &a, cv::Point &b) {
    return (a.x + a.y) < (b.x + b.y);
  });

  std::sort(
      vertexs.begin() + 1, vertexs.end() - 1,
      [](cv::Point &a, cv::Point &b) { return (a.y - a.x) < (b.y - b.x); });

  std::vector<cv::Point> ans_vertexs;
  ans_vertexs.emplace_back(vertexs[0]);
  ans_vertexs.emplace_back(vertexs[1]);
  ans_vertexs.emplace_back(vertexs[vertexs.size() - 1]);
  ans_vertexs.emplace_back(vertexs[vertexs.size() - 2]);
  for (int j = 0; j < 4; ++j) {
    cv::line(result_image, ans_vertexs[j], ans_vertexs[(j + 1) % 4],
             cv::Scalar(255, 0, 0), cv::LINE_8);
  }
  //  cv::imshow("666", result_image);
  //  cv::waitKey(0);
  Mat warp_img;
  std::vector<Point2d> warp_points(4);
  warp_points[0] = {0, 0};
  warp_points[1] = {800, 0};
  warp_points[2] = {800, 800};
  warp_points[3] = {0, 800};
  Mat warp_mat = findHomography(ans_vertexs, warp_points, cv::RANSAC);
  warpPerspective(result_image, warp_img, warp_mat, cv::Size(800, 800));
  //  cv::imshow("warp", warp_img);
  map_warp_img_ = warp_img.clone();

  unevenLightCompensate(warp_img, 32);

  cv::Mat img_thresh2;
  cv::threshold(warp_img, img_thresh2, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU); // 二值化

  Rect rect(0.12 * warp_img.cols, 0.13 * warp_img.rows, 590, 590);
  Mat resize_src = img_thresh2(rect);

  //  Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));
  //  Mat out;
  //  // 进行膨胀操作
  //  erode(resize_src, img_thresh2, element);

  //  //  cv::threshold(img_gry, img_gry, 65, 255, THRESH_BINARY);
  //  cv::threshold(img_gry, img_gry, 0, 255, THRESH_BINARY |
  //  THRESH_OTSU);//二值化
  map_warp_img_ = resize_src.clone();
  //  cv::cvtColor(resize_src, resize_src, cv::COLOR_RGB2GRAY);
  cv::threshold(resize_src, resize_src, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU); // 二值化
  Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
  Mat out;
  // 进行膨胀操作
  dilate(resize_src, resize_src, element);

  int map[10][10]{};
  int temp_pixel_value;
  for (int i = 1; i < 11; i++) {
    for (int j = 1; j < 11; j++) {
      int a_x = j * 0.1 * resize_src.cols - 0.5 * 0.1 * resize_src.cols,
          a_y = i * 0.1 * resize_src.rows - 0.5 * 0.1 * resize_src.rows;
      //      if(i == 2){
      //        //        cout << a_x << a_y << "6";
      //        std::cout<< (uint)origin_image.at<uchar>(a_y,a_x) << " ";
      //        //        cv::circle(origin_image,Point
      //        (a_x,a_y),10,Scalar(0,255,0),-1);
      //      }
      temp_pixel_value = (uint)resize_src.at<uchar>(a_y, a_x);
      if (temp_pixel_value == 0) {
        Noode treasure_node;
        treasure_node.x = 2 * (j - 1);
        treasure_node.y = 2 * (i - 1);
        treasures_nodes_.emplace_back(treasure_node);
        //        cout << 2*(j - 1) << " " <<  2*(i - 1) << endl;
        cv::circle(map_warp_img_, Point(a_x, a_y), 10, Scalar(0, 255, 0), -1);
      }
    }
  }

  treasure_list.emplace_back(70);
  for (auto &treasures_node : treasures_nodes_) {
    for (int j = 0; j < nodes_.size(); j++) {
      if (treasures_node.x == nodes_[j].x && treasures_node.y == nodes_[j].y) {
        treasure_list.emplace_back(j + 1);
        nodes_[j].node_type = true;
        cout << "treasure_list"
             << " " << j + 1 << endl;
      }
    }
  }
  treasure_list.emplace_back(7);
  cout << "treasure_list_size" << treasure_list.size() << endl;
  if (treasure_list.size() == 10)
    is_treasures_found_ = true;

  vertexs.clear();
  ans_vertexs.clear();
}

void MazeProc::followTrail() {
  Mat image_raw = image_raw_.clone();
  Mat upper_image_raw = image_raw_.clone();
  Rect upper_rect(0, 0, 1280, 512);
  Rect below_rect(0, 512, 1280, 512);
  upper_image_raw = upper_image_raw(upper_rect);
  image_raw = image_raw(below_rect);

  Mat element =
      getStructuringElement(MORPH_RECT, Size(binary_element_, binary_element_));
  Mat img_erode, img_gry, img_thred;
  // 进行膨胀操作
  cv::cvtColor(image_raw, img_gry, cv::COLOR_RGB2GRAY);
  dilate(img_gry, img_erode, element);
  //  cv::threshold(img_erode, img_thred, binary_thresh_, 255,
  //                cv::THRESH_BINARY | cv::THRESH_OTSU);

  cv::threshold(img_erode, img_thred, binary_thresh_, 255, THRESH_BINARY);

  cv::Mat edgeMat, houghMat;
  // Canny边缘检测 二值图像
  Canny(img_thred, edgeMat, canny_low_thresh_, canny_high_thresh_, 3);
  cvtColor(edgeMat, houghMat, CV_GRAY2BGR);
  // 标准的霍夫变换
  vector<Vec2f> lines, lines1;
  HoughLines(edgeMat, lines, 1, CV_PI / 180, 180, 0, 0, 0, 0.4);
  HoughLines(edgeMat, lines1, 1, CV_PI / 180, 180, 0, 0, 2.84, 3.14);
  double error;
  for (size_t i = 0; i < lines.size(); i++) {
    // 根据直线参数表达式绘制相应检测结果
    float rho = lines[i][0], theta = lines[i][1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    line(houghMat, pt1, pt2, Scalar(0, 0, 255), 3, 4);
    cout << "line_x " << pt1.x << " " << pt2.x << endl;
    error = 0.017 * (0.5 * (pt1.x + pt2.x) - 640);
  }
  for (size_t i = 0; i < lines1.size(); i++) {
    // 根据直线参数表达式绘制相应检测结果
    float rho = lines1[i][0], theta = lines1[i][1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    line(houghMat, pt1, pt2, Scalar(0, 0, 255), 3, 4);
    cout << "line1_x " << pt1.x << " " << pt2.x << endl;
    error = 0.017 * (0.5 * (pt1.x + pt2.x) - 640);
  }

  sensor_msgs::ImagePtr msg;
  Mat draw_image;
  msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_raw).toImageMsg();

  if (draw_type_ == DrawImage::DISABLE)
    return;
  else if (draw_type_ == DrawImage::RAW) {
    image_raw.copyTo(draw_image);
    msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  } else if (draw_type_ == DrawImage::BINARY) {
    img_gry.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image)
              .toImageMsg();
  } else if (draw_type_ == DrawImage::MORPHOLOGY) {
    img_erode.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image)
              .toImageMsg();
  } else if (draw_type_ == DrawImage::BARS) {
    img_thred.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image)
              .toImageMsg();
  } else if (draw_type_ == DrawImage::ARMORS) {
    edgeMat.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image)
              .toImageMsg();
  } else if (draw_type_ == DrawImage::ARMORS_VERTEXS) {
    houghMat.copyTo(draw_image);
    msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  } else if (draw_type_ == DrawImage::WARP) {
    upper_image_raw.copyTo(draw_image);
    msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  } else {
    msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
  }
  follow_trail_debug_pub_.publish(msg);

  rm_msgs::FollowTrail angle_error_msg;
  angle_error_msg.error = error;
  angle_error_msg.is_detect_corner = true;
  angle_error_msg.set_point = 0;
  angle_error_pub_.publish(angle_error_msg);
}

void MazeProc::sendPoints(const vector<Noode> &nodes, vector<int> &path) {
  vector<int> actual_path;
  actual_path.emplace_back(70);
  actual_path.emplace_back(path[0]);
  for (int i = 1; i < path.size(); i++) {
    if (nodes[path[i - 1] - 1].x != nodes[path[i + 1] - 1].x &&
        nodes[path[i - 1] - 1].y != nodes[path[i + 1] - 1].y) {
      actual_path.emplace_back(path[i]);
    } else if (nodes[path[i]].node_type == 0) {
      actual_path.emplace_back(path[i]);
    }
  }

  for (int i = 0; i < actual_path.size(); ++i) {
    cout << actual_path[i] << " ";
  }
  cout << endl;

  double actual_x, actual_y;
  for (int i = 0; i < actual_path.size(); i++) {
    actual_y = ((nodes[actual_path[i] - 1].x + 2) * 0.5) * 0.4;
    actual_x = (11 - (nodes[actual_path[i] - 1].y + 2) * 0.5 - 1) * 0.4;
    cout << actual_y << " " << actual_x << endl;
    geometry_msgs::PointStamped temp_point{};
    temp_point.header.frame_id = "odom";
    temp_point.header.stamp = ros::Time(0);
    temp_point.point.x = actual_y;
    temp_point.point.y = actual_x;
    temp_point.point.z = 0.;
    target_array_.is_treasure.emplace_back(nodes[actual_path[i] - 1].node_type);
    target_array_.target_point_array.emplace_back(temp_point);
  }
}

void MazeProc::draw(Mat &resize_img, const vector<Noode> &nodes,
                    vector<int> &path) {
  node_map_ = resize_img.clone();
  for (int i = 0; i < nodes.size(); i++) {
    int a_x = (nodes[i].x + 2) * 0.5 * 0.1 * resize_img.cols -
              0.05 * resize_img.cols,
        a_y = (nodes[i].y + 2) * 0.5 * 0.1 * resize_img.rows -
              0.05 * resize_img.rows;

    if (nodes[i].node_type)
      cv::circle(node_map_, Point(a_x, a_y), 10, Scalar(255, 0, 0), -1);
    else
      cv::circle(node_map_, Point(a_x, a_y), 10, Scalar(0, 255, 0), -1);
    cv::putText(node_map_, to_string(i + 1), cv::Point(a_x - 15, a_y),
                cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 1.5);
  }

  target_path_ = resize_img.clone();

  vector<int> actual_path;
  actual_path.emplace_back(path[0]);
  for (int i = 1; i < path.size(); i++) {
    if (nodes[path[i - 1] - 1].x != nodes[path[i + 1] - 1].x &&
        nodes[path[i - 1] - 1].y != nodes[path[i + 1] - 1].y) {
      actual_path.emplace_back(path[i]);
    } else if (nodes[path[i]].node_type == 0) {
      actual_path.emplace_back(path[i]);
    }
  }
  actual_path.emplace_back(path[path.size() - 1]);

  for (int i = 1; i < actual_path.size(); i++) {
    int starting_point_x =
        (nodes[actual_path[i - 1] - 1].x + 2) * 0.5 * 0.1 * resize_img.cols -
        0.05 * resize_img.cols;
    int starting_point_y =
        (nodes[actual_path[i - 1] - 1].y + 2) * 0.5 * 0.1 * resize_img.rows -
        0.05 * resize_img.rows;
    int terminal_x =
        (nodes[actual_path[i] - 1].x + 2) * 0.5 * 0.1 * resize_img.cols -
        0.05 * resize_img.cols;
    int terminal_y =
        (nodes[actual_path[i] - 1].y + 2) * 0.5 * 0.1 * resize_img.rows -
        0.05 * resize_img.rows;
    line(target_path_, Point(starting_point_x, starting_point_y),
         Point(terminal_x, terminal_y), Scalar(0, 0, 255), 4, 4);
  }
}

void MazeProc::drawNodes() {
  for (int i = 0; i < nodes_.size(); i++) {
    //    cout << i + 1 << " " << nodes_[i].x << " " << nodes_[i].y << endl;
    int a_x = (nodes_[i].x + 2) * 0.5 * 0.1 * node_map_.cols -
              0.05 * node_map_.cols,
        a_y = (nodes_[i].y + 2) * 0.5 * 0.1 * node_map_.rows -
              0.05 * node_map_.rows;

    cv::circle(node_map_, Point(a_x, a_y), 10, Scalar(0, 255, 0), -1);
    cv::putText(node_map_, to_string(i + 1), cv::Point(a_x - 15, a_y),
                cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 1.5);
  }
}

bool MazeProc::isRangle(std::vector<cv::Point> &vertexs) {
  float f1 = (float)(vertexs[1].y - vertexs[0].y) /
             (vertexs[1].x - vertexs[0].x); // 0->1
  float f2 = (float)(vertexs[2].y - vertexs[3].y) /
             (vertexs[2].x - vertexs[3].x); // 3->2
  float f3 = pow((vertexs[3].y - vertexs[0].y), 2) +
             pow((vertexs[3].x - vertexs[0].x), 2); // 3->0
  float f4 = pow((vertexs[2].y - vertexs[1].y), 2) +
             pow((vertexs[2].x - vertexs[1].x), 2); // 2->1
  if (abs(f1 - f2) < 0.5 && abs(f3 - f4) / f4 < 0.1) {
    return true;
  }
  return false;
}

void MazeProc::unevenLightCompensate(cv::Mat &image, int blockSize) {
  if (image.channels() == 3)
    cvtColor(image, image, 7);
  double average = mean(image)[0];
  int rows_new = ceil(double(image.rows) / double(blockSize));
  int cols_new = ceil(double(image.cols) / double(blockSize));
  cv::Mat blockImage;
  blockImage = cv::Mat::zeros(rows_new, cols_new, CV_32FC1);
  for (int i = 0; i < rows_new; i++) {
    for (int j = 0; j < cols_new; j++) {
      int rowmin = i * blockSize;
      int rowmax = (i + 1) * blockSize;
      if (rowmax > image.rows)
        rowmax = image.rows;
      int colmin = j * blockSize;
      int colmax = (j + 1) * blockSize;
      if (colmax > image.cols)
        colmax = image.cols;
      cv::Mat imageROI =
          image(cv::Range(rowmin, rowmax), cv::Range(colmin, colmax));
      double temaver = mean(imageROI)[0];
      blockImage.at<float>(i, j) = temaver;
    }
  }
  blockImage = blockImage - average;
  cv::Mat blockImage2;
  resize(blockImage, blockImage2, image.size(), (0, 0), (0, 0),
         cv::INTER_CUBIC);
  cv::Mat image2;
  image.convertTo(image2, CV_32FC1);
  cv::Mat dst = image2 - blockImage2;
  dst.convertTo(image, CV_8UC1);
}

} // namespace rm_maze
