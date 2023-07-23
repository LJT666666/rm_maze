# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;dynamic_reconfigure;geometry_msgs;message_generation;rm_common;rm_msgs;std_msgs;pluginlib".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrm_maze;-limage_transport;-lcv_bridge".split(';') if "-lrm_maze;-limage_transport;-lcv_bridge" != "" else []
PROJECT_NAME = "rm_maze"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
