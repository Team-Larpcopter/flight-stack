#ifndef GRAPH_MSGER_H
#define GRAPH_MSGER_H

#include "utility.h"

struct GraphMsgerParams {
    GraphMsgerParams() = default;
    std::string frame_id;
    int robot_id;
};


class GraphMsger {
public:
    GraphMsger() = default;
    ~GraphMsger() = default;

    void Init(const rclcpp::Node::SharedPtr& nh, const GraphMsgerParams& params);

    void UpdateGlobalGraph(const NodePtrStack& graph) {global_graph_ = graph;};

private:
    rclcpp::Node::SharedPtr nh_;
    GraphMsgerParams gm_params_;
    rclcpp::Publisher<nav_graph_msg::msg::Graph>::SharedPtr graph_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_graph_service_;

    NodePtrStack global_graph_;
    nav_graph_msg::msg::Graph nav_graph_;

    inline bool IsEncodeType(const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || node_ptr->is_goal) {
            return false;
        }
        return true;
    }

    void EncodeGraph(const NodePtrStack& graphIn, nav_graph_msg::msg::Graph& graphOut);

    bool RequestGraphService(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);



};


#endif