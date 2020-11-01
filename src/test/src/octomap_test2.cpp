#include <rclcpp/rclcpp.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

void print_query_info(octomap::point3d query, octomap::OcTreeNode *node)
{
    if (node != NULL)
    {
        std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
    }
    else
        std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}
void exec()
{

    std::cout << std::endl;
    std::cout << "generating example map" << std::endl;

    octomap::OcTree tree(0.01); // create empty tree with resolution 0.1

    // insert some measurements of occupied cells
    for (int x = -20; x < 20; x++)
    {
        for (int y = -20; y < 20; y++)
        {
            for (int z = -20; z < 20; z++)
            {
                if (!(x == -10 && y == -10 && z == -10))
                {
                    for (int count = 0; count < 2; count++)
                    {
                        octomap::point3d endpoint((float)x * 0.05f, (float)y * 0.05f, (float)z * 0.05f);
                        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
                    }
                }
            }
        }
    }

    std::cout << std::endl;
    std::cout << "performing some queries:" << std::endl;

    octomap::point3d query(0., 0., 0.);
    octomap::OcTreeNode *result = tree.search(query);
    print_query_info(query, result);

    query = octomap::point3d(-1., -1., -1.);
    result = tree.search(query);
    print_query_info(query, result);

    query = octomap::point3d(1., 1., 1.);
    result = tree.search(query);
    print_query_info(query, result);

    for (int i = -12; i < 12; i++)
    {
        query = octomap::point3d(0.1 * (float)i, 0.1 * (float)i, 0.1 * (float)i);
        result = tree.search(query);
        print_query_info(query, result);
    }
    std::cout << std::endl;
    tree.writeBinary("src/test/output/simple_tree.bt");
    std::cout << "wrote example file simple_tree.bt" << std::endl
              << std::endl;
    std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
    std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl
              << std::endl;
}

int main(int argc, char **argv)
{
    //Initialize
    rclcpp::init(argc, argv);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("octomap_test");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    exec();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}