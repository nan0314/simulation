#ifndef ROTOR_NAV_INCLUDE_GUARD_HPP
#define ROTOR_NAV_INCLUDE_GUARD_HPP
#include <vector>
#include <nav_msgs/OccupancyGrid.h>


namespace rotor_navigation{

    class Map;
    struct Node;

    using std::vector;

    /// \brief Takes an input position vector and returns it as a string
    std::string to_string(vector<int> pos);

    struct Node{

        vector<int> pos;
        double f,g,h;
        Node *prev;

        /// \brief Default constructor
        Node();

        /// \brief Constructor
        Node(vector<int> pos, Node *prev);

        /// \brief Constructor
        Node(vector<int> pos, double f, double g, double h, Node *prev);

        /// \brief Returns the path from start to this node
        vector<vector<int>> getPath();

        bool operator==(Node right);

        std::string string();

        vector<vector<int>> getSuccessors(Map map);

        double distance(vector<int> pos);
    };



    class Map{

        int height, width, ox, oy;
        std::vector<signed char> data;

        public:

        /// \brief Default Constructor - initializes parameters
        Map();

        /// \brief Constructor - sets height and width and creates map filled with zeros
        Map(int height, int width, int ox, int oy);

        /// \brief Constructor - takes an input map and saves the dimensions and data
        Map(int height, int width, int ox, int oy, std::vector<signed char> data);

        /// \brief Constructor - takes an input OccupancyGrid message and converts it
        /// into a Map object
        Map(nav_msgs::OccupancyGrid map);

        int get_ox();

        int get_oy();

        int get_height();

        int get_width();

        /// \brief Prints the map
        void print();

        /// \brief Sets point in data to the input vale
        /// \param x - x coordinate of point
        /// \param y - y coordinate of point
        /// \param value - value to set point
        void setPoint(int x, int y, int value);

        /// \brief Returns the value of a point in data
        /// \param x - x coordinate of point
        /// \param y - y coordinate of point
        /// \returns the value of the given point in the map
        int getPoint(int x, int y);

        /// \brief Returns the map as an occupancy grid message
        /// \returns the map as an occupancy grid message
        nav_msgs::OccupancyGrid getGrid();

        vector<vector<int>> a_star(vector<int> start, vector<int> end);

    };
}

#endif
