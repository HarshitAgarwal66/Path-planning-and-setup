#include <ros/ros.h>
#include <math.h>
#include <queue>
#include <vector>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <ros/console.h>
#define FMAX 999999999.99

    geometry_msgs::PoseArray pa;
    int rows = 1000, columns = 1000, size = rows * columns;
    bool visited[1000000];
    float distance[1000000];
    int prev[1000000];
    int source = 1, destination = 1; // Give source and destination
    int dr[] = {1, -1, 0, 0, 1, 1, -1, -1}; // Direction vectors
    int dc[] = {0, 0, 1, -1, 1, -1, 1, -1};
    struct node
    {
        int index;
        float dist;
        node(int index, float dist)
            : index(index), dist(dist)
        {
        } 
    };
    struct compareDist
    {
        bool operator()(node const& n1, node const& n2)
        {
            return n1.dist > n2.dist;
        }
    };

    // Priority queue
    std::priority_queue <node, std::vector<node>, compareDist> pq;

    int index(int r, int c)
    {
        return (r * 1000) + c;
    }

    void init()
    {
        std::cout << "init";
        for(int i = 0; i < size; i++)
        {
            distance[i] = FMAX;
            visited[i] = false;
            prev[i] = 99999999;
        }   
    }

    float dist_(int index1, int index2)
    {
        int r1, c1, r2, c2;
        r1 = index1 / columns; r2 = index2 / columns;
        c1 = index1 - (r1 * 1000); c2 = index2 - (r2 * 1000);
        return sqrt(pow(r1 - r2, 2) + pow(c1 - c2, 2));
    }

    void dijkstra(const nav_msgs::OccupancyGrid& map)
    {
        prev[source] = 0;
        node first = {source, 0.0}; // Define source
        pq.push(first);
        while(!pq.empty())
        {
            node temp = pq.top();
            pq.pop();
            int nodeIndex = temp.index;
            float nodeDist = temp.dist;
            visited[nodeIndex] = true;
            int r = nodeIndex / columns;
            int c = nodeIndex - (r * columns);
            int rr, cc;
            for(int i = 0; i < 8; i++) // to calculate neighbours
            {
                rr = r + dr[i];
                cc = c + dc[i];

                if(rr < 0 || rr >= 1000 || cc < 0 || cc >= 1000 || visited[index(rr, cc)] == true)
                    continue;

                if(map.data[index(rr, cc)] == 100)
                {
                    visited[index(rr, cc)] = true; // Marking blocked paths as visited
                    continue;
                }
                else
                {
                    node neighbour(index(rr, cc), dist_(nodeIndex, index(rr, cc)));
                    float alt = nodeDist + neighbour.dist;
                    if(alt < distance[index(rr, cc)])
                    {
                        visited[index(rr, cc)] = true;
                        distance[index(rr, cc)] = alt;
                        prev[index(rr, cc)] = nodeIndex;
                        node next(index(rr, cc), alt);

                        pq.push(next);
                    }
                    if(visited[destination] == true)
                        break;
                }
            }
            if(visited[destination] == true)
                break;
        }

        std::vector <int> path;
        // prev contains the path. Trace it back to get the path.
        path.push_back(destination);
        while(true)
        {
            path.push_back(prev[path.back()]);
            if(path.back() == 0)
                break;
        }

        for(int i = 0; i < path.size(); i++)
        {
            int x, y;
            x = path.back() / columns;
            y = path.back() - (x * columns);
            path.pop_back();
            geometry_msgs::Pose p;
            p.position.x = x;
            p.position.y = y;
            p.position.z = 0;
            pa.poses.push_back(p);
        }

    }



    int main(int argc, char **argv)
    {
        init();
        distance[source] = 0;
        visited[source] = true;
        ros::init(argc, argv, "dijkstra");
        ros::NodeHandle n("~");
        ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("/static_map");
        nav_msgs::GetMap srv;
        client.call(srv);
        nav_msgs::OccupancyGrid my_map = srv.response.map;
        ros::Publisher pose_array_pub = n.advertise<geometry_msgs::PoseArray>("/poseArray", 1);
        pa.header.frame_id = "map";
        dijkstra(my_map);

        while(ros::ok())
        {
            pose_array_pub.publish(pa);
        }
        ros::spin();    
    }
