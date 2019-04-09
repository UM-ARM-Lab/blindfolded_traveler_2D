#ifndef BTP_CHS
#define BTP_CHS
#include "graph_planner/2d_obstacles.hpp"
#include "graph_planner/graph_visualization.hpp"
#include "states/state.hpp"
#include "observations.hpp"
#include "beliefs/belief.hpp"
#include "arc_utilities/voxel_grid.hpp"
#include <random>

namespace BTP
{
    class VoxelGrid2D : public VoxelGrid::VoxelGrid<uint8_t>
    {
    public:
        VoxelGrid2D (double cell_size) :
            VoxelGrid::VoxelGrid<uint8_t>(Eigen::Isometry3d::Identity(),
                                          cell_size, cell_size, cell_size,
                                          1.0, 1.0, cell_size, false)
        {}

        void set(double x, double y, bool occ)
        {
            SetValue(x, y, 0.0, occ);
        }

        bool get(double x, double y) const
        {
            return GetImmutable(x, y, 0.0).first;
        }

        void subtract(const VoxelGrid2D& other)
        {
            //TODO: Checks here. "other" grid needs to be same size and resolution

            
            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    if(other.GetImmutable(x_index, y_index, 0).first > 0)
                    {
                        SetValue(x_index, y_index, 0, 0);
                    }
                }
            }
        }

        double intersectFraction(const VoxelGrid2D& other) const
        {
            int overlap = 0;
            int total = 0;

            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    if( GetImmutable(x_index, y_index, 0).first)
                    {
                        total++;
                        if(other.GetImmutable(x_index, y_index, 0).first)
                        {
                            overlap++;
                        }
                    }
                }
            }
            return (double)overlap / (double)total;
        }

        visualization_msgs::Marker toVisualizationMsg(std_msgs::ColorRGBA color) const
        {
            visualization_msgs::Marker m;
            m.header.frame_id = "/graph_frame";
            m.type = visualization_msgs::Marker::CUBE_LIST;
            m.scale.x = GetCellSizes().x();
            m.scale.y = GetCellSizes().y();
            m.scale.z = GetCellSizes().z();
            m.color = color;

            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    // Convert grid indices into a real-world location
                    const Eigen::Vector4d location = GridIndexToLocation(x_index, y_index, 0);
                    geometry_msgs::Point new_point;
                    new_point.x = location(0);
                    new_point.y = location(1);
                    new_point.z = 0;
                    if (GetImmutable(x_index, y_index, 0).first > 0)
                    {
                        m.points.push_back(new_point);
                    }

                }
            }
            return m;
        }
    };

    class Chs : public VoxelGrid2D
    {
    public:
        Chs(double cell_size) :
            VoxelGrid2D(cell_size)
        {}

        visualization_msgs::Marker toVisualizationMsg() const
        {
            std_msgs::ColorRGBA color;
            color.a = 0.4;
            color.r = 1.0;
            return VoxelGrid2D::toVisualizationMsg(color);
        }


        
        // Obstacles2D::Obstacles occ;
        // Chs(Obstacles2D::Obstacles o) :
        //     occ(o)
        // {}
        
        std::shared_ptr<Obstacles2D::Obstacle> sampleFrom(std::mt19937 &rng) const
        {
            int num_occupied = 0;
            std::cout << "Sampling...\n";

            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    if(GetImmutable(x_index, y_index, 0).first)
                    {
                        num_occupied++;
                    }
                }
            }
            std::cout << "num_occupied: " << num_occupied << "\n";

            std::uniform_int_distribution<int> distribution(1, num_occupied);
            int selected = distribution(rng);
            std::cout << "Sampled voxed " << selected << "\n";

            num_occupied = 0;
            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    if(GetImmutable(x_index, y_index, 0).first)
                    {
                        num_occupied++;
                        if(num_occupied == selected)
                        {
                            double d = GetCellSizes().x()/2;
                            const Eigen::Vector4d location = GridIndexToLocation(x_index, y_index, 0.0);

                            double x = location(0);
                            double y = location(1);

                            return std::make_shared<Obstacles2D::Rect>(x-d,y-d,x+d,y+d);
                        }
                    }
                }
            }
            assert(false && "Did not find randomly sampled voxel");

        }

        double getLikelihood(const VoxelGrid2D& sv) const
        {
            return 1.0 - intersectFraction(sv);
        }
    };

    class ChsBelief : public Belief
    {
    public:
        GraphD graph;
        Location cur;
        std::vector<Chs> chss;
        double resolution;
        VoxelGrid2D free_space;
        double robot_width;

    public:
        ChsBelief(GraphD graph, Location cur, double resolution, double robot_width) :
            graph(graph), cur(cur), resolution(resolution), free_space(resolution),
            robot_width(robot_width)
        {
        }

        virtual std::unique_ptr<Belief> clone() const override
        {
            std::unique_ptr<ChsBelief> b = std::make_unique<ChsBelief>(graph, cur, resolution, robot_width);
            b->chss = chss;
            b->free_space = free_space;
            return b;
        }

        virtual std::unique_ptr<State> sample(std::mt19937 &rng) const override
        {
            Obstacles2D::Obstacles o;
            for(const auto& chs: chss)
            {
                o.obs.push_back(chs.sampleFrom(rng));
            }
            std::cout << "Made obstacle state with " << o.obs.size() << " obstacles\n";
            return std::make_unique<ObstacleState>(&graph, cur, o);
        }

        virtual double getLikelihood(Observation obs) const
        {
            VoxelGrid2D sv(resolution);
            addFreeSpace(sv, obs);
            
            double p = 1.0;
            for(const auto& chs: chss)
            {
                p *= chs.getLikelihood(sv);
            }
            return p;
        }

        void addRobot(VoxelGrid2D& grid, std::vector<double> q) const
        {
            for(double x = q[0] - robot_width/2; x < q[0] + robot_width/2; x += resolution)
            {
                for(double y = q[1] - robot_width/2; y < q[1] + robot_width/2; y += resolution)
                {
                    grid.set(x, y, true);
                }
            }
        }

        void addFreeSpace(VoxelGrid2D& grid, Observation obs) const
        {
            auto from = graph.getNode(obs.from).getValue();
            auto to = graph.getNode(obs.to).getValue();

            for(double ratio = 0; ratio<obs.blockage; ratio += 0.1)
            {
                addRobot(grid, EigenHelpers::Interpolate(from, to, ratio));
            }
        }

        void addChs(Observation obs)
        {
            Chs chs(resolution);
            auto from = graph.getNode(obs.from).getValue();
            auto to = graph.getNode(obs.to).getValue();

            addRobot(chs, EigenHelpers::Interpolate(from, to, obs.blockage + 0.1));
            chss.push_back(chs);
        }

        virtual void update(Observation obs)
        {
            addFreeSpace(free_space, obs);
            if(!obs.succeeded())
            {
                addChs(obs);
            }

            for(auto& chs: chss)
            {
                chs.subtract(free_space);
            }
        }

        void viz(GraphVisualizer &viz) const override
        {
            visualization_msgs::MarkerArray ma;
            for(int i=0; i<chss.size(); i++)
            {
                visualization_msgs::Marker m = chss[i].toVisualizationMsg();
                m.ns = "chs " + std::to_string(i);
                ma.markers.push_back(m);
            }
            viz.obs_pub.publish(ma);

            std_msgs::ColorRGBA free_color;
            free_color.a = 0.3;
            free_color.b = 0.8;
            visualization_msgs::Marker free = free_space.toVisualizationMsg(free_color);
            free.ns = "Known free";
            visualization_msgs::MarkerArray free_arr;
            free_arr.markers.push_back(free);
            viz.obs_pub.publish(free_arr);
        }

        virtual std::string getName() const override
        {
            return "CHS";
        }
        
    };
}
#endif
