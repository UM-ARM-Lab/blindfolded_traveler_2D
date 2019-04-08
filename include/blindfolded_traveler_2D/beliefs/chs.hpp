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

        bool get(double x, double y)
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

        double intersectFraction(const VoxelGrid2D& other)
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

        double getLikelihood(Observation obs) const
        {
            throw std::logic_error("Not implemented");
            return 1.0; //TODO: return likelihood based on CHS
        }
    };

    class ChsBelief : public Belief
    {
    public:
        GraphD graph;
        Location cur;
        std::vector<Chs> chss;

    public:
        ChsBelief(GraphD graph, Location cur) :
            graph(graph), cur(cur)
        {
        }

        virtual std::unique_ptr<Belief> clone() const override
        {
            std::unique_ptr<ChsBelief> b = std::make_unique<ChsBelief>(graph, cur);
            b->chss = chss;
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
            double p = 1.0;
            for(const auto& chs: chss)
            {
                p *= chs.getLikelihood(obs);
            }
            return p;
        }

        void addChs(Observation obs)
        {
            throw std::logic_error("Not implemented");
            //Todo make chs based on observation;
            // Obstacles2D::Obstacles o;
            // o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.5, 0.5, 0.7, 0.7));
            // chss.push_back(Chs(o));
        }

        virtual void update(Observation obs)
        {
            if(!obs.succeeded())
            {
                addChs(obs);
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
        }
    };
}
#endif
