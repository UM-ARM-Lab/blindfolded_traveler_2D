
#include "strategies/myopic_strategies.hpp"
#include "strategies/optimistic_rollout.hpp"
#include "strategies/pareto_cost.hpp"
#include "scenarios/obstacle_scenario.hpp"
#include "scenarios/predefined.hpp"
#include "scenarios/trap.hpp"
#include "scenarios/many_boxes.hpp"
#include "beliefs/chs.hpp"
#include "beliefs/indep_edge_belief.hpp"
#include "player.hpp"
#include "ros/ros.h"
#include "arc_utilities/timing.hpp"

using namespace BTP;

typedef std::function<std::shared_ptr<Scenario>(std::mt19937&)> ScenarioFactory;
// typedef std::function<std::shared_ptr<Strategy>(void)> StrategyFactory;
typedef std::function<std::unique_ptr<Belief>(const Belief&)> BeliefGenerator;


// std::vector<std::function<std::shared_ptr<Scenario>(void)>> getScenarioFactories()
// {
//     std::vector<std::function<std::shared_ptr<Scenario>(void)>> factories;
//     factories.push_back([](){ return std::make_shared<WallObstacleScenario>();});
//     return factories;
// }

// std::vector<std::function<std::shared_ptr<Strategy>(void)>> getStrategyFactories(Scenario& scenario)
// {
//     std::vector<std::function<std::shared_ptr<Strategy>(void)>> factories;
//     factories.push_back([&](){ return std::make_shared<>(scenario.prior);});
//     return factories;
// }




// std::vector<std::pair<ScenarioFactory, StrategyFactory>> getTestCases()
// {
//     std::vector<std::pair<ScenarioFactory, StrategyFactory>> f;
//     f.push_back(std::make_pair([](){ return std::make_shared<WallObstacleScenario>()},
//                                [](){ return std::make_shared<OptimisticStrategy>()}));
//     return f;
// }

static std::mt19937 rng;
static double seed;

void test(Scenario &scenario, Strategy &strategy)
{
    PROFILE_REINITIALIZE(0,0);
    ros::NodeHandle n;
    Player player(n);
    ros::Duration(1).sleep(); //Sleep to allow publishers to connect
    player.run(scenario, strategy, 0.18);

    std::string filename = scenario.getName() + "_" + strategy.getName() + "_" +
        strategy.getBeliefName() + "_" +
        arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 100);
}


void test1(ScenarioFactory fac)
{
    rng.seed(seed);
    // SingleWallScenario scenario(rng);
    auto scenario_ptr = fac(rng);
    OmniscientStrategy strat(scenario_ptr->getState(), scenario_ptr->goal);
    test(*scenario_ptr, strat);
}

void test2(ScenarioFactory fac)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    OptimisticStrategy strat(scenario_ptr->getGraph(), scenario_ptr->goal);
    test(*scenario_ptr, strat);
}

void test3(ScenarioFactory fac)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    OptimisticWithPrior strat(scenario_ptr->getGraph(), scenario_ptr->goal, scenario_ptr->getPrior());
    test(*scenario_ptr, strat);
}


void test4(ScenarioFactory fac)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    OptimisticRollout strat(scenario_ptr->getGraph(), scenario_ptr->goal, scenario_ptr->getPrior());
    test(*scenario_ptr, strat);
}

void test5(ScenarioFactory fac)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    AverageOverClairvoyance strat(scenario_ptr->getGraph(), scenario_ptr->goal, scenario_ptr->getPrior());
    test(*scenario_ptr, strat);
}

void test6(ScenarioFactory fac)
{
    //Pareto Cost with bayesian belief
    // const std::vector<double> p_weights{0.01, 0.1, 1.0, 10.0, 100};
    const std::vector<double> p_weights{1.0};
    for(const auto w: p_weights)
    {
        rng.seed(seed);
        auto scenario_ptr = fac(rng);
        ParetoCost strat(scenario_ptr->getGraph(), scenario_ptr->goal, scenario_ptr->getPrior(), w);
        test(*scenario_ptr, strat);
    }
}

void test7(ScenarioFactory fac)
{
    //Pareto Cost with CHS belief
    // const std::vector<double> p_weights{0.01, 0.1, 1.0, 10.0, 100};
    const std::vector<double> p_weights{1.0};
    for(const auto w: p_weights)
    {
        rng.seed(seed);
        auto scenario_ptr = fac(rng);
        ChsBelief chsb = ChsBelief(scenario_ptr->getGraph(), scenario_ptr->getLocation(), 0.01, robot_width);
        ParetoCost strat(scenario_ptr->getGraph(), scenario_ptr->goal, chsb, w);
        test(*scenario_ptr, strat);
    }
}

void testOptimistic(ScenarioFactory fac, BeliefGenerator bg)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    OptimisticWithPrior strat(scenario_ptr->getGraph(), scenario_ptr->goal, *bg(scenario_ptr->getPrior()));
    test(*scenario_ptr, strat);
}

void testHOP(ScenarioFactory fac, BeliefGenerator bg)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    AverageOverClairvoyance strat(scenario_ptr->getGraph(), scenario_ptr->goal, *bg(scenario_ptr->getPrior()));
    test(*scenario_ptr, strat);
}

void testORO(ScenarioFactory fac, BeliefGenerator bg)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    OptimisticRollout strat(scenario_ptr->getGraph(), scenario_ptr->goal, *bg(scenario_ptr->getPrior()));
    test(*scenario_ptr, strat);
}

void testCollisionMeasure(ScenarioFactory fac, BeliefGenerator bg)
{
    rng.seed(seed);
    auto scenario_ptr = fac(rng);
    ParetoCost strat(scenario_ptr->getGraph(), scenario_ptr->goal, *bg(scenario_ptr->getPrior()), 1.0);
    test(*scenario_ptr, strat);
}


std::vector<ScenarioFactory> getAllScenarios()
{
    std::vector<ScenarioFactory> f;
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseSingleWallScenario>(rng, 0.2);});
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseSingleWallScenario>(rng, 0.1);});
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseSingleWallScenario>(rng, 0.5);});
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseSingleWallScenario>(rng, 1.0);});
    
    // f.push_back([](std::mt19937& rng) { return std::make_shared<DenseSingleWallScenario>(rng);});
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseTrapScenario>(rng);});

    f.push_back([](std::mt19937& rng) { return std::make_shared<SparseManyBoxesScenario>(rng, 0.05);});
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseManyBoxesScenario>(rng, 0.1);});
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseManyBoxesScenario>(rng, 0.3);});
    // f.push_back([](std::mt19937& rng) { return std::make_shared<SparseManyBoxesScenario>(rng, 0.05, std::vector<double>{0.3, -0.3});});
    f.push_back([](std::mt19937& rng) { return std::make_shared<SparseManyBoxesScenario>(rng, 0.3, std::vector<double>{0.3, -0.3});});
    return f;
}

std::vector<BeliefGenerator> getAllBeliefs()
{
    std::vector<BeliefGenerator> bg;
    bg.push_back([](const Belief& b) { return b.clone();});
    bg.push_back([](const Belief& b) { return IndepEdgeBelief(dynamic_cast<const ObstacleBelief&>(b)).clone(); });
    bg.push_back([](const Belief& b) {
            const ObstacleBelief& ob = dynamic_cast<const ObstacleBelief&>(b);
            return ChsBelief(ob.graph, ob.cur, 0.01, robot_width).clone();
        });
    return bg;
}



void testAll()
{
    // seed = time(0);
    seed = 0; //hardcode seed so results are repeatable

    for(auto scenario_factory: getAllScenarios())
    {
        for(auto beliefs: getAllBeliefs())
        {
            testOptimistic(scenario_factory, beliefs);
            arc_helpers::WaitForInput();
            testHOP(scenario_factory, beliefs);
            arc_helpers::WaitForInput();
            testORO(scenario_factory, beliefs);
            arc_helpers::WaitForInput();
            testCollisionMeasure(scenario_factory, beliefs);
            arc_helpers::WaitForInput();
        }
        // test1(scenario_factory); //Omniscient
        // test2(scenario_factory); //Optimistic
        // test3(scenario_factory); //Optimistic With Prior
        // test4(scenario_factory); //Optimistic Rollout
        // test5(scenario_factory); //Average Over Clairvoyance
        // test6(scenario_factory); //Pareto Cost w/prior
        // test7(scenario_factory); //Pareto Cost CHS
    }

}

void testForVideo()
{
    seed = 0;
    auto sfs = getAllScenarios();

    test1(sfs[0]); //Omniscient
    test2(sfs[0]); //Optimistic

    for(auto sf: sfs)
        test3(sf); //Optimistic With Prior
    for(auto sf: sfs)
        test4(sf); //Optimistic Rollout
    for(auto sf: sfs)
        test5(sf); //Average Over Clairvoyance
    for(auto sf: sfs)
        test6(sf); //Pareto Cost w/prior
    
    test7(sfs[0]); //Pareto Cost CHS

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_trials");
    testAll();
    // testForVideo();
}
