
#include "strategies/myopic_strategies.hpp"
#include "strategies/optimistic_rollout.hpp"
#include "strategies/pareto_cost.hpp"
#include "scenarios/obstacle_scenario.hpp"
#include "scenarios/predefined.hpp"
#include "scenarios/trap.hpp"
#include "scenarios/many_boxes.hpp"
#include "beliefs/chs.hpp"
#include "player.hpp"
#include "ros/ros.h"
#include "arc_utilities/timing.hpp"

using namespace BTP;

typedef std::function<std::shared_ptr<Scenario>(std::mt19937&)> ScenarioFactory;
// typedef std::function<std::shared_ptr<Strategy>(void)> StrategyFactory;


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
    player.run(scenario, strategy, 0.1);

    std::string filename = scenario.getName() + "_" + strategy.getName() + "_" +
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
    AverageOverClairvoyance strat(scenario_ptr->getGraph(), scenario_ptr->goal, scenario_ptr->getPrior());
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
    OptimisticWithPrior strat(scenario_ptr->getGraph(), scenario_ptr->goal, scenario_ptr->getPrior());
    test(*scenario_ptr, strat);
}

void test6(ScenarioFactory fac)
{
    //Pareto Cost with bayesian belief
    // const std::vector<double> p_weights{0.01, 0.1, 1.0, 10.0, 100};
    const std::vector<double> p_weights{0.01, 1.0, 100};
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
    const std::vector<double> p_weights{0.01, 1.0, 100};
    for(const auto w: p_weights)
    {
        rng.seed(seed);
        auto scenario_ptr = fac(rng);
        ChsBelief chsb = ChsBelief(scenario_ptr->getGraph(), scenario_ptr->getLocation(), 0.01, 0.1);
        ParetoCost strat(scenario_ptr->getGraph(), scenario_ptr->goal, chsb, w);
        test(*scenario_ptr, strat);
    }
}


std::vector<ScenarioFactory> getAllScenarios()
{
    std::vector<ScenarioFactory> f;
    f.push_back([](std::mt19937& rng) { return std::make_shared<SparseSingleWallScenario>(rng);});
    f.push_back([](std::mt19937& rng) { return std::make_shared<DenseSingleWallScenario>(rng);});
    return f;
}


void testAll()
{
    seed = time(0);

    for(auto scenario_factory: getAllScenarios())
    {
        test1(scenario_factory);
        test2(scenario_factory);
        test3(scenario_factory);
        test4(scenario_factory);
        test5(scenario_factory);
        test6(scenario_factory);
        test7(scenario_factory);
    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_trials");
    testAll();
}
