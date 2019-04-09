#include "scenarios/predefined.hpp"
#include "strategies/myopic_strategies.hpp"
#include "strategies/optimistic_rollout.hpp"
#include "strategies/pareto_cost.hpp"
#include "scenarios/obstacle_scenario.hpp"
#include "beliefs/chs.hpp"
#include "player.hpp"
#include "ros/ros.h"
#include "arc_utilities/timing.hpp"

using namespace BTP;

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


// typedef std::function<std::shared_ptr<Scenario>(void)> ScenarioFactory;
// typedef std::function<std::shared_ptr<Strategy>(void)> StrategyFactory;


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
    player.run(scenario, strategy, 0.5);

    std::string filename = scenario.getName() + "_" + strategy.getName() + "_" +
        arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 100);
}


void test1()
{
    rng.seed(seed);
    ManyPossibleWallsScenario scenario(rng);
    OmniscientStrategy strat(scenario.true_state, scenario.goal);
    test(scenario, strat);
}

void test2()
{
    rng.seed(seed);
    ManyPossibleWallsScenario scenario(rng);
    OptimisticStrategy strat(scenario.getGraph(), scenario.goal);
    test(scenario, strat);
}

void test3()
{
    rng.seed(seed);
    ManyPossibleWallsScenario scenario(rng);
    AverageOverClairvoyance strat(scenario.getGraph(), scenario.goal, scenario.bel);
    test(scenario, strat);
}

void test4()
{
    rng.seed(seed);
    ManyPossibleWallsScenario scenario(rng);
    OptimisticRollout strat(scenario.getGraph(), scenario.goal, scenario.bel);
    test(scenario, strat);
}

void test5()
{
    rng.seed(seed);
    ManyPossibleWallsScenario scenario(rng);
    OptimisticWithPrior strat(scenario.getGraph(), scenario.goal, scenario.bel);
    test(scenario, strat);
}

void test6()
{
    //Pareto Cost with bayesian belief
    const std::vector<double> p_weights{0.01, 0.1, 1.0, 10.0, 100};
    for(const auto w: p_weights)
    {
        rng.seed(seed);
        ManyPossibleWallsScenario scenario(rng);
        ParetoCost strat(scenario.getGraph(), scenario.goal, scenario.bel, w);
        test(scenario, strat);
    }
}

void test7()
{
    const std::vector<double> p_weights{0.01, 0.1, 1.0, 10.0, 100};
    for(const auto w: p_weights)
    {
        rng.seed(seed);
        ManyPossibleWallsScenario scenario(rng);
        ChsBelief chsb = ChsBelief(scenario.getGraph(), scenario.getLocation(), 0.01, 0.1);
        ParetoCost strat(scenario.getGraph(), scenario.goal, chsb, w);
        test(scenario, strat);
    }
}




void testAll()
{
    seed = time(0);
    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    test7();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_trials");
    testAll();
}
